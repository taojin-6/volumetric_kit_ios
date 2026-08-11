// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

import ARKit
import UIKit

/// Drives the renderer from a `CADisplayLink` and shows what it reports.
///
/// Swift owns the view, the display link, and the lifecycle; it never touches
/// Vulkan. Everything below `VolumetricRenderer` is C++ on the other side of the
/// Objective-C++ bridge. When the ARSession lands it joins *this* layer — Swift
/// configures it and hands frames across the same seam.
final class ScannerViewController: UIViewController {
  private var metalView: MetalView!
  private var renderer: VolumetricRenderer?
  private let arSession = ARSessionController()
  private var displayLink: CADisplayLink?
  private let statusLabel = UILabel()
  private var cameraGestures: CameraGestureController?

  /// Whether the capture + fuse + draw loop is running, so a resume cannot
  /// start a second ARSession or a second fuse thread and a suspend cannot
  /// wind the same one down twice.
  private var isCapturing = false
  /// On screen and foregrounded are tracked separately because only one of them
  /// has a view callback. `viewDidDisappear` does **not** fire on backgrounding,
  /// which is exactly how a backgrounded scan kept fusing: the display link
  /// suspends on its own, but the fuse thread is a plain `std::thread` that goes
  /// on submitting recon work — Metal submits from a backgrounded process, which
  /// iOS terminates the app for.
  private var isOnScreen = false
  private var isBackgrounded = false
  /// Bring-up runs off the main thread now, so this is what stops a second
  /// layout pass from starting a second one while the first is in flight.
  private var bringUpInFlight = false
  /// Set while `suspend()`'s drain is finishing on a background queue. `resume`
  /// refuses to start a second loop on top of one still winding down.
  private var teardownInFlight = false
  /// Set once a frame has failed fatally. The loop is not restarted afterwards:
  /// a `VK_ERROR_DEVICE_LOST` does not heal, and resuming onto the lost device
  /// would only spin the fuse thread against it.
  private var renderFailed = false

  /// Everything that must hold before the loop may run.
  private var isRunnable: Bool {
    isOnScreen && !isBackgrounded && !renderFailed
  }

  private var lastFPSUpdate = CFAbsoluteTimeGetCurrent()
  private var framesSinceUpdate = 0
  private var fps = 0.0

  /// The part of the read-out that is frozen once the renderer is up: the GPU,
  /// the negotiated API version, and how the one shared device was carved up.
  /// Cached rather than re-read per tick -- each of those properties crosses the
  /// bridge to rebuild a C++ string or re-issue
  /// `vkGetPhysicalDeviceProperties`, and the display link asks 60 times a
  /// second for a value that cannot change.
  private var deviceSummary = ""

  override func viewDidLoad() {
    super.viewDidLoad()
    view.backgroundColor = .black

    metalView = MetalView(frame: view.bounds)
    metalView.autoresizingMask = [.flexibleWidth, .flexibleHeight]
    view.addSubview(metalView)

    // Attached here, not after bring-up: the recognizers belong to the view,
    // which exists now, and they hold no camera until `renderer` is handed over
    // below. Installing them late would mean touches during the first frames
    // land on nothing.
    cameraGestures = CameraGestureController(attachingTo: metalView)

    // The retry path for bring-up. A first layout that reports a zero dimension
    // leaves nothing to build a swapchain from, and `viewDidAppear` does not
    // fire twice -- so without a subscriber here one bad first layout (iPad
    // multitasking, a transition mid-appearance) meant a black screen and a red
    // label for the rest of the process.
    metalView.onDrawableSizeChange = { [weak self] _ in
      self?.startRendererIfNeeded()
    }

    // Backgrounding is the trigger `viewDidDisappear` cannot provide; see
    // `isOnScreen`. Registered here rather than at bring-up so the app can be
    // backgrounded while the renderer is still coming up.
    let center = NotificationCenter.default
    center.addObserver(
      self, selector: #selector(appDidEnterBackground),
      name: UIApplication.didEnterBackgroundNotification, object: nil)
    center.addObserver(
      self, selector: #selector(appWillEnterForeground),
      name: UIApplication.willEnterForegroundNotification, object: nil)

    statusLabel.numberOfLines = 0
    statusLabel.textColor = .white
    statusLabel.font = .monospacedSystemFont(ofSize: 12, weight: .regular)
    statusLabel.translatesAutoresizingMaskIntoConstraints = false
    view.addSubview(statusLabel)
    NSLayoutConstraint.activate([
      statusLabel.topAnchor.constraint(
        equalTo: view.safeAreaLayoutGuide.topAnchor, constant: 12),
      statusLabel.leadingAnchor.constraint(
        equalTo: view.safeAreaLayoutGuide.leadingAnchor, constant: 12),
      statusLabel.trailingAnchor.constraint(
        lessThanOrEqualTo: view.safeAreaLayoutGuide.trailingAnchor,
        constant: -12),
      // Bounded, where it previously had no bottom at all. The overlay grew
      // past the safe area on a landscape iPhone — verified only on an iPad,
      // where it fits — and an unconstrained label simply ran off the screen,
      // silently, taking whatever was last in the string with it. The failure
      // banners now come first for that reason (see `fusionSummary`), so what
      // this clips is the least load-bearing end.
      //
      // A scroll view would make the tail reachable instead of merely bounded,
      // and was rejected on purpose: sized to the safe area it would sit over
      // `metalView` and swallow the orbit/pan/zoom recognizers, trading a debug
      // overlay's tail for the camera controls.
      statusLabel.bottomAnchor.constraint(
        lessThanOrEqualTo: view.safeAreaLayoutGuide.bottomAnchor,
        constant: -12),
    ])
  }

  override func viewDidAppear(_ animated: Bool) {
    super.viewDidAppear(animated)
    isOnScreen = true
    // Bring up on first appearance, not in viewDidLoad: the layer has no valid
    // drawableSize until the view has been laid out in a window. Bring-up is
    // asynchronous, so the loop is started by whichever of the two finishes
    // last -- `resume` here if the renderer is already up, or `adopt` if it
    // lands later.
    startRendererIfNeeded()
    resume()
  }

  override func viewDidDisappear(_ animated: Bool) {
    super.viewDidDisappear(animated)
    isOnScreen = false
    suspend()
  }

  override func didReceiveMemoryWarning() {
    super.didReceiveMemoryWarning()
    // Forwarded, not acted on. This is the only notice the OS gives before
    // jetsam, and until now the app discarded it: everything on the memory row
    // is a poll at this view's own tick rate, which cannot see a spike shorter
    // than its interval, and the kill itself reports nothing at all. Recording
    // it is what makes "the scan was warned, then died" a readable sequence
    // rather than a process that vanished.
    //
    // Deliberately not a wind-down. `suspend()` would stop the session and the
    // fuse loop, which is a policy decision about what to do with a scan in
    // progress -- and one that would fire on a warning the app may well survive.
    // See VolumetricRenderer.noteMemoryWarning().
    renderer?.noteMemoryWarning()
  }

  @objc private func appDidEnterBackground() {
    isBackgrounded = true
    suspend()
  }

  @objc private func appWillEnterForeground() {
    isBackgrounded = false
    // The retry for a bring-up declined while backgrounded. Without it, an app
    // launched straight into the background comes forward with no renderer and
    // nothing left to trigger one: `viewDidAppear` does not fire twice, and the
    // drawable size has not changed, so `onDrawableSizeChange` does not either.
    startRendererIfNeeded()
    resume()
  }

  /// Start capture, fusion and the draw loop, if everything is ready for them.
  ///
  /// Idempotent and precondition-checked rather than called only from places
  /// that "know" it is safe: the three things that gate it -- appearance,
  /// foreground, and an asynchronous bring-up -- complete in no fixed order.
  private func resume() {
    guard isRunnable, !isCapturing, !teardownInFlight, let renderer else {
      return
    }
    isCapturing = true
    // Before the first frame: ARKit poses are in the sensor's frame, which does
    // not turn with the interface, so the renderer has to be told which way the
    // viewport is facing or the scan renders on its side.
    renderer.viewOrientation = currentViewOrientation()
    arSession.start()
    // Fusion pulls from the capture bridge on its own thread; the render loop
    // only ever picks up whatever mesh it has published.
    renderer.startFusion(with: arSession.capture)
    startDisplayLink()
  }

  /// Wind the loop down, in the one order that is safe.
  ///
  /// Signalling is synchronous; waiting is not, and the split is the point.
  /// `stopFusion` joins the fuse thread, whose `fusing` flag is tested only
  /// *between* iterations — so the join waits out the whole current one, which
  /// at worst is a preemptive resize, a bounded grow loop building grown
  /// attribute arrays beside the old ones, a re-allocate, an integrate, and an
  /// extract that may free and reallocate the mesh arena, every dispatch ending
  /// in a `vkWaitForFences(..., UINT64_MAX)`. `waitIdle` then adds a
  /// `vkQueueWaitIdle` on both queues — on the render-failure path, against a
  /// GPU that has just hung.
  ///
  /// This runs from a `UIApplication` notification handler and from inside the
  /// display-link callback, so doing all of that inline blocked the main thread
  /// for hundreds of milliseconds routinely and seconds on a grow-heavy
  /// iteration: the 0x8badf00d watchdog kill that moving bring-up off-main
  /// exists to avoid, and likeliest in exactly the large-scan state where a
  /// user reaches for the home gesture.
  ///
  /// Everything that stops new work *starting* still happens before this
  /// returns — the display link, ARKit, and the fuse thread's flag — so nothing
  /// is submitting by the time the app is backgrounded. Only the waiting moves.
  private func suspend() {
    guard isCapturing else { return }
    isCapturing = false
    stopDisplayLink()
    // Fusion first, and explicitly. It submits recon work on the same queue the
    // drain below empties, so a fuse thread still running would enqueue behind
    // the teardown -- and waitIdle does not stop it: it waits on the queues and
    // never touches the thread. Until stopFusion returns, the thread is also
    // still dereferencing the capture handle.
    //
    // This half only clears the flag the loop tests between iterations, which
    // is what makes it safe to call on the main thread. The join that turns it
    // into a guarantee is below.
    renderer?.beginStopFusion()
    arSession.pause()
    guard let renderer else { return }
    teardownInFlight = true
    // `renderer` is captured strongly, deliberately: the drain must not race a
    // release of the object whose queues it is draining.
    DispatchQueue.global(qos: .userInitiated).async {
      renderer.stopFusion()
      renderer.waitIdle()
      DispatchQueue.main.async { [weak self] in
        guard let self else { return }
        self.teardownInFlight = false
        // Reconciled rather than assumed: the app may have been foregrounded,
        // or the view re-appeared, while the drain was running. `resume` is
        // precondition-checked, so this is a no-op whenever it should not run.
        self.resume()
      }
    }
  }

  override func viewWillTransition(
    to size: CGSize, with coordinator: UIViewControllerTransitionCoordinator
  ) {
    super.viewWillTransition(to: size, with: coordinator)
    // In the completion, not alongside it: `interfaceOrientation` still reports
    // the old value until the transition finishes, so reading it early would
    // pin the camera one quarter turn behind the screen.
    coordinator.animate(alongsideTransition: nil) { [weak self] _ in
      guard let self else { return }
      self.renderer?.viewOrientation = self.currentViewOrientation()
    }
  }

  /// The viewport's turn relative to ARKit's sensor-fixed camera basis.
  ///
  /// From the window scene rather than `UIDevice.orientation`: the device can
  /// be face-up or face-down, and what the render camera needs is where the
  /// *interface* ended up, which is what the app's supported orientations
  /// actually constrain.
  private func currentViewOrientation() -> VolumetricViewOrientation {
    switch view.window?.windowScene?.interfaceOrientation {
    case .portrait: return .portrait
    case .portraitUpsideDown: return .portraitUpsideDown
    case .landscapeLeft: return .landscapeLeft
    case .landscapeRight: return .landscapeRight
    // No window yet, or `.unknown`. Portrait is how the app launches.
    default: return .portrait
    }
  }

  /// The turn the renderer is actually holding, for the read-out.
  ///
  /// Four different faults all present on screen as "the scan is rotated": a
  /// wrong zero, a wrong sign, a value that went stale across a rotation, and a
  /// renderer that was never told at all and is still on its default. The fix
  /// differs for each, and the first two are indistinguishable in portrait —
  /// which is why this line exists, and why the check recorded on
  /// `kSensorBasisOrientation` asks for a landscape orientation too. The raw
  /// value is printed alongside the name because the mapping is argued in raw
  /// values.
  ///
  /// Switched over the cases rather than indexed by `rawValue`: the parallel
  /// array this replaces encoded the enum's ordering a second time, so
  /// reordering the enum would have relabelled precisely the read-out whose one
  /// job is to say which value the renderer holds — and it would have done it
  /// silently, while VolumetricRenderer.mm's static_asserts caught the same
  /// edit at build time. One less place for the two to disagree.
  private func orientationName(_ orientation: VolumetricViewOrientation)
    -> String
  {
    let name: String
    switch orientation {
    case .landscapeLeft: name = "landscape-left"
    case .portrait: name = "portrait"
    case .landscapeRight: name = "landscape-right"
    case .portraitUpsideDown: name = "upside-down"
    @unknown default: name = "invalid"
    }
    return "\(name) (\(orientation.rawValue))"
  }

  private func startRendererIfNeeded() {
    guard renderer == nil, !bringUpInFlight else { return }
    // Never while backgrounded. Bring-up is `vkCreateSwapchainKHR`, a blocking
    // one-shot `upload_texture` (a real `vkQueueSubmit` plus a fence wait) and
    // `Fusion::start`'s 48 MB commit -- Metal submits from a backgrounded
    // process, which iOS terminates the app for, and that is the whole reason
    // the state machine below exists. `suspend()` cannot cover this: it guards
    // on `isCapturing`, which bring-up has not set, and once the closure is
    // running there is nothing to cancel and no signal to block on. So the fix
    // is to not begin; `appWillEnterForeground` is the retry.
    //
    // A bring-up already in flight when the app backgrounds still runs to
    // completion -- it cannot be interrupted -- but it no longer starts the
    // loop afterwards, because `adopt` reconciles through `resume`.
    guard !isBackgrounded else { return }
    // BOTH dimensions. Height is the one that actually goes to zero -- MetalView
    // refuses to write a drawableSize with a zero in it, so a (W, 0) first
    // layout leaves the layer at (W * scale, 0), which passed a width-only guard
    // and then failed inside vkCreateSwapchainKHR.
    let size = metalView.metalLayer.drawableSize
    guard size.width > 0, size.height > 0 else { return }

    bringUpInFlight = true
    let layer = metalView.metalLayer
    // The layer itself crosses to another thread, where MoltenVK reads and sets
    // properties on it for the length of the swapchain build -- so the main
    // thread stops touching it until bring-up hands it back. Snapshotting
    // `drawableSize` above is not enough on its own; the object is what
    // crosses. See MetalView.defersLayerUpdates.
    metalView.defersLayerUpdates = true
    // Off the main thread, because this is not a constructor-shaped amount of
    // work: vkCreateInstance/Device, the pipeline creates, a blocking
    // upload_texture, and Fusion::start building the grid, integrator, marching
    // cubes and texturer -- a dozen or so SPIR-V -> MSL compiles against a cold
    // MoltenVK shader cache, plus a 48 MB zero-filled host-visible commit. On
    // first launch after install that is seconds, and seconds of blocked main
    // thread through launch is a 0x8badf00d watchdog kill. The smoke app moves
    // the same class of work off-main for the same reason.
    DispatchQueue.global(qos: .userInitiated).async {
      let outcome = Result { try VolumetricRenderer(layer: layer) }
      DispatchQueue.main.async { [weak self] in
        guard let self else { return }
        self.bringUpInFlight = false
        self.metalView.defersLayerUpdates = false
        switch outcome {
        case .success(let brought):
          self.adopt(brought)
        case .failure(let error):
          let message =
            "Renderer bring-up failed:\n\(error.localizedDescription)"
          self.statusLabel.text = message
          self.statusLabel.textColor = .systemRed
          // To stdout as well: this path never reaches updateStatus, so without
          // it a bring-up failure is visible only on the screen.
          print(message)
        }
      }
    }
  }

  /// Take ownership of a renderer that finished bring-up, on the main thread.
  private func adopt(_ brought: VolumetricRenderer) {
    renderer = brought
    cameraGestures?.renderer = brought
    deviceSummary = """
      \(brought.deviceName)
      Vulkan \(brought.apiVersion) via MoltenVK
      device    \(brought.sharedDeviceSummary)
      shared    \(brought.sharesOneDevice ? "yes - recon and gfx hold one VkDevice" : "NO - separate devices")
      """
    // The view may have appeared, or been backgrounded, or both, while bring-up
    // was in flight. `resume` is what reconciles that rather than assuming.
    resume()
  }

  private func startDisplayLink() {
    guard displayLink == nil else { return }
    // Through the proxy, never `target: self` -- see DisplayLinkProxy.
    let link = CADisplayLink(
      target: DisplayLinkProxy(owner: self),
      selector: #selector(DisplayLinkProxy.tick))
    link.add(to: .main, forMode: .common)
    displayLink = link
  }

  private func stopDisplayLink() {
    displayLink?.invalidate()
    displayLink = nil
  }

  fileprivate func tick() {
    guard let renderer else { return }
    do {
      try renderer.renderFrame(withDrawableSize: metalView.metalLayer.drawableSize)
    } catch {
      statusLabel.text = "Render failed:\n\(error.localizedDescription)"
      statusLabel.textColor = .systemRed
      // The whole loop, not just the draw. Stopping the display link alone left
      // `RendererImpl::fusing` true, so on a VK_ERROR_DEVICE_LOST the fuse
      // thread went on polling ARKit at 60 Hz and issuing allocate/integrate/
      // extract on the lost device -- each failing, each rewriting a last_error
      // nothing was reading any more -- spinning a core and holding the camera
      // and the volume until the user backgrounded the app.
      renderFailed = true
      suspend()
      return
    }
    // No poll here. `ICameraCapture` is single-consumer ("Poll from one
    // thread"), and its CapturedFrame is a non-owning view valid only until the
    // next poll -- so the bring-up placeholder that polled once per rendered
    // frame is now a second consumer racing the fuse thread for the same
    // buffers, and every frame it won was a frame fusion never saw.
    updateStatus(renderer)
  }

  private func updateStatus(_ renderer: VolumetricRenderer) {
    // Counted every tick; rebuilt at 2 Hz. The gate below already existed and
    // was spent only on `print`, so the whole overlay — a mutex acquisition, a
    // FusionStats copy, three string builds and a ~20-conversion snprintf
    // across the bridge, then a CoreText re-layout of the result — was being
    // redone 60 times a second on iPhone and up to 120 on a ProMotion iPad, for
    // content a human reads at walking pace. `deviceSummary` was cached for
    // exactly this reason; this is the same argument applied to the half that
    // does change, just not that fast.
    //
    // An equality check on the built string would not have worked: the text
    // embeds `framesPresented`, which moves every tick.
    framesSinceUpdate += 1
    let now = CFAbsoluteTimeGetCurrent()
    let elapsed = now - lastFPSUpdate
    guard elapsed >= 0.5 else { return }
    fps = Double(framesSinceUpdate) / elapsed
    framesSinceUpdate = 0
    lastFPSUpdate = now

    let size = metalView.metalLayer.drawableSize
    // Which camera is driving, and -- once the user is driving -- the way back.
    // A manual camera pointed away from the scan and a scan that stopped
    // producing geometry look identical on screen, so the mode is worth a line.
    let camera =
      renderer.followingDevice
      ? "following device"
      : String(
        format: "manual  %.2f m from pivot  (double-tap to follow)",
        renderer.cameraDistance)
    var text = """
      \(deviceSummary)
      drawable  \(Int(size.width)) x \(Int(size.height)) px
      orient    \(orientationName(renderer.viewOrientation))
      presented \(renderer.framesPresented)
      camera    \(camera)
      \(String(format: "%.0f", fps)) fps

      """

    if let reason = arSession.unsupportedReason {
      text += "ARKit: \(reason)"
    } else {
      if let failure = arSession.sessionError {
        // Alongside the counters, not instead of them: a session can fail and
        // recover, and replacing the read-out meant a single transient
        // interruption hid capture for the rest of the run.
        text += "ARKit session failed: \(failure)\n\n"
      }
      // Ahead of the counters, because it is the answer to "why is the frame
      // count rising and nothing appearing": frames arriving while ARKit does
      // not trust its own pose are withheld from fusion on purpose.
      let t = arSession.tracking
      let withheld =
        t.framesWithheld > 0 ? "  (\(t.framesWithheld) withheld)" : ""
      text += "tracking  \(t.description)\(withheld)\n\n"

      let s = arSession.capture.stats
      // Formatted up front rather than inside the literal: a `\`-continuation
      // inside a \(...) interpolation is not something the Swift parser accepts.
      let kept = String(format: "%.0f%%", s.confidence_kept * 100)
      let intrinsics = String(
        format: "fx %.1f  cx %.1f  cy %.1f", s.depth_fx, s.depth_cx, s.depth_cy)
      // What the buffer declared, not what we assumed. The matrix is separate
      // from the transfer and primaries -- it reconstructs chroma, they
      // describe the result -- so all three are shown rather than collapsed.
      //
      // `(refused)` means the buffer named an encoding the driver cannot
      // represent, so it dropped this frame's colour rather than fusing it
      // through a curve it had guessed at. The three names still show what was
      // read, which is the whole point: the alternative is colour quietly going
      // missing with nothing on screen saying why.
      let encoding = String(
        format: "%@ matrix -> %@ / %@%@",
        String(cString: s.color_matrix), String(cString: s.color_transfer),
        String(cString: s.color_primaries),
        s.color_declaration_refused
          ? "  (refused)" : (s.color_was_canonical ? "" : "  (converted)"))
      let position = String(
        format: "%.2f, %.2f, %.2f", s.position_x, s.position_y, s.position_z)
      // Colour split out from the total: the combined figure cannot answer a
      // question about either half, which is how a 35% win in the colour path
      // once read as a small loss.
      let convert = String(
        format: "%.1f ms  (colour %.2f)", s.convert_ms, s.color_convert_ms)
      let counts =
        "\(s.frames_submitted) in / \(s.frames_polled) polled"
        + " / \(s.frames_dropped) dropped / \(s.frames_rejected) rejected"
      text += """
        \(renderer.fusionSummary)

        ARKit capture
          frames    \(counts)
          depth     \(s.depth_width) x \(s.depth_height)  (\(kept) confident)
          colour    \(s.color_width) x \(s.color_height)
          encoding  \(encoding)
          intrinsic \(intrinsics)
          position  \(position) m
          convert   \(convert)
        """
    }
    statusLabel.text = text
    // Unconditional now: the 0.5 s gate at the top of this function is the same
    // cadence `shouldLog` used to carry, so the flag was tracking a rate the
    // whole body already runs at.
    //
    // Same text as the on-screen read-out, to stdout, so
    // `devicectl device process launch --console` can verify capture from the
    // build host instead of someone reading the screen.
    print(text)
  }

  deinit {
    NotificationCenter.default.removeObserver(self)
    // The run loop retains the link and the link retains its target, so a
    // controller released while capturing left it firing forever: 60 wake-ups a
    // second into a proxy whose owner is nil, with the renderer, the VkDevice,
    // the swapchain and the TSDF volume all still held behind it.
    // `stopDisplayLink` is otherwise reached only through `suspend`, which a
    // release does not go through -- and this method became reachable at all
    // only once DisplayLinkProxy broke the retain cycle that used to make it
    // unreachable, so the leak arrived with the fix for the other one.
    displayLink?.invalidate()
    displayLink = nil
  }
}

/// A weak trampoline between the display link and the controller.
///
/// `CADisplayLink` retains its target until `invalidate()`, and the controller
/// retains the link — so `target: self` is a retain cycle broken only by the one
/// code path that happens to call `stopDisplayLink()`. Any other release (a
/// root-view-controller swap, a scene teardown, a memory-pressure dismissal)
/// leaked the controller and everything under it: the VkDevice, the swapchain,
/// tens of megabytes of TSDF volume, and a fuse thread still polling a released
/// capture — because `deinit`, and so `-stopFusion` and `-dealloc`, would never
/// run. `VolumetricRenderer.h` documents `-dealloc` as the safety net that means
/// "a dropped renderer cannot leave the thread running"; the cycle is what made
/// that net unreachable.
///
/// The link retains the proxy and the proxy holds the controller weakly, so the
/// chain terminates: controller -> link -> proxy -> (weak) controller.
private final class DisplayLinkProxy: NSObject {
  private weak var owner: ScannerViewController?

  init(owner: ScannerViewController) {
    self.owner = owner
    super.init()
  }

  @objc func tick() {
    owner?.tick()
  }
}
