// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

import ARKit
import SwiftUI
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
  /// What the dashboard draws. Populated in `updateStatus`, at the display
  /// cadence -- the *history* it charts is sampled per fused frame on the
  /// fusion thread, which is a different rate on purpose (see DashboardModel).
  private let dashboard = DashboardModel()
  private var dashboardHost: UIHostingController<DashboardView>?
  /// When the view appeared, for the dashboard's elapsed clock. Wall clock
  /// rather than a fused-frame count: it is how long the person has been
  /// scanning, which is not the same as how much fusion got done.
  private let sessionStartedAt = CFAbsoluteTimeGetCurrent()
  /// How much of the safe area the dashboard occupies. The rest is the
  /// reconstruction, which is still the thing being judged -- a panel that
  /// covered it would make the numbers unfalsifiable by eye.
  private let dashboardHeightFraction: CGFloat = 0.55
  /// Active only while the dashboard is expanded; see the hosting setup.
  private var expandedHeight: NSLayoutConstraint?
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

    // The dashboard, hosted rather than laid out here.
    //
    // This replaces a UILabel fed a thirty-line monospace string. Two things
    // that cost us are gone with it: the string was rebuilt and re-laid-out by
    // CoreText on every tick until a 0.5 s gate was added to stop it, and it
    // had no bottom constraint, so it ran off a landscape iPhone silently and
    // took whatever was last in it. SwiftUI re-renders only what changed and
    // sizes itself inside the safe area.
    //
    // Pinned across the width and, expanded, 55% of the safe area: the rest of
    // the screen is the reconstruction, and the orbit/pan/zoom recognizers live
    // on `metalView` underneath.
    //
    // Which is the whole of what this comment used to claim, and it was not
    // true. The host is a sibling above `metalView`, so it took every touch in
    // its rectangle -- the camera was dead across the top half of the screen on
    // first launch, including the double-tap that refollows. Two things make it
    // true now: the container below claims only what SwiftUI actually
    // hit-tests, and `DashboardView` marks its background non-interactive so
    // there is something to fall through. Expanded, the scroll area still owns
    // its own rect -- it is a scrolling surface and a drag there should scroll
    // it -- so the reconstruction gets the lower 45% and, collapsed, all but
    // the headline.
    let host = UIHostingController(
      rootView: DashboardView(model: dashboard) { [weak self] expanded in
        // The height constraint follows the toggle. Collapsed, the body is a
        // plain VStack and sizes itself; expanded, it contains a ScrollView,
        // which has no intrinsic height and would otherwise keep whatever it
        // was last given.
        self?.expandedHeight?.isActive = expanded
      })
    host.view.backgroundColor = .clear
    host.view.translatesAutoresizingMaskIntoConstraints = false
    addChild(host)
    // Inside a passthrough container, which is what makes the sentence above
    // true rather than merely intended.
    //
    // The host is a sibling of `metalView` and sits above it in z-order, so any
    // touch it claims is one the camera recognizers never see -- and it is
    // pinned leading AND trailing across 55% of the safe area. Orbit, pan, zoom
    // and double-tap-to-refollow were all dead across the top half of the
    // screen, on first launch, with the panel's own comment claiming the
    // opposite. The container reports a touch as "inside" only where SwiftUI
    // actually hit-tests something -- a card, a chip, the chevron, the scroll
    // area -- so the panel's empty regions and its margins fall through to the
    // reconstruction underneath. The material background is explicitly
    // non-interactive in DashboardView for this to have anything to fall
    // through from.
    let container = PassthroughContainer()
    container.translatesAutoresizingMaskIntoConstraints = false
    container.addSubview(host.view)
    view.addSubview(container)
    NSLayoutConstraint.activate([
      host.view.topAnchor.constraint(equalTo: container.topAnchor),
      host.view.leadingAnchor.constraint(equalTo: container.leadingAnchor),
      host.view.trailingAnchor.constraint(equalTo: container.trailingAnchor),
      host.view.bottomAnchor.constraint(equalTo: container.bottomAnchor),
    ])
    host.didMove(toParent: self)
    dashboardHost = host
    // A DEFINITE size, which the first cut did not give it.
    //
    // A ScrollView has no intrinsic content height, so pinning top/leading and
    // leaving trailing and bottom as `lessThanOrEqualTo` left the host with
    // nothing forcing a size -- it collapsed toward its minimum and the grid
    // packed into a single narrow column. Width is pinned on both sides and
    // height is a fraction of the safe area, so the dashboard is as wide as the
    // screen allows and the reconstruction keeps the rest.
    NSLayoutConstraint.activate([
      container.topAnchor.constraint(
        equalTo: view.safeAreaLayoutGuide.topAnchor, constant: 10),
      container.leadingAnchor.constraint(
        equalTo: view.safeAreaLayoutGuide.leadingAnchor, constant: 10),
      container.trailingAnchor.constraint(
        equalTo: view.safeAreaLayoutGuide.trailingAnchor, constant: -10),
    ])
    expandedHeight = container.heightAnchor.constraint(
      equalTo: view.safeAreaLayoutGuide.heightAnchor,
      multiplier: dashboardHeightFraction)
    // Activated by the view's own onAppear, so the stored expanded/collapsed
    // choice decides it rather than this defaulting to one of them.
    expandedHeight?.isActive = false
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
  /// `kSensorBasisOrientation` (Core/ViewOrientation.hpp) asks for a landscape
  /// orientation too. The raw
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
          self.dashboard.failure = message
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
    // Cleared, because bring-up has a documented retry path: a first layout
    // reporting a zero dimension fails, `onDrawableSizeChange` tries again, and
    // the second attempt succeeds. Nothing used to clear this, so the banner
    // from the failed attempt stayed over a working dashboard for the rest of
    // the process -- a red alert reporting a state the app had recovered from.
    dashboard.failure = nil
    // The GPU as MoltenVK names it, above the machine it is part of. Vulkan
    // reports the *GPU* -- "Apple M5 GPU" -- which is not the same as knowing
    // which iPad this is: the model identifier is what a measurement in this
    // project's notes is attributed to, and every figure on the panel beneath
    // it is only meaningful against a named device.
    deviceSummary = """
      \(brought.deviceName)
      \(Self.hardwareSummary())
      Vulkan \(brought.apiVersion) via MoltenVK
      device    \(brought.sharedDeviceSummary)
      shared    \(brought.sharesOneDevice ? "yes - recon and gfx hold one VkDevice" : "NO - separate devices")
      """
    // The view may have appeared, or been backgrounded, or both, while bring-up
    // was in flight. `resume` is what reconciles that rather than assuming.
    resume()
  }

  /// The SoC side of the device identity: the model identifier and its core
  /// count.
  ///
  /// `utsname.machine` rather than `UIDevice.model`, which answers "iPad" on
  /// every iPad ever made. This one distinguishes the hardware a timing was
  /// taken on, which is the only form in which a timing means anything -- the
  /// extract cost this project quotes is a specific device's number.
  ///
  /// Installed RAM is deliberately absent: the Memory card already reports it,
  /// from the kernel reading that the gauges beside it come from, and putting a
  /// second copy here from a second source is how the two come to disagree.
  private static func hardwareSummary() -> String {
    var system = utsname()
    uname(&system)
    // NUL-terminated by uname, and read through the raw bytes because the field
    // arrives in Swift as a 256-tuple of CChar rather than as an array.
    let machine = withUnsafeBytes(of: &system.machine) { raw -> String in
      guard let base = raw.baseAddress else { return "unknown" }
      return String(cString: base.assumingMemoryBound(to: CChar.self))
    }
    let cores = ProcessInfo.processInfo.processorCount
    return "\(machine)  \(cores) cores"
  }

  /// The console transcript, rendered from the same lines the panel draws.
  ///
  /// The second of the two renderings `updateStatus` produces, and deliberately
  /// the only place that knows how the transcript is *shaped* -- the blank
  /// lines, the heading, the indent. Its inputs are already-formatted lines, so
  /// there is no figure it can format differently from the panel; that was the
  /// whole failure it replaces.
  ///
  /// Static and free of `self` so it cannot reach past its arguments and read a
  /// counter the panel was not given.
  ///
  /// @param summary  The fusion read-out, or nil to omit it entirely.
  private static func transcript(
    device: String, render: [String], fps: Double,
    session: [String], summary: String?, counters: [String]
  ) -> String {
    // Paragraphs, joined once. Appending "\n\n" at each site is what let the
    // unsupported-ARKit line lose its break while its two neighbours kept
    // theirs: a separator applied by the join cannot be forgotten at one site.
    //
    // This changes the output by exactly one blank line, after `fps`. The
    // header block used to end in the single newline its string literal
    // happened to carry, so it was the one paragraph in the transcript not
    // followed by a break -- the same per-site inconsistency, in the place it
    // was least visible. Uniform now, and that is the whole delta: every other
    // byte of all three arrangements is unchanged.
    var blocks = [
      """
      \(device)
      \(render.joined(separator: "\n"))
      \(String(format: "%.0f", fps)) fps
      """
    ]
    blocks += session
    if let summary {
      blocks.append(summary)
    }
    if !counters.isEmpty {
      // Indented under a heading, which is the one arrangement the panel does
      // not share: it has a card title and a table, and needs neither.
      blocks.append(
        "ARKit capture\n"
          + counters.map { "  " + $0 }.joined(separator: "\n"))
    }
    return blocks.joined(separator: "\n\n")
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
      dashboard.failure = "Render failed: \(error.localizedDescription)"
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
    // Read ONCE, here, and used for both the panel and the transcript.
    //
    // `ARSessionController.tracking` takes its lock per access and ARKit writes
    // at 60 Hz on sessionQueue, so reading it for the label and again for the
    // tone could straddle an update and render "limited (relocalizing)" in the
    // healthy green -- exactly what TrackingReport is a value type to prevent,
    // and its doc says so.
    let tracking = arSession.tracking
    let s = arSession.capture.stats
    // Read ONCE, here, and used for both the panel and the transcript -- the
    // same rule the two lines above follow, applied to the read-out.
    //
    // `dashboardSnapshot` exists for this. Building the panel from
    // `statSections` + `stageRows` + `frameHistory` + the memory properties took
    // five FusionStats copies and three task_info traps per tick, which the fuse
    // thread writes between: the headline meter could read 84% beside a Volume
    // card reading 86% with allocation stopped, and the pipeline bars could
    // belong to a frame the sections did not describe.
    //
    // `fusionSummary` was the last read still outside it, taken twenty lines
    // below this one -- its own FusionStats copy, its own task_info trap, and
    // the whole read-out built a second time on the main thread inside the
    // display-link callback. `.summary` is that text rendered from the rows
    // this snapshot carries, so the transcript and the panel beside it describe
    // one frame rather than two.
    let snapshot = renderer.dashboardSnapshot

    // Built as lines, once, then joined for stdout -- rather than appended to a
    // string the panel cannot read. The `Capture` card rendered an array
    // nothing ever assigned, so every figure below reached the transcript and
    // nothing else; and the two ARKit fault lines collapsed into a boolean, so
    // a device without LiDAR showed "Paused 00:00 / 0.00 M tris" indefinitely
    // with nothing on screen saying why.
    let renderLines = [
      "drawable  \(Int(size.width)) x \(Int(size.height)) px",
      "orient    \(orientationName(renderer.viewOrientation))",
      "presented \(renderer.framesPresented)",
      "camera    \(camera)",
    ]
    // The capture read-out, as two groups of lines built ONCE.
    //
    // Every figure below used to be written out twice -- appended to `text` for
    // stdout and appended again to `captureLines` for the panel -- which is the
    // defect Readout.hpp was created to remove on the bridge's side of the
    // seam, arrived at independently one layer up. The two copies had already
    // begun to differ: the unsupported-ARKit line reached the transcript
    // without the paragraph break its two neighbours got.
    //
    // So: one model, two renderings, and the renderings differ only in how they
    // *arrange* these lines. The panel draws both groups as one `Capture` card;
    // the transcript keeps the session lines above the fusion read-out and
    // indents the counters beneath a heading, because on a wall of text the
    // session's health is the thing worth reading first.
    //
    // Two groups rather than one, because that arrangement is the whole reason
    // the transcript could not simply join a single array.
    var sessionLines: [String] = []
    var counterLines: [String] = []

    if let reason = arSession.unsupportedReason {
      sessionLines.append("ARKit: \(reason)")
    } else {
      if let failure = arSession.sessionError {
        // Alongside the counters, not instead of them: a session can fail and
        // recover, and replacing the read-out meant a single transient
        // interruption hid capture for the rest of the run.
        sessionLines.append("ARKit session failed: \(failure)")
      }
      // Ahead of the counters, because it is the answer to "why is the frame
      // count rising and nothing appearing": frames arriving while ARKit does
      // not trust its own pose are withheld from fusion on purpose.
      let withheld =
        tracking.framesWithheld > 0
        ? "  (\(tracking.framesWithheld) withheld)" : ""
      sessionLines.append("tracking  \(tracking.description)\(withheld)")

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
      counterLines = [
        "frames    \(counts)",
        "depth     \(s.depth_width) x \(s.depth_height)  (\(kept) confident)",
        "colour    \(s.color_width) x \(s.color_height)",
        "encoding  \(encoding)",
        "intrinsic \(intrinsics)",
        "position  \(position) m",
        "convert   \(convert)",
      ]
    }

    let captureLines = sessionLines + counterLines
    // Withheld where there is no capture to describe. A device with no LiDAR
    // produces an all-zero read-out, and printing it under the line explaining
    // that the hardware cannot scan reads as a scan that is failing rather than
    // one that was never possible.
    let text = Self.transcript(
      device: deviceSummary, render: renderLines, fps: fps,
      session: sessionLines,
      summary: counterLines.isEmpty ? nil : snapshot.summary,
      counters: counterLines)

    // From this object's own state, not from ARKit's error fields. Those say
    // whether the session is healthy; this row claims the capture + fuse + draw
    // loop is running, and `isCapturing` is the field that means that. On a
    // VK_ERROR_DEVICE_LOST the old form left a red recording dot and "Scanning"
    // frozen on screen with nothing capturing, fusing or drawing; a non-fatal
    // didFailWithError flipped it to "Paused" while fps and triangles climbed.
    dashboard.scanning = isCapturing && !renderFailed
    dashboard.elapsed = CFAbsoluteTimeGetCurrent() - sessionStartedAt
    dashboard.fps = fps
    dashboard.trackingText = tracking.description
    // "normal" is ARKit's own word for a usable pose; anything else -- limited,
    // initializing, relocalizing -- means frames are being withheld from
    // fusion, which is exactly when this chip should stop being quiet.
    dashboard.trackingHealthy = tracking.description == "normal"
    dashboard.framesIn = Int(s.frames_submitted)
    dashboard.framesDropped = Int(s.frames_dropped)
    // Both ceilings, not one. The earlier note here picked the working set on
    // the grounds that the jetsam ceiling sits above installed RAM and so
    // reports headroom that does not exist -- true, and an argument for drawing
    // the working set *first*, not for discarding the other. They can bind in
    // either order, and the one that kills the process is the jetsam limit.
    // Neither was ever drawn anyway: these two properties were assigned every
    // tick to a view that rendered no gauge at all.
    dashboard.memoryUsedBytes = snapshot.memoryFootprintBytes
    dashboard.memoryWorkingSetBytes = snapshot.gpuWorkingSetBytes
    dashboard.memoryLimitBytes = snapshot.memoryLimitBytes
    dashboard.memoryPeakBytes = snapshot.memoryPeakBytes
    dashboard.memoryValid = snapshot.memoryValid
    dashboard.stages = snapshot.stages.map {
      StageBar(
        id: $0.name, name: $0.name, hostMs: $0.cpuMs,
        deviceMs: $0.gpuMs, hasGPU: $0.hasGpu)
    }
    // What the bars do not cover, and how far they can be trusted. The age is
    // two figures because it is a comparison: `msSinceStages` alone cannot tell
    // a stalled pipeline from a paused camera.
    dashboard.atlasCopyMs = snapshot.atlasCopyMs
    dashboard.framesFused = snapshot.framesFused
    dashboard.msSinceStages = snapshot.msSinceStages
    dashboard.msSinceFuse = snapshot.msSinceFuse
    dashboard.stagesStale = snapshot.stagesStale
    dashboard.stagesTruncated = snapshot.stagesTruncated
    dashboard.gpuTimingRetired = snapshot.gpuTimingRetired
    dashboard.history = snapshot.history.map {
      FrameSample(
        id: $0.frame, hostMs: $0.hostMs, deviceMs: $0.deviceMs,
        // The meshing cost, which neither total above covers. Dropping it made
        // the chart plot the fuse alone and miss every remesh spike -- the one
        // thing the ring was built to show.
        extractMs: $0.extractMs, deviceTimingValid: $0.deviceTimingValid)
    }
    // From the live snapshot, NOT from `history.last`. The history is appended
    // only by a frame that succeeds all the way through, while occupancy and
    // the stop cause are published before the allocate- and integrate-failure
    // returns: on a run of integrate failures the Volume card advanced and
    // turned critical every frame while the headline meter stayed frozen at the
    // last fused frame and the banner never appeared at all.
    dashboard.triangles = Int(snapshot.triangles)
    dashboard.vertices = Int(snapshot.vertices)
    dashboard.triangleCapacity = Int(snapshot.triangleCapacity)
    // Travels with the capacity it qualifies. Both halves of the arena fill are
    // stamped only by a successful remesh, so without this the gauge freezes
    // and goes on reporting a fill for a surface that has kept growing.
    dashboard.extractStale = snapshot.extractStale
    dashboard.occupancy = snapshot.occupancy
    dashboard.occupancyKnown = snapshot.occupancyKnown
    dashboard.allocationStopReason = snapshot.allocationStopReason
    // The grouping is the bridge's, not this file's, and so is the text above:
    // both come off one snapshot, so the panel and the transcript cannot
    // disagree about a figure or about which frame it belongs to. See the note
    // at the top of Dashboard.swift for what they used to disagree about.
    dashboard.groups = snapshot.sections.map { s in
      StatGroup(
        id: s.title,
        items: s.rows.map {
          StatItem(
            label: $0.label, value: $0.value, tone: $0.tone,
            drawnAsGauge: $0.drawnAsGauge)
        })
    }
    // True prose rather than figures, so these stay as lines -- but as three
    // cards rather than one. The render state used to be appended onto the
    // device identity, which put a drawable size, a camera mode and a presented
    // count under a heading that says "Device": four lines that change every
    // frame beneath four that are fixed for the life of the process. The camera
    // mode in particular is worth finding, because a manual camera pointed away
    // from the scan and a scan that stopped producing geometry look identical
    // on screen, and the only text documenting double-tap-to-refollow is on it.
    dashboard.deviceLines =
      deviceSummary.components(separatedBy: "\n").filter { !$0.isEmpty }
    dashboard.renderLines = renderLines
    dashboard.captureLines = captureLines

    // The text survives for stdout alone: `devicectl process launch --console`
    // is how every device run in this project has been read, and the dashboard
    // above is not a substitute for a transcript.
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

/// A view that claims only the touches its content actually uses.
///
/// The dashboard is a sibling of `metalView`, above it in z-order and pinned
/// across the full width and 55% of the safe area. UIKit hit-testing stops at
/// the topmost view whose `point(inside:)` says yes, so with the default
/// implementation every touch in that region belonged to the panel and the
/// camera recognizers -- orbit, pan, zoom, double-tap-to-refollow -- were dead
/// across the top half of the screen from first launch.
///
/// Asking the subtree instead of assuming: a point is inside only where the
/// hosted SwiftUI hierarchy hit-tests something real. Cards, chips, the chevron
/// and the scroll area answer yes and behave normally; the panel's margins and
/// its empty regions answer no and the touch reaches the reconstruction. This
/// only has anything to fall through from because `DashboardView` marks its
/// material background `allowsHitTesting(false)` -- a full-bleed background
/// that hit-tests is indistinguishable from the default behaviour this
/// replaces.
private final class PassthroughContainer: UIView {
  override func point(inside point: CGPoint, with event: UIEvent?) -> Bool {
    for sub in subviews {
      if sub.hitTest(convert(point, to: sub), with: event) != nil {
        return true
      }
    }
    return false
  }
}
