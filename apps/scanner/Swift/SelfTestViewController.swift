// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

import UIKit

/// The screen of a launch self-test (`-self-test <name>`), shown instead of
/// the scanner's, so the camera is never opened.
///
/// Each report line goes to stdout as it is made, which
/// `devicectl device process launch --console` relays, and onto the screen.
/// The app exits when the test ends: 0 when it passed, 1 when it failed, 2 for
/// a name no test has.
final class SelfTestViewController: UIViewController {
  private let kind: ScannerSelfTestKind
  private let report = UITextView()
  private var started = false

  init(kind: ScannerSelfTestKind) {
    self.kind = kind
    super.init(nibName: nil, bundle: nil)
  }

  @available(*, unavailable)
  required init?(coder: NSCoder) {
    fatalError("init(coder:) is not used")
  }

  override func viewDidLoad() {
    super.viewDidLoad()
    view.backgroundColor = .systemBackground
    report.isEditable = false
    report.font = .monospacedSystemFont(ofSize: 11, weight: .regular)
    report.translatesAutoresizingMaskIntoConstraints = false
    view.addSubview(report)
    let safe = view.safeAreaLayoutGuide
    NSLayoutConstraint.activate([
      report.topAnchor.constraint(equalTo: safe.topAnchor),
      report.bottomAnchor.constraint(equalTo: safe.bottomAnchor),
      report.leadingAnchor.constraint(equalTo: safe.leadingAnchor, constant: 8),
      report.trailingAnchor.constraint(equalTo: safe.trailingAnchor, constant: -8),
    ])
  }

  override func viewDidAppear(_ animated: Bool) {
    super.viewDidAppear(animated)
    guard !started else { return }
    started = true
    // A lock would background the app and take its encoder sessions.
    UIApplication.shared.isIdleTimerDisabled = true
    // The window's root, so it outlives the run it holds.
    DispatchQueue.global(qos: .userInitiated).async {
      let status: Int32
      switch self.kind {
      case .hevc:
        let passed = ScannerSelfTest.runHevc { line in self.emit(line) }
        status = passed ? 0 : 1
      default:
        self.emit("self-test: the launch names no test this app has (it has: hevc)")
        status = 2
      }
      // After every line queued for the screen.
      DispatchQueue.main.async { exit(status) }
    }
  }

  private func emit(_ line: String) {
    print(line)
    fflush(stdout)
    DispatchQueue.main.async {
      self.report.text = (self.report.text ?? "") + line + "\n"
    }
  }
}
