// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

import UIKit

/// The process's delegate. The window is the scene's (`SceneDelegate`): an app
/// built against the iOS 27 SDK must adopt the scene lifecycle, and UIKit
/// traps at launch one that still sets its window up here.
@main
final class AppDelegate: UIResponder, UIApplicationDelegate {
  func application(
    _ application: UIApplication,
    didFinishLaunchingWithOptions options: [UIApplication.LaunchOptionsKey: Any]?
  ) -> Bool {
    return true
  }
}
