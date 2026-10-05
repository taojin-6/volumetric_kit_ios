// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

import UIKit

/// Owns the app's one window, made when UIKit connects the scene the
/// Info.plist's `UIApplicationSceneManifest` declares.
///
/// `@objc(SceneDelegate)` gives the class an unqualified runtime name, which
/// is what the manifest's `UISceneDelegateClassName` names, so it does not
/// depend on the Swift module the target builds.
@objc(SceneDelegate)
final class SceneDelegate: UIResponder, UIWindowSceneDelegate {
  var window: UIWindow?

  func scene(
    _ scene: UIScene, willConnectTo session: UISceneSession,
    options connectionOptions: UIScene.ConnectionOptions
  ) {
    guard let windowScene = scene as? UIWindowScene else { return }
    let window = UIWindow(windowScene: windowScene)
    window.rootViewController = ScannerViewController()
    window.makeKeyAndVisible()
    self.window = window
  }
}
