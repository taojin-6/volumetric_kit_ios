// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The app shell for the iOS compute smoke: a scrollable monospace read-out of
// smoke_report.cpp's findings. Deliberately the thinnest possible UIKit host --
// all the substance is in the C++ that runs against MoltenVK, and this file
// exists only to put it on a screen.
//
// This is an .mm (Objective-C++) translation unit: the ARKit capture source
// that lands next uses the same seam, since ARKit is reachable only from
// Objective-C / Swift while recon is C++.

#import <UIKit/UIKit.h>

#include <string>

#include "smoke_report.hpp"

@interface SmokeViewController : UIViewController
@end

@implementation SmokeViewController {
  UILabel* _label;
  UIActivityIndicatorView* _spinner;
}

- (void)viewDidLoad {
  [super viewDidLoad];
  self.view.backgroundColor = UIColor.systemBackgroundColor;

  UIScrollView* scroll = [[UIScrollView alloc] initWithFrame:CGRectZero];
  scroll.translatesAutoresizingMaskIntoConstraints = NO;
  [self.view addSubview:scroll];

  _label = [[UILabel alloc] initWithFrame:CGRectZero];
  _label.translatesAutoresizingMaskIntoConstraints = NO;
  _label.numberOfLines = 0;
  _label.font = [UIFont monospacedSystemFontOfSize:11
                                            weight:UIFontWeightRegular];
  _label.text = @"Running smoke...";
  [scroll addSubview:_label];

  _spinner = [[UIActivityIndicatorView alloc]
      initWithActivityIndicatorStyle:UIActivityIndicatorViewStyleMedium];
  _spinner.translatesAutoresizingMaskIntoConstraints = NO;
  [self.view addSubview:_spinner];
  [_spinner startAnimating];

  UILayoutGuide* safe = self.view.safeAreaLayoutGuide;
  UILayoutGuide* content = scroll.contentLayoutGuide;
  [NSLayoutConstraint activateConstraints:@[
    [scroll.topAnchor constraintEqualToAnchor:safe.topAnchor],
    [scroll.bottomAnchor constraintEqualToAnchor:safe.bottomAnchor],
    [scroll.leadingAnchor constraintEqualToAnchor:safe.leadingAnchor],
    [scroll.trailingAnchor constraintEqualToAnchor:safe.trailingAnchor],

    [_label.topAnchor constraintEqualToAnchor:content.topAnchor constant:12],
    [_label.bottomAnchor constraintEqualToAnchor:content.bottomAnchor
                                        constant:-12],
    [_label.leadingAnchor constraintEqualToAnchor:content.leadingAnchor
                                         constant:12],
    [_label.widthAnchor constraintEqualToAnchor:scroll.widthAnchor constant:-24],

    [_spinner.centerXAnchor constraintEqualToAnchor:self.view.centerXAnchor],
    [_spinner.centerYAnchor constraintEqualToAnchor:self.view.centerYAnchor],
  ]];

  // Off the main thread: device + pipeline creation and three dispatches are
  // fast but not instant, and blocking the main thread through launch invites
  // the watchdog.
  dispatch_async(
      dispatch_get_global_queue(QOS_CLASS_USER_INITIATED, 0), ^{
        const std::string report =
            volumetric_kit::ios_app::run_smoke_report();
        NSString* text = [NSString stringWithUTF8String:report.c_str()];
        // Also to the device console, so `devicectl` / Console.app captures the
        // report without needing the screen.
        NSLog(@"%@", text);
        dispatch_async(dispatch_get_main_queue(), ^{
          [self->_spinner stopAnimating];
          self->_label.text = text;
        });
      });
}

@end

@interface AppDelegate : UIResponder <UIApplicationDelegate>
@property(nonatomic, strong) UIWindow* window;
@end

@implementation AppDelegate

- (BOOL)application:(UIApplication*)application
    didFinishLaunchingWithOptions:(NSDictionary*)options {
  self.window = [[UIWindow alloc] initWithFrame:UIScreen.mainScreen.bounds];
  self.window.rootViewController = [[SmokeViewController alloc] init];
  [self.window makeKeyAndVisible];
  return YES;
}

@end

int main(int argc, char* argv[]) {
  @autoreleasepool {
    return UIApplicationMain(argc, argv, nil,
                             NSStringFromClass([AppDelegate class]));
  }
}
