// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <util/ue-header-guard-begin.h>
#include "CoreMinimal.h"
#include <util/ue-header-guard-end.h>

/// What a steering-wheel button can do in the server player's car.
enum class EServerWheelAction : uint8
{
  SignalLeft,
  SignalRight,
  Gear,
  Camera,
  Reset,
  HandBrake,
  BlindSpot,
  Pillars,
  Mission,
  Restart,
  Count
};

/// A steering wheel and pedals (Fanatec, Logitech, Thrustmaster, ...) for the
/// server player's car.
///
/// Unreal does not read DirectInput game controllers, so this polls the
/// Windows joystick API (winmm): the axes once a frame, the buttons every
/// 2 ms on a thread of their own, so a quick tap is never missed. The
/// device is found by USB
/// vendor id, rescanned off the game thread while none is connected, so a
/// wheel can be plugged in while the game runs.
///
/// The mapping comes from Config/ServerWheel.json (or -server-wheel=FILE),
/// reloaded when it changes:
///
///   "vendor":   USB vendor id, hex (default "0EB7", Fanatec); "" for any
///               joystick
///   "steer":    { "axis": "x", "invert": false, "deadzone": 0, "gamma": 1, "gain": 1 }
///   "throttle": { "axis": "z", "invert": true, "deadzone": 0.02, "gamma": 1 }
///   "brake":    { "axis": "r", "invert": true, "deadzone": 0.02, "gamma": 1 }
///   "buttons":  { "<action>": [12, "pov_left", ...], ... }
///
/// Axes are x, y, z, r, u, v ("" for none). Pedals read 0 at rest after
/// "invert"; the deadzone is cut from the rest end, and "gamma" > 1 makes
/// the first part of the travel gentler (for steering: the centre). Steering "gain" scales the input
/// (the lock-to-lock rotation is set on the wheel base). Buttons are
/// numbered from 1 as in Windows' game controller dialog; the d-pad is
/// pov_up, pov_right, pov_down and pov_left. Actions: signal_left,
/// signal_right, gear, camera, reset, handbrake, blind_spot, pillars,
/// mission, restart.
///
/// The last button pressed is shown on the HUD, to find a button's number.
class FServerWheel
{
public:

  FServerWheel();
  ~FServerWheel();

  /// Reads the device; call once a frame before the getters.
  void Poll();

  bool IsConnected() const;

  /// -1 full left to 1 full right.
  float GetSteer() const;

  /// 0 released to 1 fully pressed.
  float GetThrottle() const;
  float GetBrake() const;

  bool IsDown(EServerWheelAction Action) const;

  /// True on the frame the action's button went down.
  bool WasPressed(EServerWheelAction Action) const;

  /// e.g. "button 12", the last control pressed, for a few seconds after
  /// it was pressed; empty otherwise.
  FString GetLastPressed() const;

  /// e.g. "Fanatec 0EB7:0007", for the HUD.
  FString GetDescription() const;

  static FString GetConfigPath();

private:

  class FImpl;
  TUniquePtr<FImpl> Impl;
};
