// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <util/ue-header-guard-begin.h>
#include "CoreMinimal.h"
#include <util/ue-header-guard-end.h>

/// Force feedback for the server player's steering wheel, through
/// DirectInput (the only API wheel bases take forces through).
///
/// The wheel base runs the forces itself, at its own rate, from a few
/// effects this keeps updated:
///
///   - a centring spring that firms up with speed, as the tyres' self-
///     aligning torque does;
///   - damping and, near standstill, friction, for the weight of the
///     steering (and so a direct-drive base does not oscillate);
///   - a jolt towards the side of an impact and a rumble that dies away,
///     on a collision.
///
/// Tuned in the "force_feedback" object of Config/ServerWheel.json (or
/// -server-wheel=FILE), reloaded when it changes:
///
///   "force_feedback": { "enabled": true, "strength": 0.5, "centering": 1,
///                       "damping": 1, "collisions": 1 }
///
/// "strength" scales everything (0 to 1, of what the base is set to give in
/// its own menu or Fanatec's app); the others scale their effect. The device
/// is the wheel config's "vendor" (Fanatec by default), found again if it is
/// unplugged and plugged back in.
class FServerForceFeedback
{
public:

  FServerForceFeedback();
  ~FServerForceFeedback();

  /// Call once a frame. Speed is along the car, m/s; bDriving is false
  /// while the car is held (the mission planner), which only damps the
  /// wheel.
  void Tick(float DeltaSeconds, float SpeedMetersPerSecond, bool bDriving);

  /// A collision: its impulse (N s) and where it hit, -1 the car's left to
  /// 1 its right.
  void AddImpact(float Impulse, float Side);

  bool IsActive() const;

private:

  class FImpl;
  TUniquePtr<FImpl> Impl;
};
