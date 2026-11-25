// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <util/ue-header-guard-begin.h>
#include "CoreMinimal.h"
#include "UObject/StrongObjectPtr.h"
#include <util/ue-header-guard-end.h>

class ACarlaWheeledVehicle;
class UAudioComponent;
class USoundWaveProcedural;

/// The server player's car's sound, synthesised as it plays (CARLA ships no
/// vehicle sounds):
///
///   - the engine: a smooth hum of the harmonics of its firing at the
///     engine's RPM, muffled at low revs, opening up with revs and throttle;
///   - road noise, rising with speed;
///   - tyre squeal when the tyres slide;
///   - a dull, muffled pound on a collision, as heard from inside the car,
///     its size from the impact, and a scrape while the car slides along
///     something.
///
/// Levels are in Config/ServerSound.json (or -server-sound=FILE), reloaded
/// when it changes:
///
///   { "volume": 0.8, "engine": 1, "road": 1, "tyres": 1, "collisions": 1,
///     "debug": false }
///
/// "debug" logs the engine and tyre readings twice a second, for tuning.
class FServerCarSound
{
public:

  explicit FServerCarSound(ACarlaWheeledVehicle &Vehicle);
  ~FServerCarSound();

  FServerCarSound(const FServerCarSound &) = delete;
  FServerCarSound &operator=(const FServerCarSound &) = delete;

  /// Call once a frame. bInterior: the camera is in the cabin, which
  /// muffles the outside.
  void Tick(float DeltaSeconds, bool bInterior);

  /// A collision this frame: its impulse, N s (FServerDriveReport::TakeImpact).
  void AddImpact(float Impulse);

  class FSynth;

private:

  TWeakObjectPtr<ACarlaWheeledVehicle> Vehicle;
  TSharedPtr<FSynth, ESPMode::ThreadSafe> Synth;
  TStrongObjectPtr<USoundWaveProcedural> Wave;
  TWeakObjectPtr<UAudioComponent> Component;

  FString ConfigPath;
  FDateTime ConfigStamp;
  double LastReloadCheck = 0.0;
  double LastDebugLog = 0.0;
  bool bDebug = false;

  float FrameImpulse = 0.0f;
  double LastContact = -1.0e9;
  float ContactLevel = 0.0f;

  void Reload();
};
