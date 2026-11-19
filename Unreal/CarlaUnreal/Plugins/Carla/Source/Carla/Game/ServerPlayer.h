// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <util/ue-header-guard-begin.h>
#include "CoreMinimal.h"
#include <util/ue-header-guard-end.h>

#include <memory>

class UCarlaEpisode;

/// Settings for the server-side player vehicle, read from the command line:
///
///   -server-player                 spawn a vehicle driven from this window
///   -server-player-vehicle=P       blueprint id or wildcard, default "vehicle.*"
///                                  (cars only; a random match is picked)
///   -server-player-seed=S          seed for the blueprint and spawn point
///   -server-dlss[=MODE]            upscale the window with DLSS Super Resolution:
///                                  dlaa, quality (the default), balanced,
///                                  performance, or off (Unreal's TSR)
///   -server-reports=DIR            where the drive reports go, default Reports
///                                  on the desktop
///   -server-surround[=FOV]         spread the window over three side-by-side
///                                  monitors, each with its own camera turned
///                                  to face that way; FOV is the centre
///                                  screen's horizontal field of view in
///                                  degrees (default 60), the side screens
///                                  continue it
struct FServerPlayerSettings
{
  bool bEnabled = false;
  FString Filter = TEXT("vehicle.*");
  int32 Seed = 0;
  /// Empty: leave the upscaler alone (TSR, unless the console changed it).
  FString Dlss;
  bool bSurround = false;
  float SurroundFov = 60.0f;

  bool IsEnabled() const { return bEnabled; }

  static FServerPlayerSettings FromCommandLine();

  /// -server-surround needs a borderless window, and the window is created
  /// before any episode: the Carla module calls this at startup.
  static void ConfigureWindowFromCommandLine();
};

/// A vehicle driven from the simulator window, so the server runs as a
/// single-player game with no client attached.
///
/// FCarlaEngine owns one per episode and calls Tick() on the game thread at
/// the start of every world tick, before the server traffic, so the player
/// takes its spawn point first. The vehicle is a regular CARLA actor
/// (role_name "hero"): clients and the server traffic see it like any other.
/// Input goes in at User priority, so a client's apply_control still wins.
///
/// The spectator stays possessed, with its own input disabled, and follows
/// the player's camera, so a client's world.get_spectator() keeps working.
///
/// Keyboard: W/Up throttle, S/Down brake, Q drive/reverse (when nearly
/// stopped), A/D or Left/Right steer, Z/X left/right turn signal, Space
/// handbrake, C camera (chase, far chase, interior), Backspace reset upright,
/// F9 cycles the upscaler (TSR, DLAA, DLSS Quality, Balanced, Performance).
/// Gamepad: right/left trigger throttle and brake, X drive/reverse, left
/// stick steer, d-pad left/right turn signal, A handbrake, Y camera, Back reset.
///
/// Each drive (a vehicle, from spawn to the end of the episode) is recorded
/// by an FServerDriveReport: collisions and traffic violations, written as
/// two CSV files to Reports on the desktop (or -server-reports=DIR).
class FServerPlayer
{
public:

  FServerPlayer(UCarlaEpisode &Episode, const FServerPlayerSettings &Settings);
  ~FServerPlayer();

  FServerPlayer(const FServerPlayer &) = delete;
  FServerPlayer &operator=(const FServerPlayer &) = delete;

  /// Spawns the vehicle on the first call, then reads input and drives it.
  void Tick(float DeltaSeconds);

private:

  class FImpl;
  std::unique_ptr<FImpl> Impl;
};
