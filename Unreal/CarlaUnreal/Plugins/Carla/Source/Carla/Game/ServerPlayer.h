// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <util/ue-header-guard-begin.h>
#include "CoreMinimal.h"
#include <util/ue-header-guard-end.h>

#include "Carla/Game/ServerWarning.h"

#include <memory>

class UCarlaEpisode;

/// Settings for the server-side player vehicle, read from the command line:
///
///   -server-player                 spawn a vehicle driven from this window
///   -server-player-vehicle=P       blueprint id or wildcard, default
///                                  "vehicle.ue4.ford.crown", the car the
///                                  mirrors are set up for (cars only; a
///                                  random match is picked)
///   -server-player-seed=S          seed for the blueprint and spawn point
///   -server-player-camera=MODE     starting camera: chase (the default),
///                                  far, or interior
///   -server-player-pitch=DEG       interior view pitch, degrees up (default 2,
///                                  so the rear-view mirror is in frame);
///                                  Page Up / Page Down adjust it while driving
///   -server-dlss[=MODE]            upscale the window with DLSS Super Resolution:
///                                  dlaa, quality (the default), balanced,
///                                  performance, or off (Unreal's TSR)
///   -server-blind-spot=MODE        the blind-spot monitor: off, visual (the
///                                  default: an indicator in each side mirror),
///                                  or audio (the indicator, and a chime on
///                                  signalling towards an occupied side);
///                                  B cycles them while driving
///   -server-pillars=MODE           the A-pillar monitor: a light on each
///                                  A-pillar while it hides a road user; off,
///                                  visual (the default) or audio (the light,
///                                  and a low chime as one disappears behind
///                                  it); P cycles them while driving
///   -server-blind-spot-test        light the mirror and A-pillar indicators,
///                                  to line them up
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
  FString Filter = TEXT("vehicle.ue4.ford.crown");
  int32 Seed = 0;
  FString Camera;
  float InteriorPitch = 2.0f;
  /// Empty: leave the upscaler alone (TSR, unless the console changed it).
  FString Dlss;
  bool bSurround = false;
  float SurroundFov = 60.0f;
  EServerWarningMode BlindSpot = EServerWarningMode::Visual;
  EServerWarningMode Pillars = EServerWarningMode::Visual;
  bool bWarningTest = false;

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
/// B cycles the blind-spot monitor (off, visual, visual + audio), P the
/// A-pillar monitor,
/// F9 cycles the upscaler (TSR, DLAA, DLSS Quality, Balanced, Performance).
/// Gamepad: right/left trigger throttle and brake, X drive/reverse, left
/// stick steer, d-pad left/right turn signal, A handbrake, Y camera, Back reset.
///
/// Each drive (a vehicle, from spawn to the end of the episode) is recorded
/// by an FServerDriveReport: collisions and traffic violations, written as
/// two CSV files to Reports on the desktop (or -server-reports=DIR), and its
/// mirrors work (FServerMirrors) where Config/ServerPlayerMirrors.json
/// says where the vehicle's mirror glass is. Its blind-spot monitor
/// (FServerBlindSpot) lights an indicator in the side mirror, or on the HUD
/// outside the interior camera; its A-pillar monitor (FServerPillars) lights
/// a light on the pillar hiding a road user.
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
