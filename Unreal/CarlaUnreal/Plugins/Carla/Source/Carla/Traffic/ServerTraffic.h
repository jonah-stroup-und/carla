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

/// Settings for the server-side traffic, read from the command line:
///
///   -server-traffic=N           number of autopilot vehicles (0 or absent: off)
///   -server-traffic-seed=S      seed for blueprints, spawn points and behaviour
///   -server-traffic-filter=P    blueprint wildcard, default "vehicle.*"
///   -server-traffic-safe        cars only (base_type=car), as generate_traffic --safe
struct FServerTrafficSettings
{
  int32 NumberOfVehicles = 0;
  int32 Seed = 0;
  FString Filter = TEXT("vehicle.*");
  bool bSafe = false;

  bool IsEnabled() const { return NumberOfVehicles > 0; }

  static FServerTrafficSettings FromCommandLine();
};

/// Runs a copy of the Traffic Manager inside the server
/// (LibCarla/source/carla/servertraffic) on vehicles the server spawns itself.
///
/// FCarlaEngine owns one per episode and calls Tick() on the game thread at
/// the start of every world tick, before actors and physics, so each vehicle
/// is planned on the frame it is driven in. No client, no RPC, no Traffic
/// Manager port: the client Traffic Manager is unaffected and never sees
/// these vehicles as its own. A client's apply_control on one of them still
/// wins, as client input outranks autopilot input.
class FServerTraffic
{
public:

  FServerTraffic(UCarlaEpisode &Episode, const FServerTrafficSettings &Settings);
  ~FServerTraffic();

  FServerTraffic(const FServerTraffic &) = delete;
  FServerTraffic &operator=(const FServerTraffic &) = delete;

  /// Builds the road graph and spawns the vehicles on the first call, then
  /// steps the traffic once.
  void Tick(float DeltaSeconds);

private:

  class FImpl;
  std::unique_ptr<FImpl> Impl;
};
