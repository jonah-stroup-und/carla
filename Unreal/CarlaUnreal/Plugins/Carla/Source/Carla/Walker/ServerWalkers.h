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

/// Settings for the server-side pedestrians, read from the command line:
///
///   -server-walkers=N         number of pedestrians (0 or absent: off)
///   -server-walkers-seed=S    seed for blueprints, spawn points and routes
///   -server-walkers-cross=P   probability (0 to 1) that a pedestrian's route
///                             crosses roads off the crosswalks, default 0
///   -server-walkers-run=P     probability (0 to 1) that a pedestrian runs,
///                             default 0
struct FServerWalkersSettings
{
  int32 NumberOfWalkers = 0;
  int32 Seed = 0;
  float CrossFactor = 0.0f;
  float RunFactor = 0.0f;

  bool IsEnabled() const { return NumberOfWalkers > 0; }

  static FServerWalkersSettings FromCommandLine();
};

/// Pedestrians the server spawns and walks itself, on the map's Recast
/// navmesh (Nav/<map>.bin), with a copy of the client's crowd
/// (LibCarla/source/carla/servercrowd) — the server-side counterpart of
/// generate_traffic.py's walkers, with no client and no walker AI controllers.
///
/// FCarlaEngine owns one per episode and calls Tick() on the game thread at
/// the start of every world tick, before actors and physics. Walkers wait at
/// crosswalks for the traffic lights and for vehicles, and avoid vehicles.
/// A walker a vehicle kills is left where it fell for a while, then removed
/// and replaced.
///
/// Maps with no .bin navmesh (World Partition maps navigate on the engine's
/// navmesh) are not supported yet: the walkers stay off with an error.
class FServerWalkers
{
public:

  FServerWalkers(UCarlaEpisode &Episode, const FServerWalkersSettings &Settings);
  ~FServerWalkers();

  FServerWalkers(const FServerWalkers &) = delete;
  FServerWalkers &operator=(const FServerWalkers &) = delete;

  /// Loads the navmesh and spawns the walkers on the first call, then steps
  /// the crowd once.
  void Tick(float DeltaSeconds);

private:

  class FImpl;
  std::unique_ptr<FImpl> Impl;
};
