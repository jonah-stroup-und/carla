// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/servercrowd/ServerCrowd.h"

#include "carla/servercrowd/Navigation.h"

#include <algorithm>

namespace carla {
namespace servercrowd {

  ServerCrowd::ServerCrowd(CrowdWorld &world, const unsigned int seed)
    : _nav(std::make_unique<Navigation>()) {
    _nav->SetSeed(seed);
    _nav->SetWorld(&world);
  }

  ServerCrowd::~ServerCrowd() = default;

  bool ServerCrowd::Load(std::vector<uint8_t> navmesh) {
    return _nav->Load(std::move(navmesh));
  }

  void ServerCrowd::SetPedestriansCrossFactor(const float probability) {
    _nav->SetPedestriansCrossFactor(probability);
  }

  bool ServerCrowd::GetRandomLocation(geom::Location &location) const {
    return _nav->GetRandomLocation(location);
  }

  bool ServerCrowd::AddWalker(
      const rpc::ActorId id,
      const geom::Location &from,
      const geom::Location &target,
      const float max_speed) {
    if (!_nav->AddWalker(id, from)) {
      return false;
    }
    _walkers.push_back(id);
    _nav->SetWalkerTarget(id, target);
    _nav->SetWalkerMaxSpeed(id, max_speed);
    return true;
  }

  void ServerCrowd::RemoveWalker(const rpc::ActorId id) {
    _nav->RemoveAgent(id);
    _walkers.erase(std::remove(_walkers.begin(), _walkers.end(), id), _walkers.end());
  }

  void ServerCrowd::Update(
      const double elapsed_seconds,
      const double delta_seconds,
      const std::vector<Vehicle> &vehicles,
      std::vector<WalkerState> &states,
      std::vector<rpc::ActorId> &killed) {
    states.clear();
    killed.clear();
    if (_walkers.empty()) {
      return;
    }

    // As client::detail::WalkerNavigation::Tick: vehicles in, crowd step,
    // walker transforms out, then the walkers a vehicle killed.
    std::vector<VehicleCollisionInfo> obstacles;
    obstacles.reserve(vehicles.size());
    for (const Vehicle &vehicle : vehicles) {
      obstacles.push_back(VehicleCollisionInfo{vehicle.id, vehicle.transform, vehicle.bounding_box});
    }
    _nav->UpdateVehicles(obstacles);

    _nav->UpdateCrowd(elapsed_seconds, delta_seconds);

    states.reserve(_walkers.size());
    geom::Transform transform;
    for (const rpc::ActorId id : _walkers) {
      if (_nav->GetWalkerTransform(id, transform)) {
        states.push_back(WalkerState{id, transform, _nav->GetWalkerSpeed(id)});
      }
    }

    bool alive = true;
    for (const rpc::ActorId id : _walkers) {
      if (_nav->IsWalkerAlive(id, alive) && !alive) {
        killed.push_back(id);
      }
    }
    for (const rpc::ActorId id : killed) {
      RemoveWalker(id);
    }
  }

} // namespace servercrowd
} // namespace carla
