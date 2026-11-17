// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/geom/BoundingBox.h"
#include "carla/geom/Location.h"
#include "carla/geom/Transform.h"
#include "carla/rpc/ActorId.h"
#include "carla/servercrowd/CrowdWorld.h"

#include <cstdint>
#include <memory>
#include <vector>

namespace carla {
namespace servercrowd {

  class Navigation;

  /// The pedestrian crowd, run inside the server.
  ///
  /// It does what client::detail::WalkerNavigation does for a client (the
  /// Recast/Detour crowd on the map's .bin navmesh, walkers routed between
  /// random points, waiting at crosswalks for traffic lights and vehicles), but
  /// the owner calls Update() once per simulation frame and applies the
  /// returned walker states itself: no RPC, no client tick.
  ///
  /// This header hides Detour (see Detour.h), so it is safe to include next to
  /// Unreal's navigation headers.
  class ServerCrowd {
  public:

    struct Vehicle {
      rpc::ActorId id;
      geom::Transform transform;
      geom::BoundingBox bounding_box;
    };

    struct WalkerState {
      rpc::ActorId id;
      geom::Transform transform;
      float speed;
    };

    /// @a world must outlive this object.
    ServerCrowd(CrowdWorld &world, unsigned int seed);
    ~ServerCrowd();

    ServerCrowd(const ServerCrowd &) = delete;
    ServerCrowd &operator=(const ServerCrowd &) = delete;

    /// Loads the navmesh (the map's Nav/<map>.bin) and creates the crowd.
    bool Load(std::vector<uint8_t> navmesh);

    /// Probability (0 to 1) that a walker's route crosses roads off crosswalks.
    void SetPedestriansCrossFactor(float probability);

    /// A random walkable location (meters), for spawning and targets.
    bool GetRandomLocation(geom::Location &location) const;

    /// Adds a walker standing at @a from and sends it towards @a target at
    /// @a max_speed (m/s), as WalkerAIController start/go_to_location/
    /// set_max_speed do.
    bool AddWalker(rpc::ActorId id, const geom::Location &from, const geom::Location &target, float max_speed);

    void RemoveWalker(rpc::ActorId id);

    /// Steps the crowd to simulation time @a elapsed_seconds. @a vehicles are
    /// the obstacles walkers avoid. Fills @a states with the new transform of
    /// every walker and @a killed with walkers a vehicle hit (already removed
    /// from the crowd).
    void Update(
        double elapsed_seconds,
        double delta_seconds,
        const std::vector<Vehicle> &vehicles,
        std::vector<WalkerState> &states,
        std::vector<rpc::ActorId> &killed);

    size_t GetNumberOfWalkers() const { return _walkers.size(); }

  private:

    std::unique_ptr<Navigation> _nav;
    std::vector<rpc::ActorId> _walkers;
  };

} // namespace servercrowd
} // namespace carla
