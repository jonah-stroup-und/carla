// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#include "carla/servertraffic/ALSM.h"
#include "carla/servertraffic/AtomicActorSet.h"
#include "carla/servertraffic/CollisionStage.h"
#include "carla/servertraffic/DataStructures.h"
#include "carla/servertraffic/InMemoryMap.h"
#include "carla/servertraffic/LocalizationStage.h"
#include "carla/servertraffic/MotionPlanStage.h"
#include "carla/servertraffic/Parameters.h"
#include "carla/servertraffic/RandomGenerator.h"
#include "carla/servertraffic/SimulationState.h"
#include "carla/servertraffic/TrackTraffic.h"
#include "carla/servertraffic/TrafficLightStage.h"
#include "carla/servertraffic/VehicleLightStage.h"
#include "carla/servertraffic/World.h"

namespace carla {
namespace servertraffic {

  using LocalMapPtr = std::shared_ptr<InMemoryMap>;

  /// The Traffic Manager, run inside the server.
  ///
  /// A copy of carla::traffic_manager::TrafficManagerLocal with the transport
  /// taken out: no RPC server, no worker thread, no episode proxy. The owner
  /// calls Step() once per simulation frame on the game thread, before
  /// physics, so the stages always plan on the frame they are about to drive
  /// — in synchronous and asynchronous mode alike, with no client attached.
  ///
  /// Vehicles registered here are driven only by this instance; the client
  /// Traffic Manager is unaffected and can run alongside it on other vehicles.
  class ServerTrafficManager {
  public:

    /// @a world must outlive this object.
    ServerTrafficManager(world::World &world, uint64_t seed);
    ~ServerTrafficManager();

    ServerTrafficManager(const ServerTrafficManager &) = delete;
    ServerTrafficManager &operator=(const ServerTrafficManager &) = delete;

    /// Builds the road graph from the OpenDRIVE map, or from a cache file
    /// (the same .bin the client Traffic Manager reads) when one is given and
    /// loads. Must be called before the first Step().
    void SetupLocalMap(const std::vector<uint8_t> &cache_content);

    /// One planning cycle: reads the world, runs the stages, applies controls.
    void Step();

    void RegisterVehicles(const std::vector<ActorPtr> &vehicles);
    void UnregisterVehicles(const std::vector<ActorPtr> &vehicles);
    std::vector<ActorId> GetRegisteredVehiclesIDs();

    /// Per-vehicle and global behaviour, as on the client Traffic Manager.
    Parameters &GetParameters() { return parameters; }

  private:

    world::World &world;

    std::vector<float> longitudinal_PID_parameters;
    std::vector<float> longitudinal_highway_PID_parameters;
    std::vector<float> lateral_PID_parameters;
    std::vector<float> lateral_highway_PID_parameters;

    AtomicActorSet registered_vehicles;
    std::unordered_map<ActorId, std::pair<float, bool>> large_vehicles;
    int registered_vehicles_state = -1;
    std::vector<ActorId> vehicle_id_list;
    LocalMapPtr local_map;
    BufferMap buffer_map;
    TrackTraffic track_traffic;
    SimulationState simulation_state;
    Parameters parameters;

    LocalizationFrame localization_frame;
    CollisionFrame collision_frame;
    TLFrame tl_frame;
    ControlFrame control_frame;
    uint64_t current_reserved_capacity = 0u;

    RandomGenerator random_device;
    std::vector<ActorId> marked_for_removal;

    LocalizationStage localization_stage;
    CollisionStage collision_stage;
    TrafficLightStage traffic_light_stage;
    MotionPlanStage motion_plan_stage;
    VehicleLightStage vehicle_light_stage;
    ALSM alsm;
  };

} // namespace servertraffic
} // namespace carla
