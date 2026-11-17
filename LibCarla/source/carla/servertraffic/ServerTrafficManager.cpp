// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/servertraffic/ServerTrafficManager.h"

#include "carla/Logging.h"
#include "carla/servertraffic/Constants.h"

#include <algorithm>

namespace carla {
namespace servertraffic {

  using namespace constants::FrameMemory;
  using namespace constants::PID;
  using constants::SpeedThreshold::INITIAL_PERCENTAGE_SPEED_DIFFERENCE;

  ServerTrafficManager::ServerTrafficManager(world::World &in_world, uint64_t seed)
    : world(in_world),
      longitudinal_PID_parameters(LONGITUDIAL_PARAM),
      longitudinal_highway_PID_parameters(LONGITUDIAL_HIGHWAY_PARAM),
      lateral_PID_parameters(LATERAL_PARAM),
      lateral_highway_PID_parameters(LATERAL_HIGHWAY_PARAM),
      random_device(seed),
      localization_stage(vehicle_id_list, buffer_map, simulation_state, track_traffic,
                         local_map, large_vehicles, parameters, marked_for_removal,
                         localization_frame, random_device),
      collision_stage(vehicle_id_list, simulation_state, buffer_map, track_traffic,
                      parameters, collision_frame, random_device),
      traffic_light_stage(vehicle_id_list, simulation_state, buffer_map, parameters,
                          world, tl_frame, random_device),
      motion_plan_stage(vehicle_id_list, simulation_state, parameters, buffer_map,
                        track_traffic, longitudinal_PID_parameters,
                        longitudinal_highway_PID_parameters, lateral_PID_parameters,
                        lateral_highway_PID_parameters, localization_frame,
                        collision_frame, tl_frame, control_frame, random_device,
                        local_map, large_vehicles),
      vehicle_light_stage(vehicle_id_list, buffer_map, parameters, world, control_frame),
      alsm(registered_vehicles, buffer_map, track_traffic, marked_for_removal,
           parameters, world, local_map, simulation_state, large_vehicles,
           localization_stage, collision_stage, traffic_light_stage,
           motion_plan_stage, vehicle_light_stage) {
    parameters.SetGlobalPercentageSpeedDifference(INITIAL_PERCENTAGE_SPEED_DIFFERENCE);
    // Step() runs once per simulation frame, in lock-step with the world:
    // that is what the stages call synchronous mode, whatever the server's
    // own setting is.
    parameters.SetSynchronousMode(true);

    localization_frame.reserve(INITIAL_SIZE);
    collision_frame.reserve(INITIAL_SIZE);
    tl_frame.reserve(INITIAL_SIZE);
    control_frame.reserve(INITIAL_SIZE);
    current_reserved_capacity = INITIAL_SIZE;
  }

  ServerTrafficManager::~ServerTrafficManager() {
    registered_vehicles.Clear();
    registered_vehicles.TakeReleased();
  }

  void ServerTrafficManager::SetupLocalMap(const std::vector<uint8_t> &cache_content) {
    const SharedPtr<const world::Map> world_map = world.GetMap();
    local_map = std::make_shared<InMemoryMap>(world_map);
    if (!cache_content.empty() && local_map->Load(cache_content)) {
      return;
    }
    if (!cache_content.empty()) {
      log_warning("Server traffic: the Traffic Manager cache did not load; building the road graph from the OpenDRIVE");
      local_map = std::make_shared<InMemoryMap>(world_map);
    }
    local_map->SetUp();
  }

  void ServerTrafficManager::Step() {
    if (local_map == nullptr) {
      return;
    }

    const world::Timestamp timestamp = world.GetTimestamp();

    // Updating simulation state, actor life cycle and performing necessary cleanup.
    alsm.Update();

    // Re-allocating inter-stage frames when the registered vehicles changed.
    const int current_registered_vehicles_state = registered_vehicles.GetState();
    unsigned long number_of_vehicles = vehicle_id_list.size();
    if (registered_vehicles_state != current_registered_vehicles_state ||
        number_of_vehicles != registered_vehicles.Size()) {
      vehicle_id_list = registered_vehicles.GetIDList();
      number_of_vehicles = vehicle_id_list.size();

      const uint64_t growth_factor = static_cast<uint64_t>(
          static_cast<float>(number_of_vehicles) * INV_GROWTH_STEP_SIZE);
      const uint64_t new_frame_capacity = INITIAL_SIZE + GROWTH_STEP_SIZE * growth_factor;
      if (new_frame_capacity > current_reserved_capacity) {
        localization_frame.reserve(new_frame_capacity);
        collision_frame.reserve(new_frame_capacity);
        tl_frame.reserve(new_frame_capacity);
        control_frame.reserve(new_frame_capacity);
        current_reserved_capacity = new_frame_capacity;
      }
      registered_vehicles_state = registered_vehicles.GetState();
    }

    localization_frame.clear();
    localization_frame.resize(number_of_vehicles);
    collision_frame.clear();
    collision_frame.resize(number_of_vehicles);
    tl_frame.clear();
    tl_frame.resize(number_of_vehicles);
    control_frame.clear();
    // One ApplyVehicleControl per vehicle, plus an optional light-state command.
    control_frame.reserve(2 * number_of_vehicles);
    control_frame.resize(number_of_vehicles);

    for (unsigned long index = 0u; index < vehicle_id_list.size(); ++index) {
      localization_stage.Update(index);
    }
    for (unsigned long index = 0u; index < vehicle_id_list.size(); ++index) {
      if (!localization_frame[index].localized) {
        continue;
      }
      collision_stage.Update(index);
    }
    collision_stage.ClearCycleCache();
    vehicle_light_stage.UpdateWorldInfo(timestamp.elapsed_seconds, true);
    motion_plan_stage.SetCycleTimestamp(timestamp);
    for (unsigned long index = 0u; index < vehicle_id_list.size(); ++index) {
      if (!localization_frame[index].localized) {
        continue;
      }
      traffic_light_stage.Update(index);
      motion_plan_stage.Update(index);
      vehicle_light_stage.Update(index);
    }

    // A vehicle skipped above never had its control written; drop its slot
    // rather than apply a default-constructed command.
    const bool any_skipped = std::any_of(
        localization_frame.begin(),
        localization_frame.begin() + static_cast<long>(vehicle_id_list.size()),
        [](const LocalizationData &data) { return !data.localized; });
    if (any_skipped) {
      ControlFrame filtered_frame;
      filtered_frame.reserve(control_frame.size());
      for (unsigned long index = 0u; index < control_frame.size(); ++index) {
        if (index < vehicle_id_list.size() && !localization_frame[index].localized) {
          continue;
        }
        filtered_frame.push_back(std::move(control_frame[index]));
      }
      control_frame = std::move(filtered_frame);
    }

    // Applied directly, this frame, before physics: no batch, no round trip.
    world.ApplyCommands(control_frame);

    // Vehicles dropped this cycle are released here, outside the stages.
    registered_vehicles.TakeReleased();
  }

  void ServerTrafficManager::RegisterVehicles(const std::vector<ActorPtr> &vehicles) {
    registered_vehicles.Insert(vehicles);
    for (const auto &actor : vehicles) {
      alsm.AddActor(actor);
    }
  }

  void ServerTrafficManager::UnregisterVehicles(const std::vector<ActorPtr> &vehicles) {
    for (const auto &actor : vehicles) {
      alsm.RemoveActor(actor->GetId(), true);
    }
  }

  std::vector<ActorId> ServerTrafficManager::GetRegisteredVehiclesIDs() {
    return registered_vehicles.GetIDList();
  }

} // namespace servertraffic
} // namespace carla
