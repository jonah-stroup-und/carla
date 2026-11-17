// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/geom/Location.h"
#include "carla/rpc/ActorId.h"
#include "carla/rpc/TrafficLightState.h"

#include <vector>

namespace carla {
namespace servercrowd {

  /// What the server-side crowd reads from the simulation. The client crowd
  /// (carla/nav) gets the same through client::World; the server implements
  /// this on its actor registry.
  class CrowdWorld {
  public:

    /// A traffic light and one of the places where it stops traffic: the
    /// stop waypoints of client::TrafficLight::GetStopWaypoints, in meters.
    struct TrafficLightStop {
      rpc::ActorId traffic_light;
      geom::Location location;
    };

    virtual ~CrowdWorld() = default;

    /// Every traffic light's stop locations, read once when the crowd starts.
    virtual std::vector<TrafficLightStop> GetTrafficLightStops() = 0;

    /// The current state of a traffic light from GetTrafficLightStops().
    virtual rpc::TrafficLightState GetTrafficLightState(rpc::ActorId traffic_light) = 0;
  };

} // namespace servercrowd
} // namespace carla
