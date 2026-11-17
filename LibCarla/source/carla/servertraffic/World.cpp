// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/servertraffic/World.h"

#include "carla/road/element/RoadInfoMarkRecord.h"
#include "carla/road/element/RoadInfoSignal.h"
#include "carla/road/Lane.h"
#include "carla/road/Road.h"
#include "carla/road/Signal.h"

#include <type_traits>
#include <unordered_map>
#include <unordered_set>

namespace carla {
namespace servertraffic {
namespace world {

  // ===========================================================================
  // -- Waypoint (mirrors carla/client/Waypoint.cpp) ---------------------------
  // ===========================================================================

  Waypoint::Waypoint(SharedPtr<const Map> parent, road::element::Waypoint waypoint)
    : _parent(std::move(parent)),
      _waypoint(std::move(waypoint)),
      _transform(_parent->GetMap().ComputeTransform(_waypoint)),
      _mark_record(_parent->GetMap().GetMarkRecord(_waypoint)) {}

  road::JuncId Waypoint::GetJunctionId() const {
    return _parent->GetMap().GetJunctionId(_waypoint.road_id);
  }

  bool Waypoint::IsJunction() const {
    return _parent->GetMap().IsJunction(_waypoint.road_id);
  }

  double Waypoint::GetLaneWidth() const {
    return _parent->GetMap().GetLaneWidth(_waypoint);
  }

  road::Lane::LaneType Waypoint::GetType() const {
    return _parent->GetMap().GetLaneType(_waypoint);
  }

  bool Waypoint::IsPositiveDirection() const {
    return _parent->GetMap().GetLane(_waypoint).IsPositiveDirection();
  }

  bool Waypoint::IsRHT() const {
    return _parent->GetMap().GetLane(_waypoint).GetRoad()->IsRHT();
  }

  std::vector<SharedPtr<Waypoint>> Waypoint::GetNext(double distance) const {
    auto waypoints = _parent->GetMap().GetNext(_waypoint, distance);
    std::vector<SharedPtr<Waypoint>> result;
    result.reserve(waypoints.size());
    for (auto &waypoint : waypoints) {
      result.emplace_back(MakeShared<Waypoint>(_parent, std::move(waypoint)));
    }
    return result;
  }

  SharedPtr<Waypoint> Waypoint::GetRight() const {
    auto right = _parent->GetMap().GetRight(_waypoint);
    return right.has_value() ? MakeShared<Waypoint>(_parent, std::move(*right)) : nullptr;
  }

  SharedPtr<Waypoint> Waypoint::GetLeft() const {
    auto left = _parent->GetMap().GetLeft(_waypoint);
    return left.has_value() ? MakeShared<Waypoint>(_parent, std::move(*left)) : nullptr;
  }

  template <typename EnumT>
  static EnumT operator&(EnumT lhs, EnumT rhs) {
    return static_cast<EnumT>(
        static_cast<typename std::underlying_type<EnumT>::type>(lhs) &
        static_cast<typename std::underlying_type<EnumT>::type>(rhs));
  }

  template <typename EnumT>
  static EnumT operator|(EnumT lhs, EnumT rhs) {
    return static_cast<EnumT>(
        static_cast<typename std::underlying_type<EnumT>::type>(lhs) |
        static_cast<typename std::underlying_type<EnumT>::type>(rhs));
  }

  road::element::LaneMarking::LaneChange Waypoint::GetLaneChange() const {
    using lane_change_type = road::element::LaneMarking::LaneChange;

    const auto road = _parent->GetMap().GetLane(_waypoint).GetRoad();

    const auto lane_change_right_info = road->IsRHT() ? _mark_record.first : _mark_record.second;
    lane_change_type c_right;
    if (lane_change_right_info != nullptr) {
      c_right = static_cast<lane_change_type>(lane_change_right_info->GetLaneChange());
    } else {
      c_right = lane_change_type::Both;
    }

    const auto lane_change_left_info = road->IsRHT() ? _mark_record.second : _mark_record.first;
    lane_change_type c_left;
    if (lane_change_left_info != nullptr) {
      c_left = static_cast<lane_change_type>(lane_change_left_info->GetLaneChange());
    } else {
      c_left = lane_change_type::Both;
    }

    if (!IsPositiveDirection()) {
      if (c_right == lane_change_type::Right) {
        c_right = lane_change_type::Left;
      } else if (c_right == lane_change_type::Left) {
        c_right = lane_change_type::Right;
      }
    }

    if (((!IsPositiveDirection()) ? _waypoint.lane_id - 1 : _waypoint.lane_id + 1) < 0) {
      if (c_left == lane_change_type::Right) {
        c_left = lane_change_type::Left;
      } else if (c_left == lane_change_type::Left) {
        c_left = lane_change_type::Right;
      }
    }

    return (c_right & lane_change_type::Right) | (c_left & lane_change_type::Left);
  }

  std::vector<SharedPtr<Waypoint::Landmark>> Waypoint::GetAllLandmarksInDistance(
      double distance, bool stop_at_junction) const {
    std::vector<SharedPtr<Landmark>> result;
    auto signals = _parent->GetMap().GetSignalsInDistance(_waypoint, distance, stop_at_junction);
    std::unordered_set<const road::element::RoadInfoSignal *> added_signals;
    for (auto &signal_data : signals) {
      if (added_signals.count(signal_data.signal) > 0) {
        continue;
      }
      added_signals.insert(signal_data.signal);
      const auto *signal = signal_data.signal->GetSignal();
      result.emplace_back(MakeShared<Landmark>(
          MakeShared<Waypoint>(_parent, signal_data.waypoint),
          signal->GetType(),
          signal->GetValue()));
    }
    return result;
  }

  // ===========================================================================
  // -- Map (mirrors carla/client/Map.cpp) -------------------------------------
  // ===========================================================================

  SharedPtr<Waypoint> Map::GetWaypointXODR(road::RoadId road_id, road::LaneId lane_id, float s) const {
    auto waypoint = _map.GetWaypoint(road_id, lane_id, s);
    return waypoint.has_value() ? MakeShared<Waypoint>(shared_from_this(), *waypoint) : nullptr;
  }

  Map::TopologyList Map::GetTopology() const {
    namespace re = carla::road::element;
    std::unordered_map<re::Waypoint, SharedPtr<Waypoint>> waypoints;

    auto get_or_make_waypoint = [&](const auto &waypoint) {
      auto it = waypoints.find(waypoint);
      if (it == waypoints.end()) {
        it = waypoints.emplace(waypoint, MakeShared<Waypoint>(shared_from_this(), waypoint)).first;
      }
      return it->second;
    };

    TopologyList result;
    auto topology = _map.GenerateTopology();
    result.reserve(topology.size());
    for (const auto &pair : topology) {
      result.emplace_back(get_or_make_waypoint(pair.first), get_or_make_waypoint(pair.second));
    }
    return result;
  }

  std::vector<SharedPtr<Waypoint>> Map::GenerateWaypoints(double distance) const {
    std::vector<SharedPtr<Waypoint>> result;
    const auto waypoints = _map.GenerateWaypoints(distance);
    result.reserve(waypoints.size());
    for (const auto &waypoint : waypoints) {
      result.emplace_back(MakeShared<Waypoint>(shared_from_this(), waypoint));
    }
    return result;
  }

} // namespace world
} // namespace servertraffic
} // namespace carla
