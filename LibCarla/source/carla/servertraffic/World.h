// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

/// The server-side Traffic Manager's view of the simulation.
///
/// The client Traffic Manager (carla/trafficmanager) reads the world through
/// carla::client — every read and every control command is an RPC. This copy
/// runs inside the server, so it reads the world through the small interfaces
/// below, which the Unreal plugin implements over its actor registry, and
/// through Waypoint/Map, which wrap the road::Map the server already holds.
/// The class and method names follow carla::client on purpose, so the copied
/// stages stay line-for-line comparable with the originals.

#pragma once

#include "carla/Memory.h"
#include "carla/geom/BoundingBox.h"
#include "carla/geom/Location.h"
#include "carla/geom/Transform.h"
#include "carla/geom/Vector3D.h"
#include "carla/road/Map.h"
#include "carla/road/RoadTypes.h"
#include "carla/road/element/LaneMarking.h"
#include "carla/road/element/Waypoint.h"
#include "carla/rpc/ActorId.h"
#include "carla/rpc/Command.h"
#include "carla/rpc/TrafficLightState.h"
#include "carla/rpc/VehicleLightState.h"
#include "carla/rpc/VehicleLightStateList.h"
#include "carla/rpc/WeatherParameters.h"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace carla {
namespace servertraffic {
/// Stand-ins for carla::client, in their own namespace as the originals are
/// (the stages refer to them as cc::...).
namespace world {

  class Map;

  /// Mirror of carla::client::Timestamp.
  class Timestamp {
  public:
    Timestamp() = default;
    Timestamp(std::size_t in_frame, double in_elapsed_seconds,
              double in_delta_seconds, double in_platform_timestamp)
      : frame(in_frame),
        elapsed_seconds(in_elapsed_seconds),
        delta_seconds(in_delta_seconds),
        platform_timestamp(in_platform_timestamp) {}

    std::size_t frame = 0u;
    double elapsed_seconds = 0.0;
    double delta_seconds = 0.0;
    double platform_timestamp = 0.0;
  };

  /// Mirror of carla::client::Waypoint over the server's road::Map.
  class Waypoint : public EnableSharedFromThis<Waypoint> {
  public:
    Waypoint(SharedPtr<const Map> parent, road::element::Waypoint waypoint);

    uint64_t GetId() const {
      return std::hash<road::element::Waypoint>()(_waypoint);
    }
    auto GetRoadId() const { return _waypoint.road_id; }
    auto GetSectionId() const { return _waypoint.section_id; }
    auto GetLaneId() const { return _waypoint.lane_id; }
    auto GetDistance() const { return _waypoint.s; }
    const geom::Transform &GetTransform() const { return _transform; }
    const road::element::Waypoint &GetRaw() const { return _waypoint; }

    road::JuncId GetJunctionId() const;
    bool IsJunction() const;
    double GetLaneWidth() const;
    road::Lane::LaneType GetType() const;
    bool IsPositiveDirection() const;
    bool IsRHT() const;

    std::vector<SharedPtr<Waypoint>> GetNext(double distance) const;
    SharedPtr<Waypoint> GetRight() const;
    SharedPtr<Waypoint> GetLeft() const;
    road::element::LaneMarking::LaneChange GetLaneChange() const;

    class Landmark;
    std::vector<SharedPtr<Landmark>> GetAllLandmarksInDistance(
        double distance, bool stop_at_junction = false) const;

  private:
    SharedPtr<const Map> _parent;
    road::element::Waypoint _waypoint;
    geom::Transform _transform;
    std::pair<const road::element::RoadInfoMarkRecord *,
              const road::element::RoadInfoMarkRecord *> _mark_record;
  };

  /// Mirror of the parts of carla::client::Landmark the stages read.
  class Waypoint::Landmark {
  public:
    Landmark(SharedPtr<Waypoint> waypoint, std::string type, double value)
      : _waypoint(std::move(waypoint)), _type(std::move(type)), _value(value) {}
    SharedPtr<Waypoint> GetWaypoint() const { return _waypoint; }
    const std::string &GetType() const { return _type; }
    double GetValue() const { return _value; }
  private:
    SharedPtr<Waypoint> _waypoint;
    std::string _type;
    double _value;
  };

  /// Mirror of carla::client::Map over a road::Map the caller keeps alive.
  class Map : public EnableSharedFromThis<Map> {
  public:
    using TopologyList = std::vector<std::pair<SharedPtr<Waypoint>, SharedPtr<Waypoint>>>;

    Map(const road::Map &map, std::string name) : _map(map), _name(std::move(name)) {}

    const std::string &GetName() const { return _name; }
    const road::Map &GetMap() const { return _map; }

    SharedPtr<Waypoint> GetWaypointXODR(road::RoadId road_id, road::LaneId lane_id, float s) const;
    TopologyList GetTopology() const;
    std::vector<SharedPtr<Waypoint>> GenerateWaypoints(double distance) const;

  private:
    const road::Map &_map;
    std::string _name;
  };

  /// One actor attribute (id/value), as carla::client::ActorAttributeValue.
  class ActorAttribute {
  public:
    ActorAttribute(std::string id, std::string value)
      : _id(std::move(id)), _value(std::move(value)) {}
    const std::string &GetId() const { return _id; }
    const std::string &GetValue() const { return _value; }
  private:
    std::string _id;
    std::string _value;
  };

  /// An actor as the stages see it. The plugin implements this over its
  /// actor registry; every call is a direct read on the game thread.
  class Actor {
  public:
    virtual ~Actor() = default;

    virtual ActorId GetId() const = 0;
    /// Blueprint id, e.g. "vehicle.tesla.model3" or "walker.pedestrian.0001".
    virtual const std::string &GetTypeId() const = 0;
    virtual const std::vector<ActorAttribute> &GetAttributes() const = 0;
    virtual bool IsAlive() const = 0;

    virtual geom::Transform GetTransform() const = 0;
    virtual geom::Vector3D GetVelocity() const = 0;
    virtual geom::BoundingBox GetBoundingBox() const = 0;

    // Vehicle-only reads; a walker returns the defaults.
    virtual float GetSpeedLimit() const { return 30.0f; }
    virtual rpc::TrafficLightState GetTrafficLightState() const { return rpc::TrafficLightState::Green; }
    virtual bool IsAtTrafficLight() const { return false; }
    /// Largest front-wheel steering lock in degrees, or 0 when unknown.
    virtual float GetMaxSteerAngle() const { return 0.0f; }

    virtual void SetSimulatePhysics(bool enabled) = 0;
    virtual void SetTargetVelocity(const geom::Vector3D &velocity) = 0;
    virtual void Destroy() = 0;
  };

  // carla::client spells a vehicle and a walker as Actor subclasses; here they
  // are the same interface, so the stages' static_pointer_casts are no-ops.
  using Vehicle = Actor;
  using Walker = Actor;
  using ActorList = std::vector<SharedPtr<Actor>>;

  /// The simulation as the stages see it.
  class World {
  public:
    virtual ~World() = default;

    virtual Timestamp GetTimestamp() const = 0;
    virtual SharedPtr<ActorList> GetActors() const = 0;
    virtual SharedPtr<const Map> GetMap() const = 0;

    virtual rpc::VehicleLightStateList GetVehiclesLightStates() const = 0;
    virtual bool IsWeatherEnabled() const = 0;
    virtual rpc::WeatherParameters GetWeather() const = 0;

    /// Applies one cycle's commands (vehicle control, light state) now.
    virtual void ApplyCommands(const std::vector<rpc::Command> &commands) = 0;
  };

} // namespace world
} // namespace servertraffic
} // namespace carla
