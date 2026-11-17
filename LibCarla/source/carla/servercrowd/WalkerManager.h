// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

// Server-side copy of carla/nav: the crowd runs inside the server (see
// carla/servercrowd/ServerCrowd.h), on CARLA's Detour compiled in its own
// namespace (Detour.h). Kept close to carla/nav so fixes port both ways.

#pragma once

#include "carla/NonCopyable.h"

#include "carla/servercrowd/CrowdWorld.h"
#include "carla/geom/Location.h"
#include "carla/servercrowd/WalkerEvent.h"
#include "carla/rpc/ActorId.h"
#include "carla/rpc/TrafficLightState.h"

namespace carla {
namespace servercrowd {

    class Navigation;

    enum WalkerState {
        WALKER_IDLE,
        WALKER_WALKING,
        WALKER_IN_EVENT,
        WALKER_STOP
    };

    struct WalkerRoutePoint {
        WalkerEvent event;
        carla::geom::Location location;
        unsigned char areaType;
        WalkerRoutePoint(WalkerEvent ev, carla::geom::Location loc, unsigned char area) : event(ev), location(loc), areaType(area) {};
    };

    struct WalkerInfo {
        carla::geom::Location from;
        carla::geom::Location to;
        unsigned int currentIndex { 0 };
        WalkerState state;
        std::vector<WalkerRoutePoint> route;
    };

  class WalkerManager : private NonCopyable {

  public:

    WalkerManager();
    ~WalkerManager();

    /// assign the navigation module
    void SetNav(Navigation *nav);

    /// the simulation to read traffic lights from; must outlive this
    void SetWorld(CrowdWorld *world);

    /// create a new walker route
    bool AddWalker(ActorId id);

    /// remove a walker route
    bool RemoveWalker(ActorId id);

    /// update all routes
    bool Update(double delta);

    /// set a new route from its current position
    bool SetWalkerRoute(ActorId id);
    bool SetWalkerRoute(ActorId id, carla::geom::Location to);

    /// set the next point in the route
    bool SetWalkerNextPoint(ActorId id);
  
    /// get the next point in the route
    bool GetWalkerNextPoint(ActorId id, carla::geom::Location &location);

    /// get the point in the route that end current crosswalk
    bool GetWalkerCrosswalkEnd(ActorId id, carla::geom::Location &location);
    
    /// return the navigation object
    Navigation *GetNavigation() { return _nav; };

    /// return the trafficlight affecting that position
    /// (0 if none)
    ActorId GetTrafficLightAffecting(carla::geom::Location UnrealPos, float max_distance = -1.0f);

    /// state of a traffic light returned by GetTrafficLightAffecting
    carla::rpc::TrafficLightState GetTrafficLightState(ActorId traffic_light);

    private:

    void GetAllTrafficLightWaypoints();

    EventResult ExecuteEvent(ActorId id, WalkerInfo &info, double delta);

    std::unordered_map<ActorId, WalkerInfo> _walkers;
    std::vector<std::pair<ActorId, carla::geom::Location>> _traffic_lights;
    Navigation *_nav { nullptr };
    CrowdWorld *_world { nullptr };
    bool _traffic_lights_loaded { false };
  };

} // namespace servercrowd
} // namespace carla
