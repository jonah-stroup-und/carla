// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Game/ServerRoutePlanner.h"
#include "Carla.h"

#include <util/ue-header-guard-begin.h>
#include "Algo/Reverse.h"
#include <util/ue-header-guard-end.h>

#include <util/disable-ue4-macros.h>
#include <carla/geom/Location.h>
#include <carla/geom/Transform.h>
#include <carla/road/Map.h>
#include <carla/road/element/LaneMarking.h>
#include <carla/road/element/RoadInfoMarkRecord.h>
#include <util/enable-ue4-macros.h>

DEFINE_LOG_CATEGORY_STATIC(LogCarlaServerRoute, Log, All);

namespace
{
  using FRouteWaypoint = carla::road::element::Waypoint;
  using ERouteMarking = carla::road::element::LaneMarking::Type;

  /// Lanes are sampled this often (m).
  constexpr double RouteNodeSpacing = 2.0;
  /// A lane change lands this far along the neighbouring lane (m).
  constexpr double RouteLaneChangeLength = 8.0;
  /// And costs this much on top of its length (cm), so a route only changes
  /// lanes when it has to.
  constexpr float RouteLaneChangePenalty = 1500.0f;
  /// The spatial grid's cell size (cm).
  constexpr float RouteGridCell = 2000.0f;
  /// Goal nodes must be within this height of the target (cm): not a bridge
  /// above it.
  constexpr float RouteGoalHeight = 500.0f;

  uint64 RouteLaneKey(const FRouteWaypoint &Waypoint)
  {
    return (static_cast<uint64>(Waypoint.road_id) << 32) |
        (static_cast<uint64>(Waypoint.section_id & 0xFFFFu) << 16) |
        static_cast<uint64>(static_cast<uint16>(static_cast<int16>(Waypoint.lane_id)));
  }

  /// Lines a lane change may cross: the same ones the drive report does not
  /// call solid.
  bool RouteCanChangeLanesOver(ERouteMarking Type)
  {
    return Type == ERouteMarking::None || Type == ERouteMarking::Broken ||
        Type == ERouteMarking::BrokenBroken || Type == ERouteMarking::BottsDots;
  }

  float RouteDistance2D(const FVector &A, const FVector &B)
  {
    return FVector::Dist2D(A, B);
  }
}

FIntPoint FServerRoadGraph::CellOf(const FVector &Location)
{
  return FIntPoint(FMath::FloorToInt(Location.X / RouteGridCell), FMath::FloorToInt(Location.Y / RouteGridCell));
}

void FServerRoadGraph::Build(const carla::road::Map &Map)
{
  const double StartTime = FPlatformTime::Seconds();
  Nodes.Reset();
  Edges.Reset();
  Grid.Reset();

  struct FLane
  {
    double S0 = 0.0;
    double Sign = 1.0;
    int32 First = 0;
    int32 Num = 0;
  };
  TMap<uint64, FLane> Lanes;
  TArray<FRouteWaypoint> Waypoints;

  const auto AddNode = [&](const FRouteWaypoint &Waypoint) {
    const carla::geom::Transform Transform = Map.ComputeTransform(Waypoint);
    const carla::geom::Vector3D Forward = Transform.GetForwardVector();
    FNode Node;
    Node.Location = FVector(Transform.location);
    Node.Direction = FVector(Forward.x, Forward.y, Forward.z).GetSafeNormal();
    Nodes.Add(Node);
    Edges.AddDefaulted();
    Waypoints.Add(Waypoint);
  };

  const auto NodeAt = [&](const FLane &Lane, double S) {
    const int32 Index = FMath::RoundToInt((S - Lane.S0) * Lane.Sign / RouteNodeSpacing);
    return Lane.First + FMath::Clamp(Index, 0, Lane.Num - 1);
  };

  // Each lane (per lane section), sampled from its start in its direction
  // of travel.
  const auto Topology = Map.GenerateTopology();
  for (const auto &Pair : Topology)
  {
    const FRouteWaypoint &Start = Pair.first;
    const uint64 Key = RouteLaneKey(Start);
    if (Lanes.Contains(Key))
    {
      continue;
    }
    FLane Lane;
    Lane.First = Nodes.Num();
    Lane.S0 = Start.s;
    FRouteWaypoint Waypoint = Start;
    while (true)
    {
      AddNode(Waypoint);
      bool bFound = false;
      for (const FRouteWaypoint &Next : Map.GetNext(Waypoint, RouteNodeSpacing))
      {
        if (RouteLaneKey(Next) == Key)
        {
          Waypoint = Next;
          bFound = true;
          break;
        }
      }
      if (!bFound)
      {
        break;
      }
    }
    Lane.Num = Nodes.Num() - Lane.First;
    Lane.Sign = Lane.Num > 1 && Waypoints[Lane.First + 1].s < Lane.S0 ? -1.0 : 1.0;
    for (int32 Index = Lane.First; Index + 1 < Lane.First + Lane.Num; ++Index)
    {
      Edges[Index].Add({Index + 1, static_cast<float>(FVector::Dist(Nodes[Index].Location, Nodes[Index + 1].Location))});
    }
    Lanes.Add(Key, Lane);
  }

  // The end of each lane to the start of the lanes it leads to.
  int32 Connections = 0;
  for (const auto &Pair : Topology)
  {
    const FLane *From = Lanes.Find(RouteLaneKey(Pair.first));
    const FLane *To = Lanes.Find(RouteLaneKey(Pair.second));
    if (From == nullptr || To == nullptr)
    {
      continue;
    }
    const int32 Last = From->First + From->Num - 1;
    const int32 Next = NodeAt(*To, Pair.second.s);
    if (Last != Next)
    {
      Edges[Last].Add({Next, static_cast<float>(FVector::Dist(Nodes[Last].Location, Nodes[Next].Location))});
      ++Connections;
    }
  }

  // Lane changes, outside junctions, over broken lines, to a lane going the
  // same way.
  int32 LaneChanges = 0;
  const int32 NumLaneNodes = Nodes.Num();
  for (int32 Node = 0; Node < NumLaneNodes; ++Node)
  {
    const FRouteWaypoint Waypoint = Waypoints[Node];
    if (Map.IsJunction(Waypoint.road_id))
    {
      continue;
    }
    for (int32 Side = 0; Side < 2; ++Side)
    {
      const std::optional<FRouteWaypoint> Neighbour = Side == 0 ? Map.GetLeft(Waypoint) : Map.GetRight(Waypoint);
      if (!Neighbour.has_value() || (Neighbour->lane_id > 0) != (Waypoint.lane_id > 0))
      {
        continue;
      }
      const FLane *NeighbourLane = Lanes.Find(RouteLaneKey(*Neighbour));
      if (NeighbourLane == nullptr)
      {
        continue; // not a driving lane
      }
      // A lane's road mark is on its outer edge: the line between the two
      // is the inner lane's.
      const FRouteWaypoint &Inner = FMath::Abs(Waypoint.lane_id) < FMath::Abs(Neighbour->lane_id) ? Waypoint : *Neighbour;
      const auto *Record = Map.GetMarkRecord(Inner).first;
      if (Record != nullptr && !RouteCanChangeLanesOver(carla::road::element::LaneMarking(*Record).type))
      {
        continue;
      }
      for (const FRouteWaypoint &Ahead : Map.GetNext(*Neighbour, RouteLaneChangeLength))
      {
        if (RouteLaneKey(Ahead) != RouteLaneKey(*Neighbour))
        {
          continue; // the lane ends first
        }
        const int32 To = NodeAt(*NeighbourLane, Ahead.s);
        Edges[Node].Add({To, static_cast<float>(FVector::Dist(Nodes[Node].Location, Nodes[To].Location)) + RouteLaneChangePenalty});
        ++LaneChanges;
        break;
      }
    }
  }

  for (int32 Node = 0; Node < Nodes.Num(); ++Node)
  {
    Grid.FindOrAdd(CellOf(Nodes[Node].Location)).Add(Node);
  }
  UE_LOG(LogCarlaServerRoute, Log, TEXT("Route planner: %d lanes, %d points, %d connections, %d lane changes (%.0f ms)"),
      Lanes.Num(), Nodes.Num(), Connections, LaneChanges, (FPlatformTime::Seconds() - StartTime) * 1000.0);
}

int32 FServerRoadGraph::FindNearest(const FVector &Location, const FVector &Forward, float MaxDistance) const
{
  const FVector Heading = FVector(Forward.X, Forward.Y, 0.0f).GetSafeNormal();
  const int32 Reach = FMath::CeilToInt(MaxDistance / RouteGridCell);
  const FIntPoint Cell = CellOf(Location);
  int32 Best = INDEX_NONE;
  float BestScore = TNumericLimits<float>::Max();
  for (int32 DX = -Reach; DX <= Reach; ++DX)
  {
    for (int32 DY = -Reach; DY <= Reach; ++DY)
    {
      const TArray<int32> *Bucket = Grid.Find(Cell + FIntPoint(DX, DY));
      if (Bucket == nullptr)
      {
        continue;
      }
      for (int32 Node : *Bucket)
      {
        const float Distance = FVector::Dist(Nodes[Node].Location, Location);
        if (Distance > MaxDistance)
        {
          continue;
        }
        // Lanes heading the other way, or across, count as further away.
        float Score = Distance;
        if (!Heading.IsZero())
        {
          const float Along = FVector::DotProduct(FVector(Nodes[Node].Direction.X, Nodes[Node].Direction.Y, 0.0f).GetSafeNormal(), Heading);
          Score += (1.0f - Along) * 400.0f;
        }
        if (Score < BestScore)
        {
          BestScore = Score;
          Best = Node;
        }
      }
    }
  }
  return Best;
}

bool FServerRoadGraph::FindRoute(int32 Start, const FVector &Target, float Radius, TArray<int32> &OutPath) const
{
  OutPath.Reset();
  if (!Nodes.IsValidIndex(Start))
  {
    return false;
  }
  const auto IsGoal = [&](int32 Node) {
    return RouteDistance2D(Nodes[Node].Location, Target) <= Radius &&
        FMath::Abs(Nodes[Node].Location.Z - Target.Z) <= RouteGoalHeight;
  };
  const auto Heuristic = [&](int32 Node) {
    return FMath::Max(RouteDistance2D(Nodes[Node].Location, Target) - Radius, 0.0f);
  };

  struct FOpen
  {
    float Priority;
    int32 Node;
    bool operator<(const FOpen &Other) const { return Priority < Other.Priority; }
  };
  TArray<float> Cost;
  Cost.Init(TNumericLimits<float>::Max(), Nodes.Num());
  TArray<int32> Parent;
  Parent.Init(INDEX_NONE, Nodes.Num());
  TArray<bool> Closed;
  Closed.Init(false, Nodes.Num());
  TArray<FOpen> Open;
  Cost[Start] = 0.0f;
  Open.HeapPush({Heuristic(Start), Start});
  int32 Goal = INDEX_NONE;
  while (Open.Num() > 0)
  {
    FOpen Top;
    Open.HeapPop(Top, EAllowShrinking::No);
    if (Closed[Top.Node])
    {
      continue;
    }
    Closed[Top.Node] = true;
    if (IsGoal(Top.Node))
    {
      Goal = Top.Node;
      break;
    }
    for (const FEdge &Edge : Edges[Top.Node])
    {
      const float NewCost = Cost[Top.Node] + Edge.Cost;
      if (NewCost < Cost[Edge.To])
      {
        Cost[Edge.To] = NewCost;
        Parent[Edge.To] = Top.Node;
        Open.HeapPush({NewCost + Heuristic(Edge.To), Edge.To});
      }
    }
  }
  if (Goal == INDEX_NONE)
  {
    return false;
  }
  for (int32 Node = Goal; Node != INDEX_NONE; Node = Parent[Node])
  {
    OutPath.Add(Node);
  }
  Algo::Reverse(OutPath);
  return true;
}
