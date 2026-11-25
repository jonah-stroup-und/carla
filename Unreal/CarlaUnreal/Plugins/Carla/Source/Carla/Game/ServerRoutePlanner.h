// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <util/ue-header-guard-begin.h>
#include "CoreMinimal.h"
#include <util/ue-header-guard-end.h>

namespace carla { namespace road { class Map; } }

/// The map's driving lanes as a graph of legal moves, for routing the server
/// player's missions.
///
/// Every driving lane is sampled every couple of metres; each sample leads
/// to the next one along the lane's direction of travel, the last one to the
/// start of each lane it connects to (so junctions only allow the turns the
/// map's OpenDRIVE has), and, outside junctions, to a point a little further
/// on in a neighbouring lane going the same way, when the line between the
/// two lanes is broken. A lane change costs a little extra, so routes do not
/// weave. Locations are Unreal's (cm).
class FServerRoadGraph
{
public:

  void Build(const carla::road::Map &Map);

  bool IsEmpty() const { return Nodes.Num() == 0; }

  int32 Num() const { return Nodes.Num(); }

  const FVector &GetLocation(int32 Node) const { return Nodes[Node].Location; }

  /// The lane's direction of travel at the node (unit vector).
  const FVector &GetDirection(int32 Node) const { return Nodes[Node].Direction; }

  /// The node nearest Location within MaxDistance, preferring lanes heading
  /// along Forward (a zero Forward: any direction). INDEX_NONE when none.
  int32 FindNearest(const FVector &Location, const FVector &Forward, float MaxDistance) const;

  /// Where a car at Location heading along Forward can start a route: the
  /// nearest node (as FindNearest) and every other node as near to the car
  /// and heading its way, e.g. at a junction's entrance, the start of each
  /// turn through it, which all begin at the same spot. Empty when none.
  TArray<int32> FindStarts(const FVector &Location, const FVector &Forward, float MaxDistance) const;

  /// The lane a goal at Target is on: the nearest node within Radius,
  /// outside a junction if there is one. INDEX_NONE when none.
  int32 FindGoalLane(const FVector &Target, float Radius) const;

  /// Whether Node counts as arriving at a goal at Target: within Radius
  /// (horizontally) and travelling the way the goal's lane (GoalLane, from
  /// FindGoalLane) does. Not the lanes on the other side of the street, nor
  /// a junction's turning lanes that cut across the goal.
  bool IsGoal(int32 Node, int32 GoalLane, const FVector &Target, float Radius) const;

  /// The shortest legal route from any of Starts to a goal node (IsGoal)
  /// of Target, as nodes from that start to that node. False when there is
  /// none.
  bool FindRoute(const TArray<int32> &Starts, const FVector &Target, float Radius, TArray<int32> &OutPath) const;

private:

  struct FNode
  {
    FVector Location = FVector::ZeroVector;
    FVector Direction = FVector::ForwardVector;
    bool bJunction = false;
  };

  struct FEdge
  {
    int32 To = 0;
    float Cost = 0.0f;
  };

  static FIntPoint CellOf(const FVector &Location);

  TArray<FNode> Nodes;
  TArray<TArray<FEdge>> Edges;
  TMap<FIntPoint, TArray<int32>> Grid;
};
