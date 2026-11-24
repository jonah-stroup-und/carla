// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <util/ue-header-guard-begin.h>
#include "CoreMinimal.h"
#include "InputCoreTypes.h"
#include <util/ue-header-guard-end.h>

#include "Carla/Game/ServerRoutePlanner.h"

class ACameraActor;
class ACarlaWheeledVehicle;
class AActor;
class APlayerController;
class FServerDriveReport;
class UCanvas;
class UCarlaEpisode;
class UMaterialInstanceDynamic;
class UProceduralMeshComponent;

/// The server player's missions: waypoints on the map to drive to in order.
///
/// Planning (M): the centre screen looks down on the town. Click a road to
/// add a waypoint at the end of the mission (it snaps to the nearest driving
/// lane), drag one to move it, right-click one to delete it; "," and "."
/// move the selected waypoint earlier or later, Backspace deletes it, Delete
/// twice clears the mission. The mouse wheel or +/- zoom, WASD or the arrow
/// keys pan (Shift: faster). The legal route through the waypoints is drawn
/// as you go. The car is held still meanwhile. M again starts the mission.
///
/// Driving: a green line on the road shows the best legal route to the next
/// waypoint (following lane directions and the turns each junction allows,
/// changing lanes only over broken lines), recomputed as you drive, and the
/// waypoint is a green ring with an arrow floating over it, pointing down. Driving into the ring scores the
/// waypoint: points_per_waypoint, less the penalty of each violation since
/// the previous one (never below zero), then the next waypoint lights up.
/// R restarts the mission. Each waypoint and the final score go in the
/// drive report.
///
/// Missions are JSON files, saved on every edit: Missions/<map>.json in the
/// project, or -server-mission=NAME (Missions/NAME.json) or =PATH. Locations
/// are CARLA's (meters):
///
///   {
///     "map": "Town10HD_Opt",
///     "radius_m": 6.0,               how close counts as reaching a waypoint
///     "points_per_waypoint": 100,
///     "penalties": {                 points off per violation, by the drive
///       "Collision with vehicle": 50,  report's event name
///       ...
///     },
///     "waypoints": [[x, y, z], ...]
///   }
class FServerMission
{
public:

  explicit FServerMission(UCarlaEpisode &Episode);
  ~FServerMission();

  FServerMission(const FServerMission &) = delete;
  FServerMission &operator=(const FServerMission &) = delete;

  bool IsPlanning() const { return bPlanning; }

  /// Enters or leaves the planner. Leaving it restarts the mission.
  void SetPlanning(bool bPlan, APlayerController &Controller, const ACarlaWheeledVehicle &Car);

  /// Back to the first waypoint, with no score.
  void Restart();

  /// Call once per tick: the planner's input and camera while planning,
  /// else the mission's progress and route.
  void Tick(APlayerController &Controller, ACarlaWheeledVehicle &Car, FServerDriveReport &Report, float DeltaSeconds);

  /// The mission's HUD line, its messages and the planner's markers, on the
  /// driver's view; Y is where the line goes.
  void Draw(UCanvas &Canvas, float ViewWidth, float Y);

private:

  // Files
  void Load();
  void Save() const;

  // Roads
  bool EnsureGraph();
  bool SnapToRoad(const FVector &Location, FVector &OutLocation) const;
  float GroundHeight(const FVector &Location) const;
  float NodeGroundHeight(int32 Node);
  bool RouteFrom(int32 Start, const FVector &Target, TArray<FVector> &OutPoints, int32 *OutEnd);

  // Driving
  void TickDriving(ACarlaWheeledVehicle &Car, FServerDriveReport &Report);
  void ReachWaypoint(FServerDriveReport &Report);
  void UpdateSummary(FServerDriveReport &Report) const;

  // Planning
  void TickPlanner(APlayerController &Controller, float DeltaSeconds);
  bool MouseOnRoad(APlayerController &Controller, FVector &OutLocation) const;
  int32 PickWaypoint(APlayerController &Controller) const;
  void UpdatePlannerCamera();
  void UpdatePlanLegs();

  // Drawing
  bool EnsureMesh();
  void ShowRoute(const TArray<TArray<FVector>> &Lines, float Width, const FVector *Ring, bool bArrow);
  void HideRoute();
  void Popup(const FString &Text, const FColor &Color, double Seconds);

  bool Pressed(const APlayerController &Controller, const FKey &Key);

  UCarlaEpisode &Episode;
  FString Path;
  FString MapName;

  // The mission
  TArray<FVector> Waypoints; // cm
  float Radius = 600.0f;     // cm
  int32 PointsPerWaypoint = 100;
  TArray<TPair<FString, int32>> Penalties;

  // Progress
  int32 Target = 0;
  int32 Score = 0;
  int32 LegPenalty = 0;
  TArray<FString> LegViolations;
  const FServerDriveReport *LastReport = nullptr;
  int32 ReportCursor = 0;
  bool bStarted = false;
  bool bComplete = false;
  bool bNoRoute = false;
  float RouteLength = 0.0f; // cm
  double NextRouteTime = 0.0;

  // Roads
  FServerRoadGraph Graph;
  bool bGraphTried = false;
  TMap<int32, float> NodeGround;

  // Drawing
  TWeakObjectPtr<AActor> RouteActor;
  TWeakObjectPtr<UProceduralMeshComponent> RouteMesh;
  TWeakObjectPtr<UMaterialInstanceDynamic> LineMaterial;
  TWeakObjectPtr<UMaterialInstanceDynamic> ArrowMaterial;
  FString PopupText;
  FColor PopupColor = FColor::White;
  double PopupUntil = 0.0;

  // Planning
  bool bPlanning = false;
  TWeakObjectPtr<ACameraActor> PlannerCamera;
  FVector Focus = FVector::ZeroVector;
  float Height = 20000.0f; // cm above the focus
  int32 Selected = INDEX_NONE;
  bool bDragging = false;
  bool bDragMoved = false;
  bool bPlanDirty = true;
  float PlanWidth = 0.0f;
  double NextPlanTime = 0.0;
  double ClearPromptTime = -1000.0;
  TArray<TArray<FVector>> PlanLegs;
  TArray<bool> PlanLegOk;
  FVector CarLocation = FVector::ZeroVector;
  FVector CarForward = FVector::ForwardVector;
  TMap<FName, bool> KeysDown;
};
