// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Game/ServerMission.h"
#include "Carla.h"

#include "Carla/Game/CarlaEpisode.h"
#include "Carla/Game/CarlaGameModeBase.h"
#include "Carla/Game/CarlaStatics.h"
#include "Carla/Game/ServerDriveReport.h"
#include "Carla/Vehicle/CarlaWheeledVehicle.h"

#include <util/disable-ue4-macros.h>
#include <carla/geom/Location.h>
#include <carla/geom/Transform.h>
#include <carla/road/Map.h>
#include <util/enable-ue4-macros.h>

#include <util/ue-header-guard-begin.h>
#include "Camera/CameraActor.h"
#include "Camera/CameraComponent.h"
#include "CollisionQueryParams.h"
#include "Dom/JsonObject.h"
#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "Engine/Font.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "HAL/FileManager.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"
#include "ProceduralMeshComponent.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include <util/ue-header-guard-end.h>

DEFINE_LOG_CATEGORY_STATIC(LogCarlaServerMission, Log, All);

namespace
{
  const TCHAR *MissionMaterialPath = TEXT("/Carla/Mission/M_ServerRoute.M_ServerRoute");
  /// The route is recomputed this often while driving (s).
  constexpr double MissionRouteInterval = 0.5;
  /// Speeding is not penalised up to this far over the limit...
  constexpr float MissionSpeedingGraceMph = 5.0f;
  /// ...and beyond it costs its penalty once per started step this wide.
  constexpr float MissionSpeedingStepMph = 5.0f;
  /// And the planner's legs at most this often while dragging (s).
  constexpr double MissionPlanInterval = 0.15;
  /// The car's lane: the nearest lane point within this (cm), preferring its
  /// heading; failing that, any within the wider reach.
  constexpr float MissionStartReach = 600.0f;
  constexpr float MissionStartFarReach = 3000.0f;
  /// A click this far (cm) from any driving lane is not on a road.
  constexpr float MissionSnapReach = 1500.0f;
  /// A waypoint is reached within its radius horizontally and this vertically.
  constexpr float MissionReachHeight = 500.0f;
  /// The road surface is looked for this far (cm) above and below the
  /// OpenDRIVE elevation.
  constexpr float MissionGroundReach = 60.0f;
  /// The line on the road: width and height above it (cm).
  constexpr float MissionLineWidth = 70.0f;
  constexpr float MissionLineLift = 12.0f;
  /// The waypoint's ring width (cm), and the arrow floating over it, tip
  /// down: the tip's height above the road, the head's length and radius,
  /// the shaft's length and radius (cm).
  constexpr float MissionRingWidth = 40.0f;
  constexpr float MissionArrowTip = 120.0f;
  constexpr float MissionArrowHead = 160.0f;
  constexpr float MissionArrowHeadRadius = 110.0f;
  constexpr float MissionArrowShaft = 200.0f;
  constexpr float MissionArrowShaftRadius = 40.0f;
  /// The planner camera's height range (cm) and pan speed (heights per second).
  constexpr float MissionMinHeight = 3000.0f;
  constexpr float MissionMaxHeight = 150000.0f;
  constexpr float MissionPanSpeed = 0.8f;
  /// Clicks within this many pixels of a waypoint pick it.
  constexpr float MissionPickPixels = 20.0f;
  constexpr double MissionClearConfirmSeconds = 3.0;

  enum EMissionSection : int32 { MissionLines = 0, MissionRing = 1, MissionArrow = 2 };

  /// A flat strip along the points, Width wide, Lift above them.
  void AppendMissionStrip(const TArray<FVector> &Points, float Width, float Lift, TArray<FVector> &Vertices, TArray<int32> &Triangles)
  {
    TArray<FVector> Clean;
    for (const FVector &Point : Points)
    {
      if (Clean.Num() == 0 || FVector::Dist2D(Clean.Last(), Point) > 30.0f)
      {
        Clean.Add(Point);
      }
    }
    if (Clean.Num() < 2)
    {
      return;
    }
    const int32 Base = Vertices.Num();
    for (int32 Index = 0; Index < Clean.Num(); ++Index)
    {
      FVector Along = Clean[FMath::Min(Index + 1, Clean.Num() - 1)] - Clean[FMath::Max(Index - 1, 0)];
      Along.Z = 0.0f;
      Along = Along.GetSafeNormal();
      const FVector Side(-Along.Y, Along.X, 0.0f);
      const FVector Center = Clean[Index] + FVector(0.0f, 0.0f, Lift);
      Vertices.Add(Center + Side * Width * 0.5f);
      Vertices.Add(Center - Side * Width * 0.5f);
    }
    for (int32 Index = 0; Index + 1 < Clean.Num(); ++Index)
    {
      const int32 A = Base + 2 * Index;
      Triangles.Append({A, A + 2, A + 1, A + 1, A + 2, A + 3});
    }
  }

  /// A flat ring (annulus) around Center.
  void AppendMissionRing(const FVector &Center, float Radius, float Width, TArray<FVector> &Vertices, TArray<int32> &Triangles)
  {
    constexpr int32 Segments = 48;
    const int32 Base = Vertices.Num();
    for (int32 Index = 0; Index <= Segments; ++Index)
    {
      const float Angle = 2.0f * PI * Index / Segments;
      const FVector Direction(FMath::Cos(Angle), FMath::Sin(Angle), 0.0f);
      Vertices.Add(Center + Direction * (Radius - Width * 0.5f));
      Vertices.Add(Center + Direction * (Radius + Width * 0.5f));
    }
    for (int32 Index = 0; Index < Segments; ++Index)
    {
      const int32 A = Base + 2 * Index;
      Triangles.Append({A, A + 2, A + 1, A + 1, A + 2, A + 3});
    }
  }

  /// A solid arrow floating over Ground, pointing down at it: a cone (the
  /// head, tip down) under a closed cylinder (the shaft).
  void AppendMissionArrow(const FVector &Ground, TArray<FVector> &Vertices, TArray<int32> &Triangles)
  {
    constexpr int32 Segments = 24;
    const FVector Tip = Ground + FVector(0.0f, 0.0f, MissionArrowTip);
    const FVector HeadBase = Tip + FVector(0.0f, 0.0f, MissionArrowHead);
    const FVector ShaftTop = HeadBase + FVector(0.0f, 0.0f, MissionArrowShaft);
    const auto Ring = [&](const FVector &Center, float Radius) {
      const int32 First = Vertices.Num();
      for (int32 Index = 0; Index < Segments; ++Index)
      {
        const float Angle = 2.0f * PI * Index / Segments;
        Vertices.Add(Center + FVector(FMath::Cos(Angle) * Radius, FMath::Sin(Angle) * Radius, 0.0f));
      }
      return First;
    };
    const auto Fan = [&](int32 Apex, int32 First) {
      for (int32 Index = 0; Index < Segments; ++Index)
      {
        Triangles.Append({Apex, First + Index, First + (Index + 1) % Segments});
      }
    };
    // The head: its sides, and the flat base around the shaft.
    const int32 TipIndex = Vertices.Add(Tip);
    const int32 HeadRing = Ring(HeadBase, MissionArrowHeadRadius);
    Fan(TipIndex, HeadRing);
    Fan(Vertices.Add(HeadBase), HeadRing);
    // The shaft: its sides and its top.
    const int32 Bottom = Ring(HeadBase, MissionArrowShaftRadius);
    const int32 Top = Ring(ShaftTop, MissionArrowShaftRadius);
    for (int32 Index = 0; Index < Segments; ++Index)
    {
      const int32 Next = (Index + 1) % Segments;
      Triangles.Append({Bottom + Index, Top + Index, Bottom + Next, Bottom + Next, Top + Index, Top + Next});
    }
    Fan(Vertices.Add(ShaftTop), Top);
  }

  const carla::road::Map *GetMissionRoadMap(UCarlaEpisode &Episode)
  {
    ACarlaGameModeBase *GameMode = UCarlaStatics::GetGameMode(Episode.GetWorld());
    if (GameMode == nullptr || !GameMode->GetMap().has_value())
    {
      return nullptr;
    }
    return &*GameMode->GetMap();
  }
}

// =============================================================================
// -- Construction and files ---------------------------------------------------
// =============================================================================

FServerMission::FServerMission(UCarlaEpisode &InEpisode)
  : Episode(InEpisode)
{
  // Points off per violation (speeding: per started 5 mph beyond 5 over);
  // unsignalled lane changes are reported but not (yet) penalised.
  Penalties = {
    {TEXT("Collision with vehicle"), 50},
    {TEXT("Collision with pedestrian"), 100},
    {TEXT("Collision with object"), 25},
    {TEXT("Ran red light"), 50},
    {TEXT("Crossed solid line"), 20},
    {TEXT("Speeding"), 10},
    {TEXT("Lane change without signal"), 0},
  };

  MapName = Episode.GetMapName();
  FString Name;
  const FString Folder = FPaths::ProjectDir() / TEXT("Missions");
  if (FParse::Value(FCommandLine::Get(), TEXT("-server-mission="), Name) && !Name.IsEmpty())
  {
    if (!Name.Contains(TEXT("/")) && !Name.Contains(TEXT("\\")))
    {
      Path = Folder / (Name.EndsWith(TEXT(".json")) ? Name : Name + TEXT(".json"));
    }
    else
    {
      Path = FPaths::IsRelative(Name) ? FPaths::ProjectDir() / Name : Name;
    }
  }
  else
  {
    Path = Folder / MapName + TEXT(".json");
  }
  Path = FPaths::ConvertRelativePathToFull(Path);
  Load();
}

FServerMission::~FServerMission()
{
  if (RouteActor.IsValid())
  {
    RouteActor->Destroy();
  }
  if (PlannerCamera.IsValid())
  {
    PlannerCamera->Destroy();
  }
}

void FServerMission::Load()
{
  FString Text;
  if (!FFileHelper::LoadFileToString(Text, *Path))
  {
    UE_LOG(LogCarlaServerMission, Log, TEXT("Mission: no mission at %s yet; press M to plan one"), *Path);
    return;
  }
  TSharedPtr<FJsonObject> Root;
  const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Text);
  if (!FJsonSerializer::Deserialize(Reader, Root) || !Root.IsValid())
  {
    UE_LOG(LogCarlaServerMission, Warning, TEXT("Mission: %s is not valid JSON; starting an empty mission (saving will overwrite it)"), *Path);
    return;
  }
  double Number = 0.0;
  if (Root->TryGetNumberField(TEXT("radius_m"), Number))
  {
    Radius = FMath::Clamp(static_cast<float>(Number), 1.0f, 50.0f) * 100.0f;
  }
  if (Root->TryGetNumberField(TEXT("points_per_waypoint"), Number))
  {
    PointsPerWaypoint = FMath::Max(FMath::RoundToInt(Number), 0);
  }
  const TSharedPtr<FJsonObject> *PenaltyObject = nullptr;
  if (Root->TryGetObjectField(TEXT("penalties"), PenaltyObject) && PenaltyObject != nullptr)
  {
    for (const auto &Pair : (*PenaltyObject)->Values)
    {
      double Value = 0.0;
      if (!Pair.Value.IsValid() || !Pair.Value->TryGetNumber(Value))
      {
        continue;
      }
      const FString Key(*Pair.Key);
      TPair<FString, int32> *Existing = Penalties.FindByPredicate(
          [&Key](const TPair<FString, int32> &Penalty) { return Penalty.Key == Key; });
      if (Existing != nullptr)
      {
        Existing->Value = FMath::RoundToInt(Value);
      }
      else
      {
        Penalties.Emplace(Key, FMath::RoundToInt(Value));
      }
    }
  }
  const TArray<TSharedPtr<FJsonValue>> *List = nullptr;
  if (Root->TryGetArrayField(TEXT("waypoints"), List) && List != nullptr)
  {
    for (const TSharedPtr<FJsonValue> &Value : *List)
    {
      const TArray<TSharedPtr<FJsonValue>> *Xyz = nullptr;
      if (!Value.IsValid() || !Value->TryGetArray(Xyz) || Xyz == nullptr || Xyz->Num() < 2)
      {
        continue;
      }
      const double X = (*Xyz)[0]->AsNumber();
      const double Y = (*Xyz)[1]->AsNumber();
      const double Z = Xyz->Num() >= 3 ? (*Xyz)[2]->AsNumber() : 0.0;
      Waypoints.Add(FVector(X, Y, Z) * 100.0);
    }
  }
  FString FileMap;
  if (Root->TryGetStringField(TEXT("map"), FileMap) && !FileMap.IsEmpty() && FileMap != MapName)
  {
    UE_LOG(LogCarlaServerMission, Warning, TEXT("Mission: %s was planned on %s, this is %s"), *Path, *FileMap, *MapName);
  }
  UE_LOG(LogCarlaServerMission, Log, TEXT("Mission: %d waypoints from %s (radius %.1f m, %d points each)"),
      Waypoints.Num(), *Path, Radius / 100.0f, PointsPerWaypoint);
}

void FServerMission::Save() const
{
  FString Out = TEXT("{\n");
  Out += FString::Printf(TEXT("  \"map\": \"%s\",\n"), *MapName);
  Out += FString::Printf(TEXT("  \"radius_m\": %.1f,\n"), Radius / 100.0f);
  Out += FString::Printf(TEXT("  \"points_per_waypoint\": %d,\n"), PointsPerWaypoint);
  Out += TEXT("  \"penalties\": {\n");
  for (int32 Index = 0; Index < Penalties.Num(); ++Index)
  {
    Out += FString::Printf(TEXT("    \"%s\": %d%s\n"), *Penalties[Index].Key.ReplaceCharWithEscapedChar(),
        Penalties[Index].Value, Index + 1 < Penalties.Num() ? TEXT(",") : TEXT(""));
  }
  Out += TEXT("  },\n  \"waypoints\": [\n");
  for (int32 Index = 0; Index < Waypoints.Num(); ++Index)
  {
    const FVector Meters = Waypoints[Index] / 100.0;
    Out += FString::Printf(TEXT("    [%.2f, %.2f, %.2f]%s\n"), Meters.X, Meters.Y, Meters.Z,
        Index + 1 < Waypoints.Num() ? TEXT(",") : TEXT(""));
  }
  Out += TEXT("  ]\n}\n");
  IFileManager::Get().MakeDirectory(*FPaths::GetPath(Path), true);
  if (!FFileHelper::SaveStringToFile(Out, *Path, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM))
  {
    UE_LOG(LogCarlaServerMission, Warning, TEXT("Mission: cannot save %s"), *Path);
  }
}

// =============================================================================
// -- Roads --------------------------------------------------------------------
// =============================================================================

bool FServerMission::EnsureGraph()
{
  if (!Graph.IsEmpty())
  {
    return true;
  }
  if (bGraphTried)
  {
    return false;
  }
  bGraphTried = true;
  const carla::road::Map *Map = GetMissionRoadMap(Episode);
  if (Map == nullptr)
  {
    UE_LOG(LogCarlaServerMission, Warning, TEXT("Mission: the map has no OpenDRIVE road network; no routes"));
    return false;
  }
  Graph.Build(*Map);
  return !Graph.IsEmpty();
}

bool FServerMission::SnapToRoad(const FVector &Location, FVector &OutLocation) const
{
  const carla::road::Map *Map = GetMissionRoadMap(Episode);
  if (Map == nullptr)
  {
    OutLocation = Location;
    return true;
  }
  const auto Waypoint = Map->GetClosestWaypointOnRoad(carla::geom::Location(Location));
  if (!Waypoint.has_value())
  {
    return false;
  }
  FVector Road(Map->ComputeTransform(*Waypoint).location);
  if (FVector::Dist2D(Road, Location) > MissionSnapReach)
  {
    return false;
  }
  Road.Z = GroundHeight(Road);
  OutLocation = Road;
  return true;
}

/// The road surface under a lane point: OpenDRIVE's elevation can be off the
/// mesh by a few centimetres. Only static, upward-facing surfaces close to
/// it count, never a car or a pedestrian standing there.
float FServerMission::GroundHeight(const FVector &Location) const
{
  UWorld *World = Episode.GetWorld();
  if (World == nullptr)
  {
    return Location.Z;
  }
  TArray<FHitResult> Hits;
  const FCollisionObjectQueryParams Objects(ECC_WorldStatic);
  const FCollisionQueryParams Params(SCENE_QUERY_STAT(ServerMissionGround), false);
  World->LineTraceMultiByObjectType(Hits, Location + FVector(0.0f, 0.0f, MissionGroundReach),
      Location - FVector(0.0f, 0.0f, MissionGroundReach), Objects, Params);
  float Best = Location.Z;
  float BestOffset = MissionGroundReach;
  for (const FHitResult &Hit : Hits)
  {
    const UPrimitiveComponent *Component = Hit.GetComponent();
    if (Component == nullptr || Component->Mobility != EComponentMobility::Static || Hit.ImpactNormal.Z < 0.7f)
    {
      continue;
    }
    const float Offset = FMath::Abs(Hit.ImpactPoint.Z - Location.Z);
    if (Offset < BestOffset)
    {
      BestOffset = Offset;
      Best = Hit.ImpactPoint.Z;
    }
  }
  return Best;
}

float FServerMission::NodeGroundHeight(int32 Node)
{
  if (const float *Cached = NodeGround.Find(Node))
  {
    return *Cached;
  }
  return NodeGround.Add(Node, GroundHeight(Graph.GetLocation(Node)));
}

TArray<int32> FServerMission::FindCarStarts(const FVector &Location, const FVector &Forward) const
{
  TArray<int32> Starts = Graph.FindStarts(Location, Forward, MissionStartReach);
  if (Starts.Num() == 0)
  {
    const int32 Far = Graph.FindNearest(Location, FVector::ZeroVector, MissionStartFarReach);
    if (Far != INDEX_NONE)
    {
      Starts.Add(Far);
    }
  }
  return Starts;
}

bool FServerMission::RouteFrom(const TArray<int32> &Starts, const FVector &TargetLocation, TArray<FVector> &OutPoints, int32 *OutEnd)
{
  OutPoints.Reset();
  TArray<int32> Nodes;
  if (Starts.Num() == 0 || !Graph.FindRoute(Starts, TargetLocation, Radius, Nodes))
  {
    return false;
  }
  for (int32 Node : Nodes)
  {
    FVector Point = Graph.GetLocation(Node);
    Point.Z = NodeGroundHeight(Node);
    OutPoints.Add(Point);
  }
  OutPoints.Add(TargetLocation);
  if (OutEnd != nullptr)
  {
    *OutEnd = Nodes.Last();
  }
  return true;
}

// =============================================================================
// -- Driving ------------------------------------------------------------------
// =============================================================================

void FServerMission::Restart()
{
  Target = 0;
  Score = 0;
  LegPenalty = 0;
  LegViolations.Reset();
  bStarted = false;
  bComplete = false;
  bNoRoute = false;
  NextRouteTime = 0.0;
}

void FServerMission::Tick(APlayerController &Controller, ACarlaWheeledVehicle &Car, FServerDriveReport &Report, float DeltaSeconds)
{
  if (&Report != LastReport)
  {
    // A new drive (the car was respawned): count its violations from now.
    LastReport = &Report;
    ReportCursor = Report.GetEventCount();
  }
  if (bPlanning)
  {
    ReportCursor = Report.GetEventCount(); // no penalties while planning
    TickPlanner(Controller, DeltaSeconds);
  }
  else
  {
    TickDriving(Car, Report);
  }
}

void FServerMission::TickDriving(ACarlaWheeledVehicle &Car, FServerDriveReport &Report)
{
  if (Waypoints.Num() == 0 || bComplete)
  {
    HideRoute();
    return;
  }
  if (!bStarted)
  {
    bStarted = true;
    ReportCursor = Report.GetEventCount();
    Report.AddInfo(TEXT("Mission started"), FString::Printf(TEXT("%s, %d waypoints"),
        *FPaths::GetBaseFilename(Path), Waypoints.Num()), nullptr);
    UpdateSummary(Report);
    Popup(FString::Printf(TEXT("Mission: %d waypoints"), Waypoints.Num()), FColor::Green, 3.0);
  }

  // This leg's penalties.
  for (const FServerDriveReport::FViolation &Violation : Report.GetViolationDetailsSince(ReportCursor))
  {
    const FString &Name = Violation.Name;
    const TPair<FString, int32> *Penalty = Penalties.FindByPredicate(
        [&Name](const TPair<FString, int32> &Entry) { return Entry.Key == Name; });
    if (Penalty == nullptr || Penalty->Value <= 0)
    {
      continue;
    }
    int32 Points = Penalty->Value;
    FString Label = Name;
    if (Name == TEXT("Speeding") && Violation.SpeedKmh > 0.0f && Violation.SpeedLimitKmh > 0.0f)
    {
      // Speeding is free up to MissionSpeedingGraceMph over the limit, then
      // costs its penalty for each started 5 mph beyond that, at the
      // stretch's peak: 5 mph over costs nothing, 8 once, 12 twice.
      const float OverMph = (Violation.SpeedKmh - Violation.SpeedLimitKmh) / FServerDriveReport::KmhPerMph;
      const float ChargedMph = OverMph - MissionSpeedingGraceMph;
      if (ChargedMph <= 0.0f)
      {
        continue;
      }
      const int32 Steps = FMath::Max(1, FMath::CeilToInt(ChargedMph / MissionSpeedingStepMph));
      Points = Penalty->Value * Steps;
      Label = FString::Printf(TEXT("Speeding, %.0f mph over"), FMath::Max(OverMph, 1.0f));
    }
    LegPenalty += Points;
    LegViolations.Add(FString::Printf(TEXT("%s -%d"), *Label.ToLower(), Points));
    Popup(FString::Printf(TEXT("-%d  %s"), Points, *Label), FColor(255, 80, 60), 2.5);
  }
  ReportCursor = Report.GetEventCount();

  const FVector Location = Car.GetActorLocation();
  const FVector &Goal = Waypoints[Target];
  bool bArrived = FVector::Dist2D(Location, Goal) <= Radius && FMath::Abs(Location.Z - Goal.Z) <= MissionReachHeight;
  if (bArrived && EnsureGraph())
  {
    // On the goal's side of the street, as the route leads: not passing it
    // the other way.
    const int32 GoalLane = Graph.FindGoalLane(Goal, Radius);
    if (GoalLane != INDEX_NONE)
    {
      const FVector Heading = Car.GetActorForwardVector();
      const FVector &Lane = Graph.GetDirection(GoalLane);
      bArrived = Heading.X * Lane.X + Heading.Y * Lane.Y > 0.0;
    }
  }
  if (bArrived)
  {
    ReachWaypoint(Report);
    if (bComplete)
    {
      return;
    }
  }

  const double Now = FPlatformTime::Seconds();
  if (Now < NextRouteTime)
  {
    return;
  }
  NextRouteTime = Now + MissionRouteInterval;
  const FVector &Next = Waypoints[Target];
  TArray<FVector> Points;
  bNoRoute = true;
  if (EnsureGraph())
  {
    const FVector Forward = Car.GetActorForwardVector();
    bNoRoute = !RouteFrom(FindCarStarts(Location, Forward), Next, Points, nullptr);
    // Not under or behind the car.
    int32 Trim = 0;
    while (Trim < 5 && Points.Num() - Trim > 2 &&
        FVector::DotProduct(Points[Trim] - Location, FVector(Forward.X, Forward.Y, 0.0f)) < 150.0f)
    {
      ++Trim;
    }
    Points.RemoveAt(0, Trim);
  }
  RouteLength = 0.0f;
  for (int32 Index = 1; Index < Points.Num(); ++Index)
  {
    RouteLength += FVector::Dist(Points[Index - 1], Points[Index]);
  }
  if (bNoRoute)
  {
    RouteLength = FVector::Dist2D(Location, Next);
  }
  TArray<TArray<FVector>> Lines;
  if (Points.Num() >= 2)
  {
    Lines.Add(MoveTemp(Points));
  }
  ShowRoute(Lines, MissionLineWidth, &Next, true);
}

void FServerMission::ReachWaypoint(FServerDriveReport &Report)
{
  const int32 Points = FMath::Max(PointsPerWaypoint - LegPenalty, 0);
  Score += Points;
  FString Details = FString::Printf(TEXT("waypoint %d of %d: +%d"), Target + 1, Waypoints.Num(), Points);
  if (LegViolations.Num() > 0)
  {
    Details += FString::Printf(TEXT(" (%d, %s)"), PointsPerWaypoint, *FString::Join(LegViolations, TEXT(", ")));
  }
  Report.AddInfo(TEXT("Waypoint reached"), Details, nullptr);
  UE_LOG(LogCarlaServerMission, Log, TEXT("Mission: %s"), *Details);
  ++Target;
  LegPenalty = 0;
  LegViolations.Reset();
  NextRouteTime = 0.0;
  const int32 Best = PointsPerWaypoint * Waypoints.Num();
  if (Target >= Waypoints.Num())
  {
    bComplete = true;
    Report.AddInfo(TEXT("Mission complete"), FString::Printf(TEXT("score %d of %d"), Score, Best), nullptr);
    UE_LOG(LogCarlaServerMission, Log, TEXT("Mission: complete, score %d of %d"), Score, Best);
    Popup(FString::Printf(TEXT("Mission complete!   Score %d / %d"), Score, Best), FColor::Green, 8.0);
    HideRoute();
  }
  else
  {
    Popup(FString::Printf(TEXT("+%d   Waypoint %d of %d"), Points, Target, Waypoints.Num()), FColor::Green, 3.0);
  }
  UpdateSummary(Report);
}

void FServerMission::UpdateSummary(FServerDriveReport &Report) const
{
  Report.SetSummaryValue(TEXT("Mission"), FPaths::GetBaseFilename(Path));
  Report.SetSummaryValue(TEXT("Mission waypoints reached"), FString::Printf(TEXT("%d of %d"), Target, Waypoints.Num()));
  Report.SetSummaryValue(TEXT("Mission score"), FString::Printf(TEXT("%d of %d"), Score, PointsPerWaypoint * Waypoints.Num()));
}

// =============================================================================
// -- Planning -----------------------------------------------------------------
// =============================================================================

bool FServerMission::Pressed(const APlayerController &Controller, const FKey &Key)
{
  bool &bWasDown = KeysDown.FindOrAdd(Key.GetFName());
  const bool bDown = Controller.IsInputKeyDown(Key);
  const bool bEdge = bDown && !bWasDown;
  bWasDown = bDown;
  return bEdge;
}

void FServerMission::SetPlanning(bool bPlan, APlayerController &Controller, const ACarlaWheeledVehicle &Car)
{
  if (bPlan == bPlanning)
  {
    return;
  }
  bPlanning = bPlan;
  KeysDown.Reset();
  if (bPlanning)
  {
    CarLocation = Car.GetActorLocation();
    CarForward = Car.GetActorForwardVector();
    Focus = CarLocation;
    if (!PlannerCamera.IsValid() && Episode.GetWorld() != nullptr)
    {
      FActorSpawnParameters Parameters;
      Parameters.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
      ACameraActor *Camera = Episode.GetWorld()->SpawnActor<ACameraActor>(Parameters);
      if (Camera != nullptr)
      {
        UCameraComponent *Lens = Camera->GetCameraComponent();
        Lens->SetFieldOfView(60.0f);
        Lens->SetConstraintAspectRatio(false);
        Lens->bOverrideAspectRatioAxisConstraint = true;
        Lens->SetAspectRatioAxisConstraint(EAspectRatioAxisConstraint::AspectRatio_MaintainXFOV);
        Lens->PostProcessSettings.bOverride_MotionBlurAmount = true;
        Lens->PostProcessSettings.MotionBlurAmount = 0.0f;
        Lens->PostProcessBlendWeight = 1.0f;
      }
      PlannerCamera = Camera;
    }
    UpdatePlannerCamera();
    Controller.bAutoManageActiveCameraTarget = false;
    if (PlannerCamera.IsValid())
    {
      Controller.SetViewTarget(PlannerCamera.Get());
    }
    Controller.SetShowMouseCursor(true);
    FInputModeGameAndUI Mode;
    Mode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock);
    Mode.SetHideCursorDuringCapture(false);
    Controller.SetInputMode(Mode);
    Selected = INDEX_NONE;
    bDragging = false;
    bPlanDirty = true;
    PlanWidth = 0.0f;
    EnsureGraph();
    UE_LOG(LogCarlaServerMission, Log, TEXT("Mission: planning %s"), *Path);
  }
  else
  {
    Controller.SetShowMouseCursor(false);
    Controller.SetInputMode(FInputModeGameOnly());
    HideRoute();
    Restart();
    UE_LOG(LogCarlaServerMission, Log, TEXT("Mission: %d waypoints; driving"), Waypoints.Num());
  }
}

void FServerMission::UpdatePlannerCamera()
{
  if (PlannerCamera.IsValid())
  {
    PlannerCamera->SetActorLocationAndRotation(Focus + FVector(0.0f, 0.0f, Height), FRotator(-89.9f, 0.0f, 0.0f));
  }
}

bool FServerMission::MouseOnRoad(APlayerController &Controller, FVector &OutLocation) const
{
  float MouseX = 0.0f;
  float MouseY = 0.0f;
  FVector Origin;
  FVector Direction;
  if (!Controller.GetMousePosition(MouseX, MouseY) ||
      !Controller.DeprojectScreenPositionToWorld(MouseX, MouseY, Origin, Direction))
  {
    return false;
  }
  FVector Point;
  FHitResult Hit;
  const FCollisionQueryParams Params(SCENE_QUERY_STAT(ServerMissionPick), true);
  UWorld *World = Episode.GetWorld();
  if (World != nullptr && World->LineTraceSingleByChannel(Hit, Origin, Origin + Direction * 1.0e7f, ECC_Visibility, Params))
  {
    Point = Hit.ImpactPoint;
  }
  else if (FMath::Abs(Direction.Z) > 1.0e-3f)
  {
    Point = Origin + Direction * ((Focus.Z - Origin.Z) / Direction.Z);
  }
  else
  {
    return false;
  }
  return SnapToRoad(Point, OutLocation);
}

int32 FServerMission::PickWaypoint(APlayerController &Controller) const
{
  float MouseX = 0.0f;
  float MouseY = 0.0f;
  if (!Controller.GetMousePosition(MouseX, MouseY))
  {
    return INDEX_NONE;
  }
  int32 Best = INDEX_NONE;
  float BestDistance = MissionPickPixels;
  for (int32 Index = 0; Index < Waypoints.Num(); ++Index)
  {
    FVector2D Screen;
    if (!Controller.ProjectWorldLocationToScreen(Waypoints[Index], Screen, false))
    {
      continue;
    }
    const float Distance = FVector2D::Distance(Screen, FVector2D(MouseX, MouseY));
    if (Distance <= BestDistance)
    {
      BestDistance = Distance;
      Best = Index;
    }
  }
  return Best;
}

void FServerMission::TickPlanner(APlayerController &Controller, float DeltaSeconds)
{
  const double Now = FPlatformTime::Seconds();
  const auto Down = [&Controller](const FKey &A, const FKey &B) {
    return Controller.IsInputKeyDown(A) || Controller.IsInputKeyDown(B);
  };

  // Pan and zoom.
  const bool bFast = Down(EKeys::LeftShift, EKeys::RightShift);
  const float Speed = Height * MissionPanSpeed * (bFast ? 3.0f : 1.0f) * FMath::Min(DeltaSeconds, 0.1f);
  Focus.X += ((Down(EKeys::W, EKeys::Up) ? 1.0f : 0.0f) - (Down(EKeys::S, EKeys::Down) ? 1.0f : 0.0f)) * Speed;
  Focus.Y += ((Down(EKeys::D, EKeys::Right) ? 1.0f : 0.0f) - (Down(EKeys::A, EKeys::Left) ? 1.0f : 0.0f)) * Speed;
  const bool bZoomIn = Controller.WasInputKeyJustPressed(EKeys::MouseScrollUp) |
      Pressed(Controller, EKeys::Equals) | Pressed(Controller, EKeys::Add);
  const bool bZoomOut = Controller.WasInputKeyJustPressed(EKeys::MouseScrollDown) |
      Pressed(Controller, EKeys::Hyphen) | Pressed(Controller, EKeys::Subtract);
  if (bZoomIn)
  {
    Height *= 0.8f;
  }
  if (bZoomOut)
  {
    Height /= 0.8f;
  }
  Height = FMath::Clamp(Height, MissionMinHeight, MissionMaxHeight);
  UpdatePlannerCamera();

  // Add, move, delete.
  if (Pressed(Controller, EKeys::LeftMouseButton))
  {
    const int32 Pick = PickWaypoint(Controller);
    FVector Location;
    if (Pick != INDEX_NONE)
    {
      Selected = Pick;
      bDragging = true;
      bDragMoved = false;
    }
    else if (MouseOnRoad(Controller, Location))
    {
      Waypoints.Add(Location);
      Selected = Waypoints.Num() - 1;
      bPlanDirty = true;
      Save();
    }
    else
    {
      Popup(TEXT("Click on a road"), FColor::Orange, 1.5);
    }
  }
  else if (bDragging)
  {
    FVector Location;
    if (!Controller.IsInputKeyDown(EKeys::LeftMouseButton))
    {
      bDragging = false;
      if (bDragMoved)
      {
        Save();
      }
    }
    else if (Waypoints.IsValidIndex(Selected) && MouseOnRoad(Controller, Location) &&
        FVector::Dist(Location, Waypoints[Selected]) > 20.0f)
    {
      Waypoints[Selected] = Location;
      bDragMoved = true;
      bPlanDirty = true;
    }
  }
  const auto Remove = [this](int32 Index) {
    Waypoints.RemoveAt(Index);
    Selected = Waypoints.Num() == 0 ? INDEX_NONE : FMath::Min(Index, Waypoints.Num() - 1);
    bDragging = false;
    bPlanDirty = true;
    Save();
  };
  if (Pressed(Controller, EKeys::RightMouseButton))
  {
    const int32 Pick = PickWaypoint(Controller);
    if (Pick != INDEX_NONE)
    {
      Remove(Pick);
      Selected = INDEX_NONE;
    }
  }
  if (Pressed(Controller, EKeys::BackSpace) && Waypoints.Num() > 0)
  {
    Remove(Waypoints.IsValidIndex(Selected) ? Selected : Waypoints.Num() - 1);
  }
  if (Pressed(Controller, EKeys::Comma) && Waypoints.IsValidIndex(Selected) && Selected > 0)
  {
    Waypoints.Swap(Selected, Selected - 1);
    --Selected;
    bPlanDirty = true;
    Save();
  }
  if (Pressed(Controller, EKeys::Period) && Waypoints.IsValidIndex(Selected) && Selected + 1 < Waypoints.Num())
  {
    Waypoints.Swap(Selected, Selected + 1);
    ++Selected;
    bPlanDirty = true;
    Save();
  }
  if (Pressed(Controller, EKeys::Delete))
  {
    if (Now - ClearPromptTime <= MissionClearConfirmSeconds)
    {
      Waypoints.Reset();
      Selected = INDEX_NONE;
      bPlanDirty = true;
      ClearPromptTime = -1000.0;
      Save();
      Popup(TEXT("Mission cleared"), FColor::Orange, 2.0);
    }
    else
    {
      ClearPromptTime = Now;
      Popup(TEXT("Press Delete again to clear every waypoint"), FColor::Orange, MissionClearConfirmSeconds);
    }
  }

  // The legs, as wide as a few pixels from up here.
  const float Width = FMath::Max(MissionLineWidth, Height * 0.005f);
  if (PlanWidth <= 0.0f || FMath::Abs(Width - PlanWidth) > PlanWidth * 0.2f)
  {
    PlanWidth = Width;
    bPlanDirty = true;
  }
  if (bPlanDirty && Now >= NextPlanTime)
  {
    UpdatePlanLegs();
    ShowRoute(PlanLegs, PlanWidth, nullptr, false);
    bPlanDirty = false;
    NextPlanTime = Now + MissionPlanInterval;
  }
}

/// The routes from the car to the first waypoint and on from each to the
/// next, each leg starting where the one before reached its waypoint.
void FServerMission::UpdatePlanLegs()
{
  PlanLegs.Reset();
  PlanLegOk.Reset();
  const bool bGraph = EnsureGraph();
  TArray<int32> Starts;
  if (bGraph)
  {
    Starts = FindCarStarts(CarLocation, CarForward);
  }
  for (const FVector &Waypoint : Waypoints)
  {
    TArray<FVector> Points;
    int32 End = INDEX_NONE;
    const bool bOk = bGraph && RouteFrom(Starts, Waypoint, Points, &End);
    PlanLegOk.Add(bOk);
    PlanLegs.Add(MoveTemp(Points));
    // The next leg leaves from where this one arrived (or, without a route,
    // from the waypoint's own lane).
    Starts.Reset();
    const int32 From = bOk ? End : (bGraph ? Graph.FindGoalLane(Waypoint, Radius) : INDEX_NONE);
    if (From != INDEX_NONE)
    {
      Starts.Add(From);
    }
  }
}

// =============================================================================
// -- Drawing ------------------------------------------------------------------
// =============================================================================

bool FServerMission::EnsureMesh()
{
  if (RouteMesh.IsValid())
  {
    return true;
  }
  UWorld *World = Episode.GetWorld();
  if (World == nullptr)
  {
    return false;
  }
  UMaterialInterface *Base = LoadObject<UMaterialInterface>(nullptr, MissionMaterialPath);
  if (Base == nullptr)
  {
    static bool bWarned = false;
    if (!bWarned)
    {
      bWarned = true;
      UE_LOG(LogCarlaServerMission, Warning, TEXT("Mission: cannot load %s (Util/Tools/CreateServerRouteMaterial.py makes it); no route line"),
          MissionMaterialPath);
    }
    return false;
  }
  FActorSpawnParameters Parameters;
  Parameters.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
  AActor *Actor = World->SpawnActor<AActor>(AActor::StaticClass(), FTransform::Identity, Parameters);
  if (Actor == nullptr)
  {
    return false;
  }
  UProceduralMeshComponent *Mesh = NewObject<UProceduralMeshComponent>(Actor, TEXT("ServerMissionRoute"));
  Mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
  Mesh->SetCastShadow(false);
  Mesh->bUseAsyncCooking = false;
  Actor->SetRootComponent(Mesh);
  Mesh->RegisterComponent();
  Actor->AddInstanceComponent(Mesh);
  Mesh->SetWorldTransform(FTransform::Identity);

  UMaterialInstanceDynamic *Line = UMaterialInstanceDynamic::Create(Base, Mesh);
  Line->SetScalarParameterValue(TEXT("Opacity"), 0.75f);
  UMaterialInstanceDynamic *Arrow = UMaterialInstanceDynamic::Create(Base, Mesh);
  Arrow->SetScalarParameterValue(TEXT("Opacity"), 0.92f);
  Arrow->SetScalarParameterValue(TEXT("Brightness"), 1.0f);
  // Assigned now, so the component holds them from the start: a material
  // only held weakly here would be garbage-collected before its section is
  // first drawn (the arrow's, when the planner opens first).
  Mesh->SetMaterial(MissionLines, Line);
  Mesh->SetMaterial(MissionRing, Line);
  Mesh->SetMaterial(MissionArrow, Arrow);

  RouteActor = Actor;
  RouteMesh = Mesh;
  LineMaterial = Line;
  ArrowMaterial = Arrow;
  return true;
}

void FServerMission::ShowRoute(const TArray<TArray<FVector>> &Lines, float Width, const FVector *Ring, bool bArrow)
{
  if (!EnsureMesh())
  {
    return;
  }
  UProceduralMeshComponent *Mesh = RouteMesh.Get();
  const auto Build = [&](int32 Section, const TArray<FVector> &Vertices, const TArray<int32> &Triangles, UMaterialInterface *Material) {
    if (Triangles.Num() == 0)
    {
      Mesh->ClearMeshSection(Section);
      return;
    }
    TArray<FVector> Normals;
    Normals.Init(FVector::UpVector, Vertices.Num());
    Mesh->CreateMeshSection_LinearColor(Section, Vertices, Triangles, Normals, TArray<FVector2D>(),
        TArray<FLinearColor>(), TArray<FProcMeshTangent>(), false);
    if (Material != nullptr)
    {
      Mesh->SetMaterial(Section, Material);
    }
  };

  TArray<FVector> Vertices;
  TArray<int32> Triangles;
  for (const TArray<FVector> &Line : Lines)
  {
    AppendMissionStrip(Line, Width, MissionLineLift, Vertices, Triangles);
  }
  Build(MissionLines, Vertices, Triangles, LineMaterial.Get());

  Vertices.Reset();
  Triangles.Reset();
  if (Ring != nullptr)
  {
    AppendMissionRing(*Ring + FVector(0.0f, 0.0f, MissionLineLift), Radius, MissionRingWidth, Vertices, Triangles);
  }
  Build(MissionRing, Vertices, Triangles, LineMaterial.Get());

  Vertices.Reset();
  Triangles.Reset();
  if (Ring != nullptr && bArrow)
  {
    AppendMissionArrow(*Ring, Vertices, Triangles);
  }
  Build(MissionArrow, Vertices, Triangles, ArrowMaterial.Get());
}

void FServerMission::HideRoute()
{
  if (RouteMesh.IsValid())
  {
    RouteMesh->ClearAllMeshSections();
  }
}

void FServerMission::Popup(const FString &Text, const FColor &Color, double Seconds)
{
  PopupText = Text;
  PopupColor = Color;
  PopupUntil = FPlatformTime::Seconds() + Seconds;
}

void FServerMission::Draw(UCanvas &Canvas, float ViewWidth, float Y)
{
  if (GEngine == nullptr)
  {
    return;
  }
  UFont *Font = GEngine->GetMediumFont();
  const float Scale = 1.3f;
  float TextWidth = 0.0f;
  float TextHeight = 0.0f;

  FString Line;
  FColor LineColor = FColor::White;
  if (bPlanning)
  {
    Line = FString::Printf(TEXT("MISSION PLANNER  %s, %d waypoints    drive [M]"),
        *FPaths::GetBaseFilename(Path), Waypoints.Num());
    LineColor = FColor::Green;
  }
  else if (Waypoints.Num() == 0)
  {
    Line = TEXT("mission: none    plan [M]");
  }
  else if (bComplete)
  {
    Line = FString::Printf(TEXT("mission complete: score %d / %d    restart [R]    plan [M]"),
        Score, PointsPerWaypoint * Waypoints.Num());
    LineColor = FColor::Green;
  }
  else
  {
    Line = FString::Printf(TEXT("mission: waypoint %d/%d    %.0f m%s    score %d%s    restart [R]    plan [M]"),
        Target + 1, Waypoints.Num(), RouteLength / 100.0f, bNoRoute ? TEXT(" (no legal route)") : TEXT(""), Score,
        LegPenalty > 0 ? *FString::Printf(TEXT(" (this waypoint -%d)"), LegPenalty) : TEXT(""));
    LineColor = FColor(120, 255, 140);
  }
  Canvas.SetDrawColor(LineColor);
  Canvas.DrawText(Font, Line, 24.0f, Y, Scale, Scale);
  Canvas.StrLen(Font, Line, TextWidth, TextHeight);

  if (bPlanning)
  {
    const FString Help = TEXT("click a road: add    drag: move    right-click: delete    , . : reorder selected    "
        "Backspace: delete selected    Delete twice: clear    wheel or + -: zoom    WASD: pan (Shift: fast)");
    Canvas.SetDrawColor(FColor::White);
    Canvas.DrawText(Font, Help, 24.0f, Y + TextHeight * Scale + 4.0f, 1.1f, 1.1f);

    // The car, and an arrow the way it faces.
    const FVector CarScreen = Canvas.Project(CarLocation);
    const FVector NoseScreen = Canvas.Project(CarLocation + CarForward * 800.0f);
    Canvas.K2_DrawLine(FVector2D(CarScreen), FVector2D(NoseScreen), 4.0f, FLinearColor::White);
    Canvas.SetDrawColor(FColor::White);
    Canvas.DrawText(Font, TEXT("car"), CarScreen.X + 8.0f, CarScreen.Y + 8.0f, 1.2f, 1.2f);

    FVector Previous = CarScreen;
    for (int32 Index = 0; Index < Waypoints.Num(); ++Index)
    {
      const FVector Center = Canvas.Project(Waypoints[Index]);
      const FVector Edge = Canvas.Project(Waypoints[Index] + FVector(Radius, 0.0f, 0.0f));
      const float Pixels = FMath::Max(FVector2D::Distance(FVector2D(Center), FVector2D(Edge)), 10.0f);
      const FLinearColor Color = Index == Selected ? FLinearColor::Yellow : FLinearColor(0.1f, 1.0f, 0.25f);
      constexpr int32 Segments = 32;
      for (int32 Segment = 0; Segment < Segments; ++Segment)
      {
        const float A = 2.0f * PI * Segment / Segments;
        const float B = 2.0f * PI * (Segment + 1) / Segments;
        Canvas.K2_DrawLine(FVector2D(Center.X + FMath::Cos(A) * Pixels, Center.Y + FMath::Sin(A) * Pixels),
            FVector2D(Center.X + FMath::Cos(B) * Pixels, Center.Y + FMath::Sin(B) * Pixels), 3.0f, Color);
      }
      if (PlanLegOk.IsValidIndex(Index) && !PlanLegOk[Index])
      {
        // No legal route: a red line where the leg should be.
        Canvas.K2_DrawLine(FVector2D(Previous), FVector2D(Center), 3.0f, FLinearColor::Red);
      }
      const FString Number = FString::FromInt(Index + 1);
      Canvas.StrLen(Font, Number, TextWidth, TextHeight);
      const float NumberScale = 1.8f;
      Canvas.SetDrawColor(Color.ToFColor(true));
      Canvas.DrawText(Font, Number, Center.X - TextWidth * NumberScale * 0.5f, Center.Y - TextHeight * NumberScale * 0.5f,
          NumberScale, NumberScale);
      Previous = Center;
    }
  }

  if (FPlatformTime::Seconds() < PopupUntil && !PopupText.IsEmpty())
  {
    const float PopupScale = 2.5f;
    Canvas.StrLen(Font, PopupText, TextWidth, TextHeight);
    Canvas.SetDrawColor(PopupColor);
    Canvas.DrawText(Font, PopupText, (ViewWidth - TextWidth * PopupScale) * 0.5f, Canvas.ClipY * 0.25f,
        PopupScale, PopupScale);
  }
}
