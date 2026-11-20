// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Game/ServerDriveReport.h"
#include "Carla.h"

#include "Carla/Actor/CarlaActor.h"
#include "Carla/Game/CarlaEpisode.h"
#include "Carla/Game/CarlaGameModeBase.h"
#include "Carla/Game/CarlaStatics.h"
#include "Carla/Traffic/TrafficLightBase.h"
#include "Carla/Vehicle/CarlaWheeledVehicle.h"
#include "Carla/Vehicle/WheeledVehicleAIController.h"

#include <util/disable-ue4-macros.h>
#include <carla/geom/Location.h>
#include <carla/road/Map.h>
#include <carla/road/element/LaneMarking.h>
#include <util/enable-ue4-macros.h>

#include <util/ue-header-guard-begin.h>
#include "Components/PrimitiveComponent.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformMisc.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"
#if PLATFORM_WINDOWS
#include "Windows/AllowWindowsPlatformTypes.h"
#include <ShlObj.h>
#include "Windows/HideWindowsPlatformTypes.h"
#endif
#include <util/ue-header-guard-end.h>

DEFINE_LOG_CATEGORY_STATIC(LogCarlaDriveReport, Log, All);

namespace
{
  /// Both files are rewritten this often (wall-clock seconds) while driving.
  constexpr double WriteIntervalSeconds = 5.0;
  /// A contact ends when the two actors have not touched for this long.
  constexpr double ContactGapSeconds = 1.0;
  /// Contacts with anything that is not a vehicle or a pedestrian are only
  /// reported above this peak impulse (N s), so scraping a curb is not a crash.
  constexpr float ObjectImpulseThreshold = 200.0f;
  /// Speeding starts this far above the limit, ends at the limit, and is only
  /// reported when it lasts this long.
  constexpr float SpeedingMarginKmh = 2.0f;
  constexpr double SpeedingMinSeconds = 1.0;
  /// The four corners of the car cross a line one after another: crossings of
  /// the same kind this close together are one event.
  constexpr double CrossingCooldownSeconds = 2.0;
  /// Running a red light: leaving the light's trigger box forwards, faster
  /// than this, towards a junction this far ahead.
  constexpr float RedLightMinSpeedKmh = 3.0f;
  constexpr float RedLightJunctionLookahead = 600.0f; // cm

  using LaneMarkingType = carla::road::element::LaneMarking::Type;

  const TCHAR *LaneMarkingName(LaneMarkingType Type)
  {
    switch (Type)
    {
      case LaneMarkingType::Broken:       return TEXT("Broken");
      case LaneMarkingType::Solid:        return TEXT("Solid");
      case LaneMarkingType::SolidSolid:   return TEXT("SolidSolid");
      case LaneMarkingType::SolidBroken:  return TEXT("SolidBroken");
      case LaneMarkingType::BrokenSolid:  return TEXT("BrokenSolid");
      case LaneMarkingType::BrokenBroken: return TEXT("BrokenBroken");
      case LaneMarkingType::BottsDots:    return TEXT("BottsDots");
      case LaneMarkingType::Grass:        return TEXT("Grass");
      case LaneMarkingType::Curb:         return TEXT("Curb");
      case LaneMarkingType::None:         return TEXT("None");
      default:                            return TEXT("Other");
    }
  }

  bool IsSolid(LaneMarkingType Type)
  {
    return Type == LaneMarkingType::Solid || Type == LaneMarkingType::SolidSolid ||
        Type == LaneMarkingType::SolidBroken || Type == LaneMarkingType::BrokenSolid;
  }

  bool IsBroken(LaneMarkingType Type)
  {
    return Type == LaneMarkingType::Broken || Type == LaneMarkingType::BrokenBroken ||
        Type == LaneMarkingType::BottsDots;
  }

  FString Capitalize(const FString &Word)
  {
    return Word.IsEmpty() ? Word : Word.Left(1).ToUpper() + Word.Mid(1);
  }

  /// "vehicle.mercedes.sprinter" and "vehicle.ue4.mercedes.sprinter" ->
  /// "Mercedes Sprinter", "walker.pedestrian.0015" -> "Pedestrian 0015":
  /// the last two parts, make and model.
  FString DescribeBlueprint(const FString &Id)
  {
    TArray<FString> Parts;
    Id.ParseIntoArray(Parts, TEXT("."));
    if (Parts.Num() >= 3 && (Parts[0] == TEXT("vehicle") || Parts[0] == TEXT("walker")))
    {
      FString Model = Parts.Last();
      Model.ReplaceInline(TEXT("_"), TEXT(" "));
      return Capitalize(Parts[Parts.Num() - 2]) + TEXT(" ") + Capitalize(Model);
    }
    return Id;
  }

  FString Csv(const FString &Field)
  {
    if (Field.Contains(TEXT(",")) || Field.Contains(TEXT("\"")) || Field.Contains(TEXT("\n")))
    {
      return TEXT("\"") + Field.Replace(TEXT("\""), TEXT("\"\"")) + TEXT("\"");
    }
    return Field;
  }

  FString Number(double Value, int32 Decimals)
  {
    switch (Decimals)
    {
      case 0:  return FString::Printf(TEXT("%.0f"), Value);
      case 1:  return FString::Printf(TEXT("%.1f"), Value);
      default: return FString::Printf(TEXT("%.2f"), Value);
    }
  }

  FString WallTime(const FDateTime &Time)
  {
    return Time.ToString(TEXT("%m/%d/%Y %H:%M:%S"));
  }

  const carla::road::Map *GetRoadMap(UCarlaEpisode &Episode)
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
// -- UServerDriveHitListener --------------------------------------------------
// =============================================================================

void UServerDriveHitListener::OnHit(AActor *, AActor *OtherActor, FVector NormalImpulse, const FHitResult &Hit)
{
  if (Report != nullptr)
  {
    Report->OnHit(OtherActor, NormalImpulse, Hit);
  }
}

// =============================================================================
// -- FServerDriveReport -------------------------------------------------------
// =============================================================================

FString FServerDriveReport::GetDirectory()
{
  FString Directory;
  if (FParse::Value(FCommandLine::Get(), TEXT("-server-reports="), Directory) && !Directory.IsEmpty())
  {
    return FPaths::ConvertRelativePathToFull(Directory);
  }
  FString Desktop;
#if PLATFORM_WINDOWS
  PWSTR Path = nullptr;
  if (SUCCEEDED(SHGetKnownFolderPath(FOLDERID_Desktop, 0, nullptr, &Path)))
  {
    Desktop = Path;
  }
  CoTaskMemFree(Path);
#endif
  if (Desktop.IsEmpty())
  {
    FString Home = FPlatformMisc::GetEnvironmentVariable(TEXT("HOME"));
    if (Home.IsEmpty())
    {
      Home = FPlatformMisc::GetEnvironmentVariable(TEXT("USERPROFILE"));
    }
    Desktop = FPaths::Combine(Home, TEXT("Desktop"));
  }
  return FPaths::Combine(Desktop, TEXT("Reports"));
}

FServerDriveReport::FServerDriveReport(UCarlaEpisode &InEpisode, ACarlaWheeledVehicle &InVehicle, const FString &BlueprintId)
  : Episode(InEpisode),
    Vehicle(&InVehicle),
    StartWall(FDateTime::Now())
{
  const FString Directory = GetDirectory();
  IFileManager::Get().MakeDirectory(*Directory, true);
  const FString Stem = FPaths::Combine(Directory, TEXT("drive_") + StartWall.ToString(TEXT("%Y-%m-%d_%H-%M-%S")));
  SummaryPath = Stem + TEXT("_summary.csv");
  EverythingPath = Stem + TEXT("_everything.csv");

  StartTime = GameTime();
  LastSample = StartTime;

  // The CARLA vehicle blueprints already raise hit events (the collision
  // sensor relies on them); make sure of it for this one.
  if (UPrimitiveComponent *Root = Cast<UPrimitiveComponent>(InVehicle.GetRootComponent()))
  {
    Root->SetNotifyRigidBodyCollision(true);
  }
  Listener.Reset(NewObject<UServerDriveHitListener>());
  Listener->Report = this;
  InVehicle.OnActorHit.AddDynamic(Listener.Get(), &UServerDriveHitListener::OnHit);

  FEvent Spawned = MakeEvent(TEXT("Vehicle spawned"), false);
  Spawned.Details = DescribeBlueprint(BlueprintId);
  Spawned.SpeedKmh = -1.0f;
  Spawned.SpeedLimitKmh = -1.0f;
  Add(Spawned);
  Write();
  UE_LOG(LogCarlaDriveReport, Log, TEXT("Drive report: writing %s and %s"), *SummaryPath, *EverythingPath);
}

FServerDriveReport::~FServerDriveReport()
{
  if (Vehicle.IsValid())
  {
    Vehicle->OnActorHit.RemoveDynamic(Listener.Get(), &UServerDriveHitListener::OnHit);
  }
  Listener->Report = nullptr;
  CloseContacts(true);
  TrackSpeeding(true);
  Write();
  UE_LOG(LogCarlaDriveReport, Log, TEXT("Drive report: %d events, %d violations, %.2f km -> %s"),
      Events.Num(), ViolationCount, DistanceMeters / 1000.0, *SummaryPath);
}

double FServerDriveReport::GameTime() const
{
  return Episode.GetElapsedGameTime();
}

float FServerDriveReport::SpeedKmh() const
{
  return Vehicle.IsValid() ? FMath::Abs(Vehicle->GetVehicleForwardSpeed()) * 0.036f : 0.0f;
}

float FServerDriveReport::SpeedLimitKmh() const
{
  if (!Vehicle.IsValid())
  {
    return -1.0f;
  }
  const AWheeledVehicleAIController *Controller = Cast<AWheeledVehicleAIController>(Vehicle->GetController());
  return Controller != nullptr ? Controller->GetSpeedLimit() : -1.0f;
}

FServerDriveReport::FEvent FServerDriveReport::MakeEvent(const TCHAR *Name, bool bViolation) const
{
  FEvent Event;
  Event.Wall = FDateTime::Now();
  Event.DriveTime = GameTime() - StartTime;
  Event.Name = Name;
  Event.bViolation = bViolation;
  Event.SpeedKmh = SpeedKmh();
  Event.SpeedLimitKmh = SpeedLimitKmh();
  if (Vehicle.IsValid())
  {
    Event.Location = Vehicle->GetActorLocation();
  }
  return Event;
}

void FServerDriveReport::Add(FEvent Event)
{
  if (Event.bViolation)
  {
    ++ViolationCount;
  }
  UE_LOG(LogCarlaDriveReport, Log, TEXT("Drive report: %.1f s %s %s %s"),
      Event.DriveTime, *Event.Name, *Event.Details, *Event.OtherActor);
  Events.Add(MoveTemp(Event));
  bDirty = true;
}

// -----------------------------------------------------------------------------
// Sampling
// -----------------------------------------------------------------------------

void FServerDriveReport::Tick()
{
  if (!Vehicle.IsValid())
  {
    return;
  }
  const double Now = GameTime();
  const double Delta = FMath::Max(Now - LastSample, 0.0);
  LastSample = Now;
  DistanceMeters += SpeedKmh() / 3.6 * Delta;

  CloseContacts(false);
  TrackSpeeding(false);
  TrackLaneMarkings();
  TrackTrafficLight();

  const double Wall = FPlatformTime::Seconds();
  if (Wall - LastWrite >= WriteIntervalSeconds)
  {
    bDirty = true; // the duration and distance change even without events
    Write();
  }
}

void FServerDriveReport::OnHit(AActor *OtherActor, const FVector &NormalImpulse, const FHitResult &Hit)
{
  if (OtherActor == nullptr || OtherActor == Vehicle.Get())
  {
    return;
  }
  const FCarlaActor *View = Episode.FindCarlaActor(OtherActor);
  const bool bVehicle = View != nullptr && View->GetActorType() == FCarlaActor::ActorType::Vehicle;
  const bool bWalker = View != nullptr && View->GetActorType() == FCarlaActor::ActorType::Walker;
  if (!bVehicle && !bWalker && FMath::Abs(Hit.ImpactNormal.Z) > 0.6f)
  {
    return; // the road, a kerb top or a ramp: driving, not a crash
  }

  const float Impulse = NormalImpulse.Size() * 0.01f; // kg cm/s -> N s
  FContact *Contact = Contacts.Find(OtherActor);
  if (Contact == nullptr)
  {
    const TCHAR *Name = bVehicle ? TEXT("Collision with vehicle")
        : (bWalker ? TEXT("Collision with pedestrian") : TEXT("Collision with object"));
    FContact NewContact;
    NewContact.Event = MakeEvent(Name, true);
    NewContact.Event.OtherActor = DescribeActor(*OtherActor);
    NewContact.Event.SpeedLimitKmh = -1.0f;
    Contact = &Contacts.Add(OtherActor, NewContact);
  }
  Contact->LastHit = GameTime();
  Contact->PeakImpulse = FMath::Max(Contact->PeakImpulse, Impulse);
  Contact->bReportable = Contact->bReportable || bVehicle || bWalker || Contact->PeakImpulse >= ObjectImpulseThreshold;
}

void FServerDriveReport::CloseContacts(bool bAll)
{
  const double Now = GameTime();
  for (auto It = Contacts.CreateIterator(); It; ++It)
  {
    FContact &Contact = It.Value();
    if (!bAll && It.Key().IsValid() && Now - Contact.LastHit < ContactGapSeconds)
    {
      continue;
    }
    if (Contact.bReportable)
    {
      FEvent Event = Contact.Event;
      const double Duration = FMath::Max(Contact.LastHit - (StartTime + Event.DriveTime), 0.0);
      Event.Details = FString::Printf(TEXT("peak impulse %.0f N s, contact %.1f s"), Contact.PeakImpulse, Duration);
      Add(MoveTemp(Event));
    }
    It.RemoveCurrent();
  }
}

void FServerDriveReport::TrackSpeeding(bool bFinish)
{
  const float Speed = SpeedKmh();
  const float Limit = SpeedLimitKmh();
  const double Now = GameTime();
  if (!bSpeeding)
  {
    if (!bFinish && Limit > 0.0f && Speed > Limit + SpeedingMarginKmh)
    {
      bSpeeding = true;
      SpeedingEvent = MakeEvent(TEXT("Speeding"), true);
      SpeedingStart = Now;
      SpeedingPeak = Speed;
    }
    return;
  }
  SpeedingPeak = FMath::Max(SpeedingPeak, Speed);
  if (!bFinish && Limit > 0.0f && Speed > SpeedingEvent.SpeedLimitKmh)
  {
    return; // still above the limit it started under
  }
  bSpeeding = false;
  const double Duration = Now - SpeedingStart;
  if (Duration >= SpeedingMinSeconds)
  {
    SpeedingEvent.SpeedKmh = SpeedingPeak;
    SpeedingEvent.Details = FString::Printf(TEXT("peak %.0f km/h for %.1f s"), SpeedingPeak, Duration);
    Add(SpeedingEvent);
  }
}

void FServerDriveReport::TrackLaneMarkings()
{
  const carla::road::Map *RoadMap = GetRoadMap(Episode);
  const FCarlaActor *View = Episode.FindCarlaActor(Vehicle.Get());
  const FActorInfo *Info = View != nullptr ? View->GetActorInfo() : nullptr;
  if (RoadMap == nullptr || Info == nullptr)
  {
    return;
  }
  // The four corners of the bounding box at road level, like CARLA's
  // lane invasion sensor.
  const FTransform Transform = Vehicle->GetActorTransform();
  const FVector Origin = Info->BoundingBox.Origin;
  const FVector Extent = Info->BoundingBox.Extent;
  FVector Next[4];
  const float Signs[4][2] = {{1, 1}, {-1, 1}, {1, -1}, {-1, -1}};
  for (int32 Index = 0; Index < 4; ++Index)
  {
    Next[Index] = Transform.TransformPosition(
        FVector(Origin.X + Signs[Index][0] * Extent.X, Origin.Y + Signs[Index][1] * Extent.Y, Origin.Z));
  }
  if (!bHaveCorners)
  {
    FMemory::Memcpy(Corners, Next, sizeof(Corners));
    bHaveCorners = true;
    return;
  }

  LaneMarkingType Solid = LaneMarkingType::None;
  LaneMarkingType Broken = LaneMarkingType::None;
  for (int32 Index = 0; Index < 4; ++Index)
  {
    if (FVector::DistSquared(Corners[Index], Next[Index]) < 1.0f)
    {
      continue;
    }
    for (const auto &Marking : RoadMap->CalculateCrossedLanes(
             carla::geom::Location(Corners[Index]), carla::geom::Location(Next[Index])))
    {
      if (IsSolid(Marking.type) && Solid == LaneMarkingType::None)
      {
        Solid = Marking.type;
      }
      else if (IsBroken(Marking.type) && Broken == LaneMarkingType::None)
      {
        Broken = Marking.type;
      }
    }
  }
  FMemory::Memcpy(Corners, Next, sizeof(Corners));

  const double Now = GameTime();
  if (Solid != LaneMarkingType::None && Now - LastSolidCrossing >= CrossingCooldownSeconds)
  {
    LastSolidCrossing = Now;
    LastBrokenCrossing = Now; // the same manoeuvre: do not count it twice
    FEvent Event = MakeEvent(TEXT("Crossed solid line"), true);
    Event.Details = FString(TEXT("crossed ")) + LaneMarkingName(Solid);
    Add(MoveTemp(Event));
  }
  else if (Broken != LaneMarkingType::None && Now - LastBrokenCrossing >= CrossingCooldownSeconds)
  {
    LastBrokenCrossing = Now;
    const FVehicleLightState Lights = Vehicle->GetVehicleLightState();
    if (Lights.LeftBlinker || Lights.RightBlinker)
    {
      LastSignalledLaneChange = Now;
    }
    else
    {
      FEvent Event = MakeEvent(TEXT("Lane change without signal"), true);
      Event.Details = FString(TEXT("crossed ")) + LaneMarkingName(Broken);
      Add(MoveTemp(Event));
    }
  }
}

void FServerDriveReport::TrackTrafficLight()
{
  const AWheeledVehicleAIController *Controller = Cast<AWheeledVehicleAIController>(Vehicle->GetController());
  ATrafficLightBase *Current = Controller != nullptr ? Controller->GetTrafficLight() : nullptr;
  ATrafficLightBase *Previous = TrafficLight.Get();
  TrafficLight = Current;
  if (Previous == nullptr || Previous == Current)
  {
    return;
  }
  // Left the light's trigger box (it sits on the approach, up to the stop
  // line). Driving out of it forwards into the junction on red runs the light.
  if (Previous->GetTrafficLightState() != ETrafficLightState::Red)
  {
    return;
  }
  const float ForwardSpeed = Vehicle->GetVehicleForwardSpeed() * 0.036f;
  if (ForwardSpeed < RedLightMinSpeedKmh)
  {
    return;
  }
  const carla::road::Map *RoadMap = GetRoadMap(Episode);
  if (RoadMap != nullptr)
  {
    const FVector Ahead = Vehicle->GetActorLocation() + Vehicle->GetActorForwardVector() * RedLightJunctionLookahead;
    const auto Waypoint = RoadMap->GetClosestWaypointOnRoad(carla::geom::Location(Ahead));
    const auto Here = RoadMap->GetClosestWaypointOnRoad(carla::geom::Location(Vehicle->GetActorLocation()));
    const bool bTowardsJunction = (Waypoint && RoadMap->IsJunction(Waypoint->road_id)) ||
        (Here && RoadMap->IsJunction(Here->road_id));
    if (!bTowardsJunction)
    {
      return; // changed lanes out of the box, or reversed out of it
    }
  }
  FEvent Event = MakeEvent(TEXT("Ran red light"), true);
  const FCarlaActor *View = Episode.FindCarlaActor(Previous);
  Event.Details = View != nullptr ? FString::Printf(TEXT("traffic light %u"), View->GetActorId()) : TEXT("traffic light");
  Add(MoveTemp(Event));
}

void FServerDriveReport::SetAidMode(const FString &Aid, const FString &Mode)
{
  FAidMode &Current = AidModes.FindOrAdd(Aid);
  if (Mode == Current.Mode)
  {
    return;
  }
  Current.bMixed = Current.bMixed || !Current.Mode.IsEmpty();
  Current.Mode = Mode;
  FEvent Event = MakeEvent(*Aid, false);
  Event.Details = Mode;
  Add(Event);
}

void FServerDriveReport::AddInfo(const TCHAR *Name, const FString &Details, const AActor *Other)
{
  FEvent Event = MakeEvent(Name, false);
  Event.Details = Details;
  if (Other != nullptr)
  {
    Event.OtherActor = DescribeActor(*Other);
  }
  Add(Event);
}

FString FServerDriveReport::DescribeActor(const AActor &Actor) const
{
  const FCarlaActor *View = Episode.FindCarlaActor(const_cast<AActor *>(&Actor));
  if (View != nullptr && View->GetActorInfo() != nullptr)
  {
    return DescribeBlueprint(View->GetActorInfo()->Description.Id);
  }
  // A map object: "BP_StreetLight_simple64" -> "StreetLight simple", its
  // name without the blueprint prefix and the instance number.
  FString Name = Actor.GetName();
  Name.RemoveFromStart(TEXT("BP_"));
  Name.RemoveFromStart(TEXT("SM_"));
  while (Name.Len() > 1 && (FChar::IsDigit(Name[Name.Len() - 1]) || Name[Name.Len() - 1] == TEXT('_')))
  {
    Name.LeftChopInline(1);
  }
  Name.ReplaceInline(TEXT("_"), TEXT(" "));
  return Name;
}

// -----------------------------------------------------------------------------
// Output
// -----------------------------------------------------------------------------

void FServerDriveReport::Write()
{
  LastWrite = FPlatformTime::Seconds();
  if (!bDirty)
  {
    return;
  }
  bDirty = false;

  // Events in drive order, the open speeding stretch and contacts included
  // as they stand, so a crash mid-drive still leaves them in the file.
  TArray<FEvent> Rows = Events;
  int32 Violations = ViolationCount;
  if (bSpeeding)
  {
    FEvent Open = SpeedingEvent;
    Open.SpeedKmh = SpeedingPeak;
    Open.Details = FString::Printf(TEXT("peak %.0f km/h for %.1f s (ongoing)"), SpeedingPeak, GameTime() - SpeedingStart);
    Rows.Add(Open);
    ++Violations;
  }
  for (const auto &Pair : Contacts)
  {
    if (Pair.Value.bReportable)
    {
      FEvent Open = Pair.Value.Event;
      Open.Details = FString::Printf(TEXT("peak impulse %.0f N s (ongoing)"), Pair.Value.PeakImpulse);
      Rows.Add(Open);
      ++Violations;
    }
  }
  Rows.StableSort([](const FEvent &A, const FEvent &B) { return A.DriveTime < B.DriveTime; });

  FString Everything = TEXT("time,drive_time_s,event,severity,details,other_actor,speed_kmh,speed_limit_kmh,x,y\n");
  TMap<FString, int32> Counts;
  for (const FEvent &Event : Rows)
  {
    const carla::geom::Location Location(Event.Location);
    Everything += FString::Join(TArray<FString>{
        WallTime(Event.Wall),
        Number(Event.DriveTime, 1),
        Csv(Event.Name),
        Event.bViolation ? TEXT("violation") : TEXT("info"),
        Csv(Event.Details),
        Csv(Event.OtherActor),
        Event.SpeedKmh >= 0.0f ? Number(Event.SpeedKmh, 0) : FString(),
        Event.SpeedLimitKmh > 0.0f ? Number(Event.SpeedLimitKmh, 0) : FString(),
        Number(Location.x, 1),
        Number(Location.y, 1)}, TEXT(","));
    Everything += TEXT("\n");
    Counts.FindOrAdd(Event.Name) += 1;
  }

  const FDateTime Now = FDateTime::Now();
  const double DurationSeconds = FMath::Max(GameTime() - StartTime, 0.0);
  const double AverageKmh = DurationSeconds > 0.0 ? DistanceMeters / DurationSeconds * 3.6 : 0.0;
  FString Summary = TEXT("metric,value\n");
  Summary += TEXT("Drive started,") + WallTime(StartWall) + TEXT("\n");
  Summary += TEXT("Drive ended,") + WallTime(Now) + TEXT("\n");
  Summary += TEXT("Duration (min),") + Number(DurationSeconds / 60.0, 1) + TEXT("\n");
  Summary += TEXT("Distance (km),") + Number(DistanceMeters / 1000.0, 2) + TEXT("\n");
  Summary += TEXT("Average speed (km/h),") + Number(AverageKmh, 0) + TEXT("\n");
  Summary += TEXT("Violations,") + FString::FromInt(Violations) + TEXT("\n");
  for (const auto &Pair : AidModes)
  {
    Summary += Csv(Pair.Key) + TEXT(",") + Csv(Pair.Value.bMixed ? FString(TEXT("mixed")) : Pair.Value.Mode) + TEXT("\n");
  }
  Counts.KeySort([](const FString &A, const FString &B) { return A < B; });
  for (const auto &Pair : Counts)
  {
    Summary += Csv(Pair.Key) + TEXT(",") + FString::FromInt(Pair.Value) + TEXT("\n");
  }

  const bool bOk = FFileHelper::SaveStringToFile(Everything, *EverythingPath, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM) &&
      FFileHelper::SaveStringToFile(Summary, *SummaryPath, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
  if (!bOk)
  {
    UE_LOG(LogCarlaDriveReport, Warning, TEXT("Drive report: cannot write %s (open in Excel?); retrying"), *SummaryPath);
    bDirty = true;
  }
}
