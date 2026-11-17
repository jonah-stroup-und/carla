// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Walker/ServerWalkers.h"
#include "Carla.h"

#include "Carla/Actor/ActorDefinition.h"
#include "Carla/Actor/ActorDescription.h"
#include "Carla/Actor/CarlaActor.h"
#include "Carla/Game/CarlaEpisode.h"
#include "Carla/Game/CarlaGameModeBase.h"
#include "Carla/Game/CarlaStatics.h"
#include "Carla/Recorder/CarlaReplayer.h"
#include "Carla/Traffic/TrafficLightBase.h"
#include "Carla/Util/NavigationMesh.h"

#include <util/disable-ue4-macros.h>
#include <carla/geom/BoundingBox.h>
#include <carla/geom/Location.h>
#include <carla/geom/Rotation.h>
#include <carla/geom/Transform.h>
#include <carla/road/Map.h>
#include <carla/rpc/TrafficLightState.h>
#include <carla/rpc/WalkerControl.h>
#include <carla/servercrowd/CrowdWorld.h>
#include <carla/servercrowd/ServerCrowd.h>
#include <util/enable-ue4-macros.h>

#include <util/ue-header-guard-begin.h>
#include "Components/BoxComponent.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include <util/ue-header-guard-end.h>

#include <random>
#include <set>
#include <utility>
#include <vector>

namespace csc = carla::servercrowd;

DEFINE_LOG_CATEGORY_STATIC(LogCarlaServerWalkers, Log, All);

// =============================================================================
// -- Settings -----------------------------------------------------------------
// =============================================================================

FServerWalkersSettings FServerWalkersSettings::FromCommandLine()
{
  FServerWalkersSettings Settings;
  const TCHAR *CommandLine = FCommandLine::Get();
  FParse::Value(CommandLine, TEXT("-server-walkers="), Settings.NumberOfWalkers);
  FParse::Value(CommandLine, TEXT("-server-walkers-seed="), Settings.Seed);
  FParse::Value(CommandLine, TEXT("-server-walkers-cross="), Settings.CrossFactor);
  FParse::Value(CommandLine, TEXT("-server-walkers-run="), Settings.RunFactor);
  Settings.NumberOfWalkers = FMath::Max(0, Settings.NumberOfWalkers);
  Settings.CrossFactor = FMath::Clamp(Settings.CrossFactor, 0.0f, 1.0f);
  Settings.RunFactor = FMath::Clamp(Settings.RunFactor, 0.0f, 1.0f);
  return Settings;
}

// =============================================================================
// -- FServerWalkers -----------------------------------------------------------
// =============================================================================

namespace
{
  /// How long a killed walker lies where it fell before it is removed.
  constexpr double DeadWalkerSeconds = 10.0;
  /// generate_traffic.py's walking speed when a blueprint has none.
  constexpr float DefaultWalkingSpeed = 1.4f;
  /// Above the navmesh point, so the capsule clears the ground at spawn.
  constexpr float SpawnLiftMeters = 1.0f;
}

class FServerWalkers::FImpl final : public csc::CrowdWorld
{
public:

  FImpl(UCarlaEpisode &InEpisode, const FServerWalkersSettings &InSettings)
    : Episode(InEpisode),
      Settings(InSettings),
      RandomEngine(static_cast<uint32>(InSettings.Seed)) {}

  void Tick(float DeltaSeconds)
  {
    if (bFailed)
    {
      return;
    }
    if (!bInitialized)
    {
      bInitialized = true;
      if (!Initialize())
      {
        bFailed = true;
        return;
      }
    }

    // A replay drives the actors itself.
    const CarlaReplayer *Replayer = Episode.GetReplayer();
    if (Replayer != nullptr && const_cast<CarlaReplayer *>(Replayer)->IsEnabled())
    {
      return;
    }

    std::vector<csc::ServerCrowd::WalkerState> States;
    std::vector<carla::rpc::ActorId> Killed;
    Crowd->Update(Episode.GetElapsedGameTime(), DeltaSeconds, CollectVehicles(), States, Killed);

    for (const csc::ServerCrowd::WalkerState &State : States)
    {
      FCarlaActor *View = Episode.FindCarlaActor(State.id);
      if (View == nullptr)
      {
        Crowd->RemoveWalker(State.id); // destroyed, by a client most likely
        continue;
      }
      const ECarlaServerResponse Response = View->SetWalkerState(
          State.transform,
          carla::rpc::WalkerControl(State.transform.GetForwardVector(), State.speed, false));
      if (Response == ECarlaServerResponse::WalkerDead)
      {
        Crowd->RemoveWalker(State.id);
      }
    }

    const double Now = Episode.GetElapsedGameTime();
    for (const carla::rpc::ActorId Id : Killed)
    {
      if (FCarlaActor *View = Episode.FindCarlaActor(Id))
      {
        View->SetActorDead();
        DeadWalkers.emplace_back(Id, Now);
      }
    }
    RemoveDeadWalkers(Now);
    TopUpWalkers(Now);
  }

  // ---------------------------------------------------------------------------
  // CrowdWorld
  // ---------------------------------------------------------------------------

  /// As client::TrafficLight::GetStopWaypoints: sample the trigger volume
  /// across the road every meter and keep one waypoint per lane it touches.
  std::vector<TrafficLightStop> GetTrafficLightStops() override
  {
    std::vector<TrafficLightStop> Result;
    if (RoadMap == nullptr)
    {
      return Result;
    }
    for (const auto &Pair : Episode.GetActorRegistry())
    {
      const FCarlaActor &View = *Pair.Value.Get();
      if (View.GetActorType() != FCarlaActor::ActorType::TrafficLight)
      {
        continue;
      }
      const ATrafficLightBase *Light = Cast<ATrafficLightBase>(View.GetActor());
      const UBoxComponent *Box = Light != nullptr ? Light->GetTriggerVolume() : nullptr;
      if (Box == nullptr)
      {
        continue;
      }
      const FVector Center = Box->GetComponentLocation();
      const FVector Across = Light->GetActorForwardVector();
      const float HalfWidth = 0.9f * Box->GetScaledBoxExtent().X;
      std::set<std::pair<carla::road::RoadId, carla::road::LaneId>> Lanes;
      for (float Offset = -HalfWidth; Offset < HalfWidth; Offset += 100.0f)
      {
        const auto Waypoint = RoadMap->GetClosestWaypointOnRoad(carla::geom::Location(Center + Across * Offset));
        if (!Waypoint.has_value() || !Lanes.emplace(Waypoint->road_id, Waypoint->lane_id).second)
        {
          continue;
        }
        Result.push_back(TrafficLightStop{View.GetActorId(), RoadMap->ComputeTransform(*Waypoint).location});
      }
    }
    return Result;
  }

  carla::rpc::TrafficLightState GetTrafficLightState(carla::rpc::ActorId Id) override
  {
    const FCarlaActor *View = Episode.FindCarlaActor(Id);
    const ATrafficLightBase *Light = View != nullptr ? Cast<ATrafficLightBase>(View->GetActor()) : nullptr;
    if (Light == nullptr)
    {
      return carla::rpc::TrafficLightState::Unknown;
    }
    // ETrafficLightState is Red, Yellow, Green, Off: the rpc enum's order.
    return static_cast<carla::rpc::TrafficLightState>(Light->GetTrafficLightState());
  }

private:

  // ---------------------------------------------------------------------------
  // Setup and spawning
  // ---------------------------------------------------------------------------

  bool Initialize()
  {
    ACarlaGameModeBase *GameMode = UCarlaStatics::GetGameMode(Episode.GetWorld());
    if (GameMode != nullptr && GameMode->GetMap().has_value())
    {
      RoadMap = &*GameMode->GetMap();
    }

    const TArray<uint8> Bytes = FNavigationMesh::Load(Episode.GetMapName());
    if (Bytes.Num() == 0)
    {
      UE_LOG(LogCarlaServerWalkers, Error,
          TEXT("Server walkers: %s has no Recast navmesh (Nav/%s.bin); disabled"),
          *Episode.GetMapName(), *Episode.GetMapName());
      return false;
    }
    Crowd = std::make_unique<csc::ServerCrowd>(*this, static_cast<unsigned int>(Settings.Seed));
    if (!Crowd->Load(std::vector<uint8_t>(Bytes.GetData(), Bytes.GetData() + Bytes.Num())))
    {
      UE_LOG(LogCarlaServerWalkers, Error, TEXT("Server walkers: the navmesh for %s did not load; disabled"),
          *Episode.GetMapName());
      return false;
    }
    Crowd->SetPedestriansCrossFactor(Settings.CrossFactor);

    for (const FActorDefinition &Definition : Episode.GetActorDefinitions())
    {
      if (Definition.Id.StartsWith(TEXT("walker.pedestrian.")))
      {
        Blueprints.Add(&Definition);
      }
    }
    if (Blueprints.Num() == 0)
    {
      UE_LOG(LogCarlaServerWalkers, Error, TEXT("Server walkers: no walker.pedestrian.* blueprint; disabled"));
      return false;
    }

    const int32 Spawned = SpawnWalkers(Settings.NumberOfWalkers, Settings.NumberOfWalkers * 4);
    UE_LOG(LogCarlaServerWalkers, Log, TEXT("Server walkers: %d/%d pedestrians spawned and walking on %s"),
        Spawned, Settings.NumberOfWalkers, *Episode.GetMapName());
    return true;
  }

  /// Tries up to @a Attempts random navmesh points; returns how many spawned.
  int32 SpawnWalkers(int32 Count, int32 Attempts)
  {
    int32 Spawned = 0;
    std::uniform_real_distribution<float> Unit(0.0f, 1.0f);
    for (int32 Attempt = 0; Attempt < Attempts && Spawned < Count; ++Attempt)
    {
      carla::geom::Location From;
      carla::geom::Location Target;
      if (!Crowd->GetRandomLocation(From) || !Crowd->GetRandomLocation(Target))
      {
        continue;
      }
      std::uniform_int_distribution<int32> PickBlueprint(0, Blueprints.Num() - 1);
      const FActorDefinition &Definition = *Blueprints[PickBlueprint(RandomEngine)];
      const bool bRun = Unit(RandomEngine) < Settings.RunFactor;
      float Speed = DefaultWalkingSpeed;
      FActorDescription Description = MakeDescription(Definition, bRun, Speed);

      // The navmesh point is on the ground, and a walker's capsule is centred
      // on its location: lift it clear, or the spawn fails the collision
      // check. The crowd puts it back on the ground on the next frame.
      const carla::geom::Location Lifted(From.x, From.y, From.z + SpawnLiftMeters);
      const carla::geom::Transform SpawnPoint(Lifted, carla::geom::Rotation(0.0f, 360.0f * Unit(RandomEngine), 0.0f));
      auto Result = Episode.SpawnActorWithInfo(SpawnPoint, std::move(Description));
      if (Result.Key != EActorSpawnResultStatus::Success || Result.Value == nullptr)
      {
        continue;
      }
      FCarlaActor &View = *Result.Value;
      // As WalkerAIController::start: the crowd places the walker each frame.
      View.SetActorSimulatePhysics(false);
      if (!Crowd->AddWalker(View.GetActorId(), From, Target, Speed))
      {
        Episode.DestroyActor(View.GetActorId());
        continue;
      }
      ++Spawned;
    }
    return Spawned;
  }

  /// As generate_traffic.py: a random variation of the blueprint, mortal,
  /// at the blueprint's walking (or running) speed.
  FActorDescription MakeDescription(const FActorDefinition &Definition, bool bRun, float &OutSpeed)
  {
    FActorDescription Description;
    Description.UId = Definition.UId;
    Description.Id = Definition.Id;
    Description.Class = Definition.Class;
    for (const FActorVariation &Variation : Definition.Variations)
    {
      if (Variation.RecommendedValues.Num() == 0)
      {
        continue;
      }
      FActorAttribute Attribute;
      Attribute.Id = Variation.Id;
      Attribute.Type = Variation.Type;
      if (Variation.Id == TEXT("speed"))
      {
        // recommended_values: [stopped, walking, running]
        const int32 Index = FMath::Min(bRun ? 2 : 1, Variation.RecommendedValues.Num() - 1);
        Attribute.Value = Variation.RecommendedValues[Index];
        OutSpeed = FCString::Atof(*Attribute.Value);
        if (OutSpeed <= 0.0f)
        {
          OutSpeed = DefaultWalkingSpeed;
        }
      }
      else
      {
        std::uniform_int_distribution<int32> Pick(0, Variation.RecommendedValues.Num() - 1);
        Attribute.Value = Variation.RecommendedValues[Pick(RandomEngine)];
      }
      Description.Variations.Add(Attribute.Id, Attribute);
    }
    for (const FActorAttribute &Attribute : Definition.Attributes)
    {
      Description.Variations.Add(Attribute.Id, Attribute);
    }
    SetAttribute(Description, TEXT("is_invincible"), EActorAttributeType::Bool, TEXT("false"));
    SetAttribute(Description, TEXT("role_name"), EActorAttributeType::String, TEXT("server_walker"));
    return Description;
  }

  static void SetAttribute(FActorDescription &Description, const TCHAR *Id, EActorAttributeType Type, const TCHAR *Value)
  {
    FActorAttribute Attribute;
    Attribute.Id = Id;
    Attribute.Type = Type;
    Attribute.Value = Value;
    Description.Variations.Add(Attribute.Id, Attribute);
  }

  // ---------------------------------------------------------------------------
  // Per frame
  // ---------------------------------------------------------------------------

  std::vector<csc::ServerCrowd::Vehicle> CollectVehicles() const
  {
    std::vector<csc::ServerCrowd::Vehicle> Vehicles;
    for (const auto &Pair : Episode.GetActorRegistry())
    {
      const FCarlaActor &View = *Pair.Value.Get();
      if (View.GetActorType() != FCarlaActor::ActorType::Vehicle ||
          !View.IsAlive() || View.IsDormant() || !IsValid(View.GetActor()))
      {
        continue;
      }
      const FActorInfo *Info = View.GetActorInfo();
      Vehicles.push_back(csc::ServerCrowd::Vehicle{
          View.GetActorId(),
          carla::geom::Transform(View.GetActorGlobalTransform()),
          Info != nullptr ? carla::geom::BoundingBox(Info->BoundingBox) : carla::geom::BoundingBox()});
    }
    return Vehicles;
  }

  void RemoveDeadWalkers(double Now)
  {
    for (auto It = DeadWalkers.begin(); It != DeadWalkers.end();)
    {
      if (Now - It->second < DeadWalkerSeconds)
      {
        ++It;
        continue;
      }
      if (Episode.FindCarlaActor(It->first) != nullptr)
      {
        Episode.DestroyActor(It->first);
      }
      It = DeadWalkers.erase(It);
    }
  }

  /// Keep the requested number walking: replace walkers that died or were
  /// destroyed, at most one per second.
  void TopUpWalkers(double Now)
  {
    if (static_cast<int32>(Crowd->GetNumberOfWalkers()) >= Settings.NumberOfWalkers || Now - LastTopUp < 1.0)
    {
      return;
    }
    LastTopUp = Now;
    SpawnWalkers(1, 4);
  }

  UCarlaEpisode &Episode;
  FServerWalkersSettings Settings;
  std::mt19937 RandomEngine;

  const carla::road::Map *RoadMap = nullptr;
  std::unique_ptr<csc::ServerCrowd> Crowd;
  TArray<const FActorDefinition *> Blueprints;
  std::vector<std::pair<carla::rpc::ActorId, double>> DeadWalkers;
  double LastTopUp = 0.0;
  bool bInitialized = false;
  bool bFailed = false;
};

FServerWalkers::FServerWalkers(UCarlaEpisode &Episode, const FServerWalkersSettings &Settings)
  : Impl(std::make_unique<FImpl>(Episode, Settings)) {}

FServerWalkers::~FServerWalkers() = default;

void FServerWalkers::Tick(float DeltaSeconds)
{
  Impl->Tick(DeltaSeconds);
}
