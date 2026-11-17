// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Traffic/ServerTraffic.h"
#include "Carla.h"

#include "Carla/Actor/ActorDefinition.h"
#include "Carla/Actor/ActorDescription.h"
#include "Carla/Actor/CarlaActor.h"
#include "Carla/Game/CarlaEngine.h"
#include "Carla/Game/CarlaEpisode.h"
#include "Carla/Game/CarlaGameModeBase.h"
#include "Carla/Game/CarlaStatics.h"
#include "Carla/Recorder/CarlaReplayer.h"
#include "Carla/Traffic/TrafficLightBase.h"
#include "Carla/Vehicle/CarlaWheeledVehicle.h"
#include "Carla/Vehicle/VehicleControl.h"
#include "Carla/Vehicle/VehicleInputPriority.h"
#include "Carla/Vehicle/VehicleLightState.h"
#include "Carla/Vehicle/VehiclePhysicsControl.h"
#include "Carla/Vehicle/WheeledVehicleAIController.h"
#include "Carla/Weather/Weather.h"

#include <util/disable-ue4-macros.h>
#include <carla/geom/BoundingBox.h>
#include <carla/geom/Transform.h>
#include <carla/rpc/VehicleControl.h>
#include <carla/rpc/VehicleLightState.h>
#include <carla/rpc/WeatherParameters.h>
// Unreal defines DELTA as a macro; the Traffic Manager's constants name a
// constant DELTA (carla/servertraffic/Constants.h).
#pragma push_macro("DELTA")
#undef DELTA
#include <carla/servertraffic/ServerTrafficManager.h>
#pragma pop_macro("DELTA")
#include <util/enable-ue4-macros.h>

#include <util/ue-header-guard-begin.h>
#include "HAL/FileManager.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"
#include <util/ue-header-guard-end.h>

#include <random>
#include <unordered_map>

namespace cst = carla::servertraffic;
namespace stw = carla::servertraffic::world;

DEFINE_LOG_CATEGORY_STATIC(LogCarlaServerTraffic, Log, All);

// =============================================================================
// -- Settings -----------------------------------------------------------------
// =============================================================================

FServerTrafficSettings FServerTrafficSettings::FromCommandLine()
{
  FServerTrafficSettings Settings;
  const TCHAR *CommandLine = FCommandLine::Get();
  FParse::Value(CommandLine, TEXT("-server-traffic="), Settings.NumberOfVehicles);
  FParse::Value(CommandLine, TEXT("-server-traffic-seed="), Settings.Seed);
  FParse::Value(CommandLine, TEXT("-server-traffic-filter="), Settings.Filter);
  Settings.bSafe = FParse::Param(CommandLine, TEXT("server-traffic-safe"));
  Settings.NumberOfVehicles = FMath::Max(0, Settings.NumberOfVehicles);
  return Settings;
}

// =============================================================================
// -- Actor adapter ------------------------------------------------------------
// =============================================================================

/// An actor as the copied stages see it, read straight from the registry.
/// Holds only the id: the FCarlaActor is looked up on each call, so a
/// destroyed actor reads as not alive rather than dangling.
class FServerTrafficActor final : public stw::Actor
{
public:

  FServerTrafficActor(UCarlaEpisode &InEpisode, const FCarlaActor &View)
    : Episode(InEpisode),
      Id(View.GetActorId())
  {
    const FActorInfo *Info = View.GetActorInfo();
    if (Info != nullptr)
    {
      TypeId = TCHAR_TO_UTF8(*Info->Description.Id);
      for (const auto &Pair : Info->Description.Variations)
      {
        Attributes.emplace_back(
            std::string(TCHAR_TO_UTF8(*Pair.Value.Id)),
            std::string(TCHAR_TO_UTF8(*Pair.Value.Value)));
      }
      BoundingBox = carla::geom::BoundingBox(Info->BoundingBox);
    }
  }

  carla::ActorId GetId() const override { return Id; }
  const std::string &GetTypeId() const override { return TypeId; }
  const std::vector<stw::ActorAttribute> &GetAttributes() const override { return Attributes; }

  bool IsAlive() const override
  {
    const FCarlaActor *View = Find();
    return View != nullptr && View->IsAlive() && !View->IsDormant() && IsValid(View->GetActor());
  }

  carla::geom::Transform GetTransform() const override
  {
    const FCarlaActor *View = Find();
    return View != nullptr ? carla::geom::Transform(View->GetActorGlobalTransform()) : carla::geom::Transform();
  }

  carla::geom::Vector3D GetVelocity() const override
  {
    const FCarlaActor *View = Find();
    if (View == nullptr)
    {
      return {};
    }
    const FVector Velocity = View->GetActorVelocity(); // cm/s
    return carla::geom::Vector3D(Velocity.X, Velocity.Y, Velocity.Z).ToMeters();
  }

  carla::geom::BoundingBox GetBoundingBox() const override { return BoundingBox; }

  float GetSpeedLimit() const override
  {
    const AWheeledVehicleAIController *Controller = GetController();
    return Controller != nullptr ? Controller->GetSpeedLimit() : 30.0f;
  }

  carla::rpc::TrafficLightState GetTrafficLightState() const override
  {
    const AWheeledVehicleAIController *Controller = GetController();
    return Controller != nullptr ?
        static_cast<carla::rpc::TrafficLightState>(Controller->GetTrafficLightState()) :
        carla::rpc::TrafficLightState::Green;
  }

  bool IsAtTrafficLight() const override
  {
    const AWheeledVehicleAIController *Controller = GetController();
    return Controller != nullptr && Controller->GetTrafficLight() != nullptr;
  }

  float GetMaxSteerAngle() const override
  {
    FCarlaActor *View = Find();
    FVehiclePhysicsControl PhysicsControl;
    if (View == nullptr || View->GetPhysicsControl(PhysicsControl) != ECarlaServerResponse::Success)
    {
      return 0.0f;
    }
    float MaxSteerAngle = 0.0f;
    for (const auto &Wheel : PhysicsControl.Wheels)
    {
      MaxSteerAngle = FMath::Max(MaxSteerAngle, Wheel.MaxSteerAngle);
    }
    return MaxSteerAngle;
  }

  void SetSimulatePhysics(bool bEnabled) override
  {
    if (FCarlaActor *View = Find())
    {
      View->SetActorSimulatePhysics(bEnabled);
    }
  }

  void SetTargetVelocity(const carla::geom::Vector3D &Velocity) override
  {
    if (FCarlaActor *View = Find())
    {
      View->SetActorTargetVelocity(Velocity.ToCentimeters().ToFVector());
    }
  }

  void Destroy() override
  {
    if (Find() != nullptr)
    {
      Episode.DestroyActor(Id);
    }
  }

private:

  FCarlaActor *Find() const
  {
    return Episode.FindCarlaActor(Id);
  }

  const AWheeledVehicleAIController *GetController() const
  {
    const FCarlaActor *View = Find();
    if (View == nullptr || View->GetActorType() != FCarlaActor::ActorType::Vehicle)
    {
      return nullptr;
    }
    const auto *Vehicle = Cast<ACarlaWheeledVehicle>(View->GetActor());
    return Vehicle != nullptr ? Cast<AWheeledVehicleAIController>(Vehicle->GetController()) : nullptr;
  }

  UCarlaEpisode &Episode;
  carla::ActorId Id;
  std::string TypeId;
  std::vector<stw::ActorAttribute> Attributes;
  carla::geom::BoundingBox BoundingBox;
};

// =============================================================================
// -- World adapter ------------------------------------------------------------
// =============================================================================

class FServerTrafficWorld final : public stw::World
{
public:

  FServerTrafficWorld(UCarlaEpisode &InEpisode, const carla::road::Map &RoadMap)
    : Episode(InEpisode),
      Map(std::make_shared<stw::Map>(RoadMap, std::string(TCHAR_TO_UTF8(*InEpisode.GetMapName())))) {}

  void SetFrameTime(float InDeltaSeconds) { DeltaSeconds = InDeltaSeconds; }

  stw::Timestamp GetTimestamp() const override
  {
    return stw::Timestamp(
        FCarlaEngine::GetFrameCounter(),
        Episode.GetElapsedGameTime(),
        DeltaSeconds,
        FPlatformTime::Seconds());
  }

  carla::SharedPtr<stw::ActorList> GetActors() const override
  {
    // The stages only care about vehicles and walkers.
    auto Result = std::make_shared<stw::ActorList>();
    for (const auto &Pair : Episode.GetActorRegistry())
    {
      const FCarlaActor &View = *Pair.Value.Get();
      const auto Type = View.GetActorType();
      if ((Type != FCarlaActor::ActorType::Vehicle && Type != FCarlaActor::ActorType::Walker) ||
          !View.IsAlive() || View.IsDormant() || !IsValid(View.GetActor()))
      {
        continue;
      }
      Result->emplace_back(GetOrMakeActor(View));
    }
    // Forget actors that are gone, so their adapters do not pile up.
    for (auto It = Actors.begin(); It != Actors.end();)
    {
      if (Episode.FindCarlaActor(It->first) == nullptr)
      {
        It = Actors.erase(It);
      }
      else
      {
        ++It;
      }
    }
    return Result;
  }

  carla::SharedPtr<const stw::Map> GetMap() const override { return Map; }

  carla::rpc::VehicleLightStateList GetVehiclesLightStates() const override
  {
    carla::rpc::VehicleLightStateList List;
    for (const auto &Pair : Episode.GetActorRegistry())
    {
      const FCarlaActor &View = *Pair.Value.Get();
      if (View.GetActorType() != FCarlaActor::ActorType::Vehicle || View.IsDormant())
      {
        continue;
      }
      const auto *Vehicle = Cast<ACarlaWheeledVehicle>(View.GetActor());
      if (IsValid(Vehicle))
      {
        List.emplace_back(
            View.GetActorId(),
            carla::rpc::VehicleLightState(Vehicle->GetVehicleLightState()).GetLightStateAsValue());
      }
    }
    return List;
  }

  bool IsWeatherEnabled() const override
  {
    return Episode.GetWeather() != nullptr;
  }

  carla::rpc::WeatherParameters GetWeather() const override
  {
    const AWeather *Weather = Episode.GetWeather();
    return Weather != nullptr ?
        carla::rpc::WeatherParameters(Weather->GetCurrentWeather()) :
        carla::rpc::WeatherParameters();
  }

  void ApplyCommands(const std::vector<carla::rpc::Command> &Commands) override
  {
    namespace C = carla::rpc;
    for (const C::Command &Command : Commands)
    {
      if (const auto *Control = std::get_if<C::Command::ApplyVehicleControl>(&Command.command))
      {
        if (FCarlaActor *View = Episode.FindCarlaActor(Control->actor))
        {
          // Autopilot priority: a client's apply_control still overrides it.
          View->ApplyControlToVehicle(FVehicleControl(Control->control), EVehicleInputPriority::Autopilot);
        }
      }
      else if (const auto *Lights = std::get_if<C::Command::SetVehicleLightState>(&Command.command))
      {
        if (FCarlaActor *View = Episode.FindCarlaActor(Lights->actor))
        {
          View->SetVehicleLightState(FVehicleLightState(C::VehicleLightState(Lights->light_state)));
        }
      }
      else if (const auto *Physics = std::get_if<C::Command::SetSimulatePhysics>(&Command.command))
      {
        if (FCarlaActor *View = Episode.FindCarlaActor(Physics->actor))
        {
          View->SetActorSimulatePhysics(Physics->enabled);
        }
      }
      else if (const auto *Transform = std::get_if<C::Command::ApplyTransform>(&Command.command))
      {
        if (FCarlaActor *View = Episode.FindCarlaActor(Transform->actor))
        {
          View->SetActorGlobalTransform(FTransform(Transform->transform));
        }
      }
    }
  }

  carla::SharedPtr<stw::Actor> GetOrMakeActor(const FCarlaActor &View) const
  {
    auto It = Actors.find(View.GetActorId());
    if (It == Actors.end())
    {
      It = Actors.emplace(View.GetActorId(), std::make_shared<FServerTrafficActor>(Episode, View)).first;
    }
    return It->second;
  }

private:

  UCarlaEpisode &Episode;
  carla::SharedPtr<const stw::Map> Map;
  float DeltaSeconds = 0.0f;
  mutable std::unordered_map<carla::ActorId, carla::SharedPtr<stw::Actor>> Actors;
};

// =============================================================================
// -- FServerTraffic -----------------------------------------------------------
// =============================================================================

class FServerTraffic::FImpl
{
public:

  FImpl(UCarlaEpisode &InEpisode, const FServerTrafficSettings &InSettings)
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

    World->SetFrameTime(DeltaSeconds);
    TopUpVehicles();
    TrafficManager->Step();
  }

private:

  bool Initialize()
  {
    ACarlaGameModeBase *GameMode = UCarlaStatics::GetGameMode(Episode.GetWorld());
    if (GameMode == nullptr || !GameMode->GetMap().has_value())
    {
      UE_LOG(LogCarlaServerTraffic, Error, TEXT("Server traffic: the map has no OpenDRIVE road network; disabled"));
      return false;
    }

    World = std::make_unique<FServerTrafficWorld>(Episode, *GameMode->GetMap());
    TrafficManager = std::make_unique<cst::ServerTrafficManager>(*World, static_cast<uint64_t>(Settings.Seed));

    const double Start = FPlatformTime::Seconds();
    TrafficManager->SetupLocalMap(LoadCache(*GameMode));
    UE_LOG(LogCarlaServerTraffic, Log, TEXT("Server traffic: road graph for %s ready in %.0f ms"),
        *Episode.GetMapName(), (FPlatformTime::Seconds() - Start) * 1000.0);

    CollectBlueprints();
    if (Blueprints.Num() == 0)
    {
      UE_LOG(LogCarlaServerTraffic, Error, TEXT("Server traffic: no vehicle blueprint matches '%s'%s; disabled"),
          *Settings.Filter, Settings.bSafe ? TEXT(" with base_type=car") : TEXT(""));
      return false;
    }

    SpawnPoints = GameMode->GetSpawnPointsTransforms();
    if (SpawnPoints.Num() == 0)
    {
      UE_LOG(LogCarlaServerTraffic, Error, TEXT("Server traffic: the map has no spawn points; disabled"));
      return false;
    }
    // Shuffle once, then walk the list: the first wave never reuses a point.
    for (int32 Index = SpawnPoints.Num() - 1; Index > 0; --Index)
    {
      std::uniform_int_distribution<int32> Pick(0, Index);
      SpawnPoints.Swap(Index, Pick(RandomEngine));
    }
    if (Settings.NumberOfVehicles > SpawnPoints.Num())
    {
      UE_LOG(LogCarlaServerTraffic, Warning, TEXT("Server traffic: %d vehicles requested, but the map has %d spawn points"),
          Settings.NumberOfVehicles, SpawnPoints.Num());
    }

    const int32 Spawned = SpawnVehicles(Settings.NumberOfVehicles, SpawnPoints.Num());
    UE_LOG(LogCarlaServerTraffic, Log, TEXT("Server traffic: %d/%d vehicles spawned and driving on %s"),
        Spawned, Settings.NumberOfVehicles, *Episode.GetMapName());
    return true;
  }

  /// The same road-graph cache the client Traffic Manager downloads.
  TArray<uint8> LoadCacheBytes(const ACarlaGameModeBase &GameMode) const
  {
    TArray<FString> Files;
    IFileManager::Get().FindFilesRecursive(
        Files, *(GameMode.GetFullMapPath() / TEXT("TM")), *(Episode.GetMapName() + TEXT(".bin")), true, false, false);
    TArray<uint8> Bytes;
    if (Files.Num() > 0)
    {
      FFileHelper::LoadFileToArray(Bytes, *Files[0]);
    }
    return Bytes;
  }

  std::vector<uint8_t> LoadCache(const ACarlaGameModeBase &GameMode) const
  {
    const TArray<uint8> Bytes = LoadCacheBytes(GameMode);
    return std::vector<uint8_t>(Bytes.GetData(), Bytes.GetData() + Bytes.Num());
  }

  void CollectBlueprints()
  {
    for (const FActorDefinition &Definition : Episode.GetActorDefinitions())
    {
      if (!Definition.Id.StartsWith(TEXT("vehicle.")) || !Definition.Id.MatchesWildcard(Settings.Filter))
      {
        continue;
      }
      if (Settings.bSafe)
      {
        const FActorAttribute *BaseType = Definition.Attributes.FindByPredicate(
            [](const FActorAttribute &Attribute) { return Attribute.Id == TEXT("base_type"); });
        if (BaseType == nullptr || !BaseType->Value.Equals(TEXT("car"), ESearchCase::IgnoreCase))
        {
          continue;
        }
      }
      Blueprints.Add(&Definition);
    }
  }

  /// A description as a client blueprint would send it: every fixed
  /// attribute, and one recommended value per variation.
  FActorDescription MakeDescription(const FActorDefinition &Definition)
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
      std::uniform_int_distribution<int32> Pick(0, Variation.RecommendedValues.Num() - 1);
      FActorAttribute Attribute;
      Attribute.Id = Variation.Id;
      Attribute.Type = Variation.Type;
      Attribute.Value = Variation.RecommendedValues[Pick(RandomEngine)];
      Description.Variations.Add(Attribute.Id, Attribute);
    }
    for (const FActorAttribute &Attribute : Definition.Attributes)
    {
      Description.Variations.Add(Attribute.Id, Attribute);
    }
    FActorAttribute RoleName;
    RoleName.Id = TEXT("role_name");
    RoleName.Type = EActorAttributeType::String;
    RoleName.Value = TEXT("server_traffic");
    Description.Variations.Add(RoleName.Id, RoleName);
    return Description;
  }

  /// Tries up to @a Attempts spawn points; returns how many vehicles spawned.
  int32 SpawnVehicles(int32 Count, int32 Attempts)
  {
    std::vector<carla::SharedPtr<stw::Actor>> Spawned;
    for (int32 Attempt = 0; Attempt < Attempts && static_cast<int32>(Spawned.size()) < Count; ++Attempt)
    {
      const FTransform &SpawnPoint = SpawnPoints[NextSpawnPoint];
      NextSpawnPoint = (NextSpawnPoint + 1) % SpawnPoints.Num();

      std::uniform_int_distribution<int32> PickBlueprint(0, Blueprints.Num() - 1);
      const FActorDefinition &Definition = *Blueprints[PickBlueprint(RandomEngine)];
      auto Result = Episode.SpawnActorWithInfo(SpawnPoint, MakeDescription(Definition));
      if (Result.Key != EActorSpawnResultStatus::Success || Result.Value == nullptr)
      {
        continue; // occupied, most likely: try the next point
      }
      Spawned.emplace_back(World->GetOrMakeActor(*Result.Value));
    }
    if (!Spawned.empty())
    {
      TrafficManager->RegisterVehicles(Spawned);
    }
    return static_cast<int32>(Spawned.size());
  }

  /// The Traffic Manager removes vehicles stuck for too long, and clients can
  /// destroy them: keep the requested number on the road, at most one new
  /// vehicle per second so a blocked map does not retry every frame.
  void TopUpVehicles()
  {
    const int32 Registered = static_cast<int32>(TrafficManager->GetRegisteredVehiclesIDs().size());
    if (Registered >= Settings.NumberOfVehicles)
    {
      return;
    }
    const double Now = Episode.GetElapsedGameTime();
    if (Now - LastTopUp < 1.0)
    {
      return;
    }
    LastTopUp = Now;
    SpawnVehicles(1, FMath::Min(8, SpawnPoints.Num()));
  }

  UCarlaEpisode &Episode;
  FServerTrafficSettings Settings;
  std::mt19937 RandomEngine;

  std::unique_ptr<FServerTrafficWorld> World;
  std::unique_ptr<cst::ServerTrafficManager> TrafficManager;

  TArray<const FActorDefinition *> Blueprints;
  TArray<FTransform> SpawnPoints;
  int32 NextSpawnPoint = 0;
  double LastTopUp = 0.0;
  bool bInitialized = false;
  bool bFailed = false;
};

FServerTraffic::FServerTraffic(UCarlaEpisode &Episode, const FServerTrafficSettings &Settings)
  : Impl(std::make_unique<FImpl>(Episode, Settings)) {}

FServerTraffic::~FServerTraffic() = default;

void FServerTraffic::Tick(float DeltaSeconds)
{
  Impl->Tick(DeltaSeconds);
}
