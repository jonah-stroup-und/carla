// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Game/ServerPlayer.h"
#include "Carla.h"

#include "Carla/Actor/ActorDefinition.h"
#include "Carla/Actor/ActorDescription.h"
#include "Carla/Actor/CarlaActor.h"
#include "Carla/Game/CarlaEpisode.h"
#include "Carla/Game/CarlaGameModeBase.h"
#include "Carla/Game/CarlaStatics.h"
#include "Carla/Game/ServerBlindSpot.h"
#include "Carla/Game/ServerBlinker.h"
#include "Carla/Game/ServerDriveReport.h"
#include "Carla/Game/ServerMirrors.h"
#include "Carla/Game/ServerMission.h"
#include "Carla/Game/ServerPillars.h"
#include "Carla/Game/ServerWheel.h"
#include "Carla/Game/ServerForceFeedback.h"
#include "Carla/Vehicle/CarlaWheeledVehicle.h"
#include "Carla/Vehicle/VehicleControl.h"
#include "Carla/Vehicle/VehicleInputPriority.h"

#include <util/ue-header-guard-begin.h>
#include "Camera/CameraActor.h"
#include "Camera/CameraComponent.h"
#include "Camera/PlayerCameraManager.h"
#include "Components/PrimitiveComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Engine/SkeletalMesh.h"
#include "Debug/DebugDrawService.h"
#include "Engine/Canvas.h"
#include "CanvasItem.h"
#include "TextureResource.h"
#include "Engine/Font.h"
#include "Engine/GameViewportClient.h"
#include "Engine/LocalPlayer.h"
#include "GameMapsSettings.h"
#include "GenericPlatform/GenericApplication.h"
#include "Misc/ConfigCacheIni.h"
#include "Widgets/SWindow.h"
#include "HAL/IConsoleManager.h"
#include "Engine/Engine.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/SpringArmComponent.h"
#include "InputCoreTypes.h"
#include "Kismet/GameplayStatics.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "DynamicRHI.h"
#include "RenderTimer.h"
#include <util/ue-header-guard-end.h>

#include <random>

DEFINE_LOG_CATEGORY_STATIC(LogCarlaServerPlayer, Log, All);

// =============================================================================
// -- Settings -----------------------------------------------------------------
// =============================================================================

FServerPlayerSettings FServerPlayerSettings::FromCommandLine()
{
  FServerPlayerSettings Settings;
  const TCHAR *CommandLine = FCommandLine::Get();
  Settings.bEnabled = FParse::Param(CommandLine, TEXT("server-player"));
  FParse::Value(CommandLine, TEXT("-server-player-vehicle="), Settings.Filter);
  FParse::Value(CommandLine, TEXT("-server-player-seed="), Settings.Seed);
  FParse::Value(CommandLine, TEXT("-server-player-camera="), Settings.Camera);
  FParse::Value(CommandLine, TEXT("-server-player-pitch="), Settings.InteriorPitch);
  Settings.InteriorPitch = FMath::Clamp(Settings.InteriorPitch, -30.0f, 30.0f);
  if (!FParse::Value(CommandLine, TEXT("-server-dlss="), Settings.Dlss) &&
      FParse::Param(CommandLine, TEXT("server-dlss")))
  {
    Settings.Dlss = TEXT("quality");
  }
  if (FParse::Value(CommandLine, TEXT("-server-surround="), Settings.SurroundFov))
  {
    Settings.bSurround = true;
  }
  else
  {
    Settings.bSurround = FParse::Param(CommandLine, TEXT("server-surround"));
  }
  Settings.SurroundFov = FMath::Clamp(Settings.SurroundFov, 20.0f, 120.0f);
  const auto ReadMode = [CommandLine](const TCHAR *Flag, EServerWarningMode &Mode) {
    FString Text;
    if (FParse::Value(CommandLine, Flag, Text) && !ParseServerWarningMode(Text, Mode))
    {
      UE_LOG(LogCarlaServerPlayer, Warning, TEXT("Server player: unknown %s%s (off, visual, audio); using %s"),
          Flag, *Text, GetServerWarningModeName(Mode));
    }
  };
  ReadMode(TEXT("-server-blind-spot="), Settings.BlindSpot);
  ReadMode(TEXT("-server-pillars="), Settings.Pillars);
  Settings.bWarningTest = FParse::Param(CommandLine, TEXT("server-blind-spot-test"));
  return Settings;
}

void FServerPlayerSettings::ConfigureWindowFromCommandLine()
{
  if (!FServerPlayerSettings::FromCommandLine().bSurround)
  {
    return;
  }
  // Read by UGameEngine::CreateGameWindow: no title bar or frame, so the
  // window can cover the monitors edge to edge, and no aspect lock when it
  // is resized to span them.
  const TCHAR *Section = TEXT("/Script/EngineSettings.GeneralProjectSettings");
  GConfig->SetBool(Section, TEXT("bUseBorderlessWindow"), true, GGameIni);
  GConfig->SetBool(Section, TEXT("bShouldWindowPreserveAspectRatio"), false, GGameIni);
}

// =============================================================================
// -- FServerPlayer ------------------------------------------------------------
// =============================================================================

namespace
{
  /// Keyboard steering ramps, so a tap is a small correction.
  constexpr float SteerAttackPerSecond = 2.0f;
  constexpr float SteerReleasePerSecond = 5.0f;
  /// Steering available at and above this speed, as a fraction of full lock.
  constexpr float HighSpeedSteer = 0.35f;
  /// The speedometer turns red this far over the limit (the drive report
  /// starts a speeding stretch 2 km/h, about 1.2 mph, over it).
  constexpr float SpeedometerOverMph = 1.0f;
  constexpr float HighSpeedKmh = 120.0f;
  /// The gear only changes below this speed (cm/s, about 5 km/h).
  constexpr float GearSwitchSpeed = 140.0f;
  constexpr float GearRefusedHudSeconds = 1.5f;
  constexpr float StickDeadZone = 0.15f;
  /// The FPS counter averages frames over this window.
  constexpr double FpsWindowSeconds = 0.5;
  /// A turn signal cancels itself this long after the lane change it
  /// announced, or when the steering returns after a turn.
  constexpr double SignalCancelAfterLaneChange = 1.5;
  constexpr float SignalTurnSteer = 0.3f;
  constexpr float SignalCenteredSteer = 0.05f;

  /// The window's upscaler: Unreal's TSR at its automatic screen percentage,
  /// or DLSS Super Resolution (DLSSRRDenoiser's r.DLSSSR.Viewport) at NVIDIA's
  /// quality-mode render scales.
  enum class EUpscaler : uint8 { Tsr, Dlaa, Quality, Balanced, Performance, Count };

  struct FUpscalerMode
  {
    const TCHAR *Name;
    const TCHAR *Flag;
    bool bDlss;
    float ScreenPercentage; // 0: Unreal's automatic, based on display resolution
  };

  constexpr FUpscalerMode UpscalerModes[] = {
    {TEXT("TSR"),               TEXT("off"),         false, 0.0f},
    {TEXT("DLAA"),              TEXT("dlaa"),        true,  100.0f},
    {TEXT("DLSS Quality"),      TEXT("quality"),     true,  66.7f},
    {TEXT("DLSS Balanced"),     TEXT("balanced"),    true,  58.0f},
    {TEXT("DLSS Performance"),  TEXT("performance"), true,  50.0f},
  };

  const FUpscalerMode &GetMode(EUpscaler Upscaler)
  {
    return UpscalerModes[static_cast<uint8>(Upscaler)];
  }

  enum class ECameraMode : uint8 { Chase, FarChase, Interior, Count };

  /// Surround: the three monitors left to right, and which monitor each
  /// local player sees. Player 0 (keyboard, first gamepad) takes the centre.
  constexpr int32 SurroundScreens = 3;
  constexpr int32 SurroundCentre = 1;
  constexpr int32 SurroundPlayerScreen[SurroundScreens] = {1, 0, 2};

  const TCHAR *CameraModeName(ECameraMode Mode)
  {
    switch (Mode)
    {
      case ECameraMode::Chase:    return TEXT("chase");
      case ECameraMode::FarChase: return TEXT("far chase");
      case ECameraMode::Interior: return TEXT("interior");
      default:                    return TEXT("?");
    }
  }
}

class FServerPlayer::FImpl
{
public:

  FImpl(UCarlaEpisode &InEpisode, const FServerPlayerSettings &InSettings)
    : Episode(InEpisode),
      Settings(InSettings),
      RandomEngine(static_cast<uint32>(InSettings.Seed))
  {
    Wheel = MakeUnique<FServerWheel>();
    ForceFeedback = MakeUnique<FServerForceFeedback>();
    // A canvas overlay rather than "stat fps": stat commands are compiled
    // out of Shipping builds.
    FpsDrawHandle = UDebugDrawService::Register(
        TEXT("Game"), FDebugDrawDelegate::CreateRaw(this, &FImpl::DrawFps));
    if (!Settings.Dlss.IsEmpty())
    {
      bool bKnown = false;
      for (uint8 Index = 0; Index < static_cast<uint8>(EUpscaler::Count); ++Index)
      {
        if (Settings.Dlss.Equals(UpscalerModes[Index].Flag, ESearchCase::IgnoreCase))
        {
          Upscaler = static_cast<EUpscaler>(Index);
          bKnown = true;
        }
      }
      if (!bKnown)
      {
        UE_LOG(LogCarlaServerPlayer, Warning,
            TEXT("Server player: unknown -server-dlss=%s (dlaa, quality, balanced, performance, off); using quality"),
            *Settings.Dlss);
        Upscaler = EUpscaler::Quality;
      }
      ApplyUpscaler();
    }
    // The driver's seat unless asked otherwise.
    CameraMode = ECameraMode::Interior;
    if (Settings.Camera.Equals(TEXT("chase"), ESearchCase::IgnoreCase))
    {
      CameraMode = ECameraMode::Chase;
    }
    else if (Settings.Camera.Equals(TEXT("far"), ESearchCase::IgnoreCase))
    {
      CameraMode = ECameraMode::FarChase;
    }
  }

  ~FImpl()
  {
    ForceFeedback.Reset(); // the wheel goes limp
    Mission.Reset();
    Report.reset();
    BlindSpot.Reset();
    Pillars.Reset();
    Blinker.Reset();
    Mirrors.Reset();
    UDebugDrawService::Unregister(FpsDrawHandle);
    for (TWeakObjectPtr<ACameraActor> &SideCamera : SideCameras)
    {
      if (SideCamera.IsValid())
      {
        SideCamera->Destroy();
      }
    }
  }

  void Tick(float DeltaSeconds)
  {
    if (bFailed)
    {
      return;
    }
    APlayerController *Controller = UGameplayStatics::GetPlayerController(Episode.GetWorld(), 0);
    if (Controller == nullptr)
    {
      return; // not yet: retry next frame
    }
    Wheel->Poll();
    if (!Vehicle.IsValid())
    {
      if (bSpawned)
      {
        UE_LOG(LogCarlaServerPlayer, Warning, TEXT("Server player: the vehicle was destroyed; spawning a new one"));
      }
      Report.reset(); // that drive is over
      BlindSpot.Reset();
      Pillars.Reset();
      Blinker.Reset();
      Mirrors.Reset();
      if (!Spawn())
      {
        bFailed = true;
        return;
      }
      AttachCamera();
      Report = std::make_unique<FServerDriveReport>(Episode, *Vehicle, BlueprintId);
      Mirrors = MakeUnique<FServerMirrors>(*Vehicle, BlueprintId);
      SwapMesh(Mirrors->GetMesh());
      BlindSpot = MakeUnique<FServerBlindSpot>(*Vehicle, BoundsOrigin, BoundsExtent, GetEyeLocation().X, Settings.BlindSpot);
      BlindSpot->SetTest(Settings.bWarningTest);
      Pillars = MakeUnique<FServerPillars>(*Vehicle, BlueprintId, Settings.Pillars);
      Pillars->SetTest(Settings.bWarningTest);
      Blinker = MakeUnique<FServerBlinker>(*Vehicle);
      Report->SetAidMode(BlindSpotAid, GetServerWarningModeName(Settings.BlindSpot));
      if (Pillars->HasPillars())
      {
        Report->SetAidMode(PillarAid, GetServerWarningModeName(Settings.Pillars));
      }
      bSpawned = true;
    }

    if (!Mission)
    {
      Mission = MakeUnique<FServerMission>(Episode);
    }
    // M: the mission planner (a top-down map), and back to driving.
    if (Pressed(*Controller, EKeys::M, bMissionKeyDown) | Wheel->WasPressed(EServerWheelAction::Mission))
    {
      Mission->SetPlanning(!Mission->IsPlanning(), *Controller, *Vehicle);
    }
    const bool bPlanning = Mission->IsPlanning();
    if ((Pressed(*Controller, EKeys::R, bRestartKeyDown) | Wheel->WasPressed(EServerWheelAction::Restart)) && !bPlanning)
    {
      Mission->Restart();
    }
    if (CheckQuit(*Controller))
    {
      return;
    }

    TakeView(*Controller, !bPlanning);
    if (Settings.bSurround && !bSurroundFailed)
    {
      if (!bSurroundReady)
      {
        bSurroundReady = SetupSurround();
        bSurroundFailed = !bSurroundReady;
        AttachSurroundCameras();
      }
      TakeSurroundViews();
    }
    if (Pressed(*Controller, EKeys::F9, bUpscalerKeyDown))
    {
      Upscaler = static_cast<EUpscaler>((static_cast<uint8>(Upscaler) + 1) % static_cast<uint8>(EUpscaler::Count));
      ApplyUpscaler();
    }
    if (bPlanning)
    {
      HoldStill(*Vehicle);
    }
    else
    {
      Drive(*Controller, DeltaSeconds);
      Signal(*Controller);
    }
    {
      const FVehicleLightState Lights = Vehicle->GetVehicleLightState();
      Blinker->Tick(Lights.LeftBlinker, Lights.RightBlinker);
    }
    MonitorBlindSpot(*Controller, DeltaSeconds);
    MonitorPillars(*Controller, DeltaSeconds);
    Report->Tick();
    if (ForceFeedback)
    {
      float Side = 0.0f;
      const float Impulse = Report->TakeImpact(Side);
      if (Impulse > 0.0f)
      {
        ForceFeedback->AddImpact(Impulse, Side);
      }
      ForceFeedback->Tick(DeltaSeconds, Vehicle->GetVehicleForwardSpeed() * 0.01f, !bPlanning);
    }
    Mission->Tick(*Controller, *Vehicle, *Report, DeltaSeconds);
    // The mirrors are aimed from the driver's eye, and only render while
    // the driver can see them.
    Mirrors->Tick(Vehicle->GetActorTransform().TransformPosition(GetMirrorEyeLocation()),
        CameraMode == ECameraMode::Interior && !bPlanning);
    // The config's eye can move while driving.
    if (CameraMode == ECameraMode::Interior && Arm.IsValid() &&
        !Arm->GetRelativeLocation().Equals(GetEyeLocation(), 0.01))
    {
      Arm->SetRelativeLocation(GetEyeLocation());
    }
    FollowWithSpectator(*Controller);
    DrawHud();
  }

private:

  // ---------------------------------------------------------------------------
  // Spawn
  // ---------------------------------------------------------------------------

  bool Spawn()
  {
    ACarlaGameModeBase *GameMode = UCarlaStatics::GetGameMode(Episode.GetWorld());
    if (GameMode == nullptr)
    {
      UE_LOG(LogCarlaServerPlayer, Error, TEXT("Server player: no CARLA game mode; disabled"));
      return false;
    }

    TArray<const FActorDefinition *> Blueprints = CollectBlueprints(Settings.Filter);
    if (Blueprints.Num() == 0)
    {
      UE_LOG(LogCarlaServerPlayer, Warning, TEXT("Server player: no car matches '%s'; picking any car"), *Settings.Filter);
      Blueprints = CollectBlueprints(TEXT("vehicle.*"));
    }
    if (Blueprints.Num() == 0)
    {
      UE_LOG(LogCarlaServerPlayer, Error, TEXT("Server player: no car blueprint available; disabled"));
      return false;
    }
    std::uniform_int_distribution<int32> PickBlueprint(0, Blueprints.Num() - 1);
    const FActorDefinition &Definition = *Blueprints[PickBlueprint(RandomEngine)];

    TArray<FTransform> SpawnPoints = GameMode->GetSpawnPointsTransforms();
    if (SpawnPoints.Num() == 0)
    {
      UE_LOG(LogCarlaServerPlayer, Error, TEXT("Server player: the map has no spawn points; disabled"));
      return false;
    }
    for (int32 Index = SpawnPoints.Num() - 1; Index > 0; --Index)
    {
      std::uniform_int_distribution<int32> Pick(0, Index);
      SpawnPoints.Swap(Index, Pick(RandomEngine));
    }

    for (const FTransform &SpawnPoint : SpawnPoints)
    {
      auto Result = Episode.SpawnActorWithInfo(SpawnPoint, MakeDescription(Definition));
      if (Result.Key != EActorSpawnResultStatus::Success || Result.Value == nullptr)
      {
        continue; // occupied, most likely
      }
      ACarlaWheeledVehicle *Spawned = Cast<ACarlaWheeledVehicle>(Result.Value->GetActor());
      if (Spawned == nullptr)
      {
        continue;
      }
      Vehicle = Spawned;
      BlueprintId = Definition.Id;
      const FActorInfo *Info = Result.Value->GetActorInfo();
      if (Info != nullptr)
      {
        BoundsOrigin = Info->BoundingBox.Origin;
        BoundsExtent = Info->BoundingBox.Extent;
      }
      UE_LOG(LogCarlaServerPlayer, Log, TEXT("Server player: driving %s (actor %u) on %s"),
          *Definition.Id, Result.Value->GetActorId(), *Episode.GetMapName());
      return true;
    }
    UE_LOG(LogCarlaServerPlayer, Error, TEXT("Server player: every spawn point is occupied; disabled"));
    return false;
  }

  TArray<const FActorDefinition *> CollectBlueprints(const FString &Filter) const
  {
    TArray<const FActorDefinition *> Result;
    for (const FActorDefinition &Definition : Episode.GetActorDefinitions())
    {
      if (!Definition.Id.StartsWith(TEXT("vehicle.")) || !Definition.Id.MatchesWildcard(Filter))
      {
        continue;
      }
      const FActorAttribute *BaseType = Definition.Attributes.FindByPredicate(
          [](const FActorAttribute &Attribute) { return Attribute.Id == TEXT("base_type"); });
      if (BaseType == nullptr || !BaseType->Value.Equals(TEXT("car"), ESearchCase::IgnoreCase))
      {
        continue;
      }
      Result.Add(&Definition);
    }
    return Result;
  }

  /// A description as a client blueprint would send it, with role_name hero.
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
    RoleName.Value = TEXT("hero");
    Description.Variations.Add(RoleName.Id, RoleName);
    return Description;
  }

  // ---------------------------------------------------------------------------
  // Camera
  // ---------------------------------------------------------------------------

  void AttachCamera()
  {
    ACarlaWheeledVehicle *Car = Vehicle.Get();
    // The view target uses the first active camera it finds: ours only.
    TInlineComponentArray<UCameraComponent *> Existing;
    Car->GetComponents(Existing);
    for (UCameraComponent *Other : Existing)
    {
      Other->Deactivate();
    }

    USpringArmComponent *NewArm = NewObject<USpringArmComponent>(Car, TEXT("ServerPlayerCameraArm"));
    NewArm->SetupAttachment(Car->GetRootComponent());
    NewArm->bInheritRoll = false;
    NewArm->RegisterComponent();
    Car->AddInstanceComponent(NewArm);

    UCameraComponent *NewCamera = NewObject<UCameraComponent>(Car, TEXT("ServerPlayerCamera"));
    NewCamera->SetupAttachment(NewArm, USpringArmComponent::SocketName);
    NewCamera->FieldOfView = 90.0f;
    if (bSurroundReady)
    {
      ApplySurroundLens(*NewCamera, SurroundFovs[SurroundCentre]);
    }
    NewCamera->RegisterComponent();
    Car->AddInstanceComponent(NewCamera);

    Arm = NewArm;
    Camera = NewCamera;
    ApplyCameraMode();
    AttachSurroundCameras();
  }

  // ---------------------------------------------------------------------------
  // Surround (three monitors)
  // ---------------------------------------------------------------------------

  /// Spans the window over the three leftmost monitors, splits it into one
  /// view per monitor (Unreal's three-player split screen, laid out on the
  /// monitors' rectangles), and works out each screen's field of view and
  /// heading. All screens share one vertical field of view, so the picture
  /// lines up across them, and each side screen's heading puts its inner
  /// edge where the centre screen's edge is.
  bool SetupSurround()
  {
    UGameViewportClient *Viewport = GEngine != nullptr ? GEngine->GameViewport : nullptr;
    TSharedPtr<SWindow> Window = Viewport != nullptr ? Viewport->GetWindow() : nullptr;
    if (!Window.IsValid())
    {
      UE_LOG(LogCarlaServerPlayer, Error, TEXT("Server player: surround needs a game window; disabled"));
      return false;
    }

    FDisplayMetrics Metrics;
    FDisplayMetrics::RebuildDisplayMetrics(Metrics);
    TArray<FPlatformRect> Monitors;
    for (const FMonitorInfo &Info : Metrics.MonitorInfo)
    {
      Monitors.Add(Info.DisplayRect);
    }
    if (Monitors.Num() < SurroundScreens)
    {
      UE_LOG(LogCarlaServerPlayer, Error, TEXT("Server player: surround needs 3 monitors, found %d; disabled"), Monitors.Num());
      return false;
    }
    Monitors.Sort([](const FPlatformRect &A, const FPlatformRect &B) { return A.Left < B.Left; });
    if (Monitors.Num() > SurroundScreens)
    {
      UE_LOG(LogCarlaServerPlayer, Warning, TEXT("Server player: %d monitors; surround uses the three leftmost"), Monitors.Num());
      Monitors.SetNum(SurroundScreens);
    }

    FPlatformRect Bounds = Monitors[0];
    for (const FPlatformRect &Monitor : Monitors)
    {
      Bounds.Left = FMath::Min(Bounds.Left, Monitor.Left);
      Bounds.Top = FMath::Min(Bounds.Top, Monitor.Top);
      Bounds.Right = FMath::Max(Bounds.Right, Monitor.Right);
      Bounds.Bottom = FMath::Max(Bounds.Bottom, Monitor.Bottom);
    }
    const float Width = static_cast<float>(Bounds.Right - Bounds.Left);
    const float Height = static_cast<float>(Bounds.Bottom - Bounds.Top);

    if (Window->HasOSWindowBorder())
    {
      UE_LOG(LogCarlaServerPlayer, Warning, TEXT("Server player: the window has a frame; surround will show its title bar"));
    }
    Window->SetWindowMode(EWindowMode::Windowed);
    Window->ReshapeWindow(FVector2D(Bounds.Left, Bounds.Top), FVector2D(Width, Height));

    // Fields of view from the centre screen's, at a shared vertical one.
    const auto AspectOf = [](const FPlatformRect &Monitor) {
      return static_cast<float>(Monitor.Right - Monitor.Left) / static_cast<float>(Monitor.Bottom - Monitor.Top);
    };
    const float TanHalfVertical = FMath::Tan(FMath::DegreesToRadians(Settings.SurroundFov * 0.5f)) / AspectOf(Monitors[SurroundCentre]);
    for (int32 Screen = 0; Screen < SurroundScreens; ++Screen)
    {
      SurroundFovs[Screen] = 2.0f * FMath::RadiansToDegrees(FMath::Atan(TanHalfVertical * AspectOf(Monitors[Screen])));
    }
    SurroundYaws[0] = -0.5f * (SurroundFovs[SurroundCentre] + SurroundFovs[0]);
    SurroundYaws[SurroundCentre] = 0.0f;
    SurroundYaws[2] = 0.5f * (SurroundFovs[SurroundCentre] + SurroundFovs[2]);
    SurroundCentreWidth = static_cast<float>(Monitors[SurroundCentre].Right - Monitors[SurroundCentre].Left);

    // Three-player split screen, each player's view on its monitor.
    UGameMapsSettings *Maps = GetMutableDefault<UGameMapsSettings>();
    Maps->bUseSplitscreen = true;
    Maps->ThreePlayerSplitscreenLayout = EThreePlayerSplitScreenType::Vertical;
    Viewport->SetForceDisableSplitscreen(false);
    TArray<FPerPlayerSplitscreenData> &Layout = Viewport->SplitscreenInfo[ESplitScreenType::ThreePlayer_Vertical].PlayerData;
    Layout.Reset();
    for (int32 Player = 0; Player < SurroundScreens; ++Player)
    {
      const FPlatformRect &Monitor = Monitors[SurroundPlayerScreen[Player]];
      Layout.Add(FPerPlayerSplitscreenData(
          (Monitor.Right - Monitor.Left) / Width, (Monitor.Bottom - Monitor.Top) / Height,
          (Monitor.Left - Bounds.Left) / Width, (Monitor.Top - Bounds.Top) / Height));
    }

    // The two side players (local players outlive the map, so a reloaded
    // episode reuses them).
    UWorld *World = Episode.GetWorld();
    while (UGameplayStatics::GetPlayerController(World, SurroundScreens - 1) == nullptr)
    {
      if (UGameplayStatics::CreatePlayer(World, -1, true) == nullptr)
      {
        UE_LOG(LogCarlaServerPlayer, Error, TEXT("Server player: cannot add the side screens' players; surround disabled"));
        Maps->bUseSplitscreen = false;
        return false;
      }
    }

    if (UCameraComponent *Centre = Camera.Get())
    {
      ApplySurroundLens(*Centre, SurroundFovs[SurroundCentre]);
    }
    UE_LOG(LogCarlaServerPlayer, Log,
        TEXT("Server player: surround over %.0fx%.0f at (%d, %d): left %.1f deg at %.1f, centre %.1f deg, right %.1f deg at %.1f"),
        Width, Height, Bounds.Left, Bounds.Top, SurroundFovs[0], SurroundYaws[0], SurroundFovs[SurroundCentre],
        SurroundFovs[2], SurroundYaws[2]);
    return true;
  }

  /// The field of view is horizontal across the camera's own view, whatever
  /// the project's aspect-ratio axis constraint.
  static void ApplySurroundLens(UCameraComponent &Lens, float Fov)
  {
    Lens.SetFieldOfView(Fov);
    Lens.SetConstraintAspectRatio(false);
    Lens.bOverrideAspectRatioAxisConstraint = true;
    Lens.SetAspectRatioAxisConstraint(EAspectRatioAxisConstraint::AspectRatio_MaintainXFOV);
  }

  /// The side screens' cameras ride on the same spring arm as the centre
  /// one, turned by their headings, so every camera mode works across all
  /// three screens.
  void AttachSurroundCameras()
  {
    USpringArmComponent *SpringArm = Arm.Get();
    if (!bSurroundReady || SpringArm == nullptr)
    {
      return;
    }
    for (int32 Side = 0; Side < 2; ++Side)
    {
      const int32 Screen = SurroundPlayerScreen[Side + 1];
      ACameraActor *SideCamera = SideCameras[Side].Get();
      if (SideCamera == nullptr)
      {
        FActorSpawnParameters Parameters;
        Parameters.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
        SideCamera = Episode.GetWorld()->SpawnActor<ACameraActor>(Parameters);
        SideCameras[Side] = SideCamera;
      }
      ApplySurroundLens(*SideCamera->GetCameraComponent(), SurroundFovs[Screen]);
      SideCamera->AttachToComponent(SpringArm, FAttachmentTransformRules::KeepRelativeTransform, USpringArmComponent::SocketName);
      SideCamera->SetActorRelativeLocation(FVector::ZeroVector);
      SideCamera->SetActorRelativeRotation(FRotator(0.0f, SurroundYaws[Screen], 0.0f));
    }
  }

  void TakeSurroundViews()
  {
    if (!bSurroundReady)
    {
      return;
    }
    for (int32 Side = 0; Side < 2; ++Side)
    {
      APlayerController *SideController = UGameplayStatics::GetPlayerController(Episode.GetWorld(), Side + 1);
      ACameraActor *SideCamera = SideCameras[Side].Get();
      if (SideController == nullptr || SideCamera == nullptr)
      {
        continue;
      }
      SideController->bAutoManageActiveCameraTarget = false;
      if (SideController->GetViewTarget() != SideCamera)
      {
        SideController->SetViewTarget(SideCamera);
      }
      APawn *SidePawn = SideController->GetPawn();
      if (SidePawn != nullptr && SidePawn->InputEnabled())
      {
        SidePawn->DisableInput(SideController);
      }
    }
  }

  void ApplyCameraMode()
  {
    USpringArmComponent *SpringArm = Arm.Get();
    if (SpringArm == nullptr)
    {
      return;
    }
    // Sized from the vehicle's bounding box (cm, actor space), so a bus and
    // a microcar both frame sensibly.
    const float Length = FMath::Max(2.0f * BoundsExtent.X, 300.0f);
    const float Roof = BoundsOrigin.Z + FMath::Max(BoundsExtent.Z, 70.0f);
    switch (CameraMode)
    {
      case ECameraMode::Chase:
      case ECameraMode::FarChase:
      {
        const bool bFar = CameraMode == ECameraMode::FarChase;
        SpringArm->SetRelativeLocation(FVector(BoundsOrigin.X, 0.0f, Roof));
        SpringArm->SetRelativeRotation(FRotator(bFar ? -15.0f : -8.0f, 0.0f, 0.0f));
        SpringArm->TargetArmLength = Length * (bFar ? 2.6f : 1.5f) + 150.0f;
        SpringArm->SocketOffset = FVector(0.0f, 0.0f, bFar ? 150.0f : 60.0f);
        SpringArm->bDoCollisionTest = true;
        SpringArm->bEnableCameraLag = true;
        SpringArm->bEnableCameraRotationLag = true;
        SpringArm->CameraLagSpeed = 12.0f;
        SpringArm->CameraRotationLagSpeed = 8.0f;
        SpringArm->bInheritPitch = false;
        break;
      }
      case ECameraMode::Interior:
      default:
      {
        // The driver's eyes: left seat (UE's +Y is right), a little behind
        // the middle of the car, at about three quarters of its height.
        SpringArm->SetRelativeLocation(GetEyeLocation());
        SpringArm->SetRelativeRotation(FRotator(Settings.InteriorPitch, 0.0f, 0.0f));
        SpringArm->TargetArmLength = 0.0f;
        SpringArm->SocketOffset = FVector::ZeroVector;
        SpringArm->bDoCollisionTest = false;
        SpringArm->bEnableCameraLag = false;
        SpringArm->bEnableCameraRotationLag = false;
        SpringArm->bInheritPitch = true;
        break;
      }
    }
  }

  /// The driver's eyes, in actor space: the vehicle's "eye" in the mirror
  /// config, else the left seat (UE's +Y is right), a little behind the
  /// middle of the car, at about three quarters of its height.
  FVector GetEyeLocation() const
  {
    FVector Eye;
    if (Mirrors && Mirrors->GetEye(Eye))
    {
      return Eye;
    }
    return GetDefaultEyeLocation();
  }

  /// The eye the mirrors are aimed from: the config's "mirror_eye", else
  /// the driver's eye.
  FVector GetMirrorEyeLocation() const
  {
    FVector Eye;
    if (Mirrors && Mirrors->GetMirrorEye(Eye))
    {
      return Eye;
    }
    return GetDefaultEyeLocation();
  }

  FVector GetDefaultEyeLocation() const
  {
    return FVector(
        BoundsOrigin.X - 0.08f * BoundsExtent.X,
        BoundsOrigin.Y - 0.36f * BoundsExtent.Y,
        BoundsOrigin.Z + 0.5f * BoundsExtent.Z);
  }

  /// Swaps the car's skeletal mesh for the config's "mesh" (e.g. one without
  /// the front headrests), keeping its materials.
  void SwapMesh(const FString &Path)
  {
    ACarlaWheeledVehicle *Car = Vehicle.Get();
    if (Path.IsEmpty() || Car == nullptr)
    {
      return;
    }
    USkeletalMesh *Mesh = LoadObject<USkeletalMesh>(nullptr, *Path);
    if (Mesh == nullptr)
    {
      UE_LOG(LogCarlaServerPlayer, Warning, TEXT("Server player: cannot load mesh %s; keeping the vehicle's"), *Path);
      return;
    }
    Car->GetMesh()->SetSkeletalMeshAsset(Mesh);
    UE_LOG(LogCarlaServerPlayer, Log, TEXT("Server player: car mesh %s"), *Path);
  }

  /// The driver's view (unless the mission planner has it).
  void TakeView(APlayerController &Controller, bool bDriverView)
  {
    ACarlaWheeledVehicle *Car = Vehicle.Get();
    Controller.bAutoManageActiveCameraTarget = false;
    if (bDriverView && Controller.GetViewTarget() != Car)
    {
      Controller.SetViewTarget(Car);
    }
    // The spectator stays possessed so clients keep their spectator, but it
    // must not fly around on WASD while the player drives.
    APawn *Spectator = Episode.GetSpectatorPawn();
    if (Spectator != nullptr && Spectator->InputEnabled())
    {
      Spectator->DisableInput(&Controller);
    }
  }

  void FollowWithSpectator(APlayerController &Controller)
  {
    APawn *Spectator = Episode.GetSpectatorPawn();
    if (Spectator == nullptr || Controller.PlayerCameraManager == nullptr)
    {
      return;
    }
    Spectator->SetActorLocationAndRotation(
        Controller.PlayerCameraManager->GetCameraLocation(),
        Controller.PlayerCameraManager->GetCameraRotation());
  }

  // ---------------------------------------------------------------------------
  // Upscaler
  // ---------------------------------------------------------------------------

  void ApplyUpscaler()
  {
    const FUpscalerMode &Mode = GetMode(Upscaler);
    IConsoleVariable *DlssViewport = IConsoleManager::Get().FindConsoleVariable(TEXT("r.DLSSSR.Viewport"));
    IConsoleVariable *ScreenPercentage = IConsoleManager::Get().FindConsoleVariable(TEXT("r.ScreenPercentage"));
    if (Mode.bDlss && DlssViewport == nullptr)
    {
      UE_LOG(LogCarlaServerPlayer, Warning,
          TEXT("Server player: %s needs the DLSSRRDenoiser plugin built with the DLSS SDK; keeping TSR"), Mode.Name);
      Upscaler = EUpscaler::Tsr;
      return;
    }
    if (DlssViewport != nullptr)
    {
      DlssViewport->Set(Mode.bDlss ? 1 : 0, ECVF_SetByCode);
    }
    if (ScreenPercentage != nullptr)
    {
      ScreenPercentage->Set(Mode.ScreenPercentage, ECVF_SetByCode);
    }
    UE_LOG(LogCarlaServerPlayer, Log, TEXT("Server player: upscaler %s"), Mode.Name);
  }

  // ---------------------------------------------------------------------------
  // Driving
  // ---------------------------------------------------------------------------

  bool Pressed(const APlayerController &Controller, const FKey &Key, bool &bWasDown)
  {
    const bool bDown = Controller.IsInputKeyDown(Key);
    const bool bEdge = bDown && !bWasDown;
    bWasDown = bDown;
    return bEdge;
  }

  float Analog(const APlayerController &Controller, const FKey &Key) const
  {
    const float Value = Controller.GetInputAnalogKeyState(Key);
    return FMath::Abs(Value) < StickDeadZone ? 0.0f : Value;
  }

  /// Esc twice within a few seconds closes the game (the drive report is
  /// written on the way out).
  bool CheckQuit(APlayerController &Controller)
  {
    if (!Pressed(Controller, EKeys::Escape, bQuitKeyDown))
    {
      return false;
    }
    const double Now = FPlatformTime::Seconds();
    if (Now - QuitPromptTime <= QuitConfirmSeconds)
    {
      UE_LOG(LogCarlaServerPlayer, Log, TEXT("Server player: Esc pressed twice; quitting"));
      ForceFeedback.Reset(); // limp at once, not after the shutdown
      Controller.ConsoleCommand(TEXT("quit"));
      return true;
    }
    QuitPromptTime = Now;
    return false;
  }

  /// Brakes on while the mission planner is open.
  void HoldStill(ACarlaWheeledVehicle &Car)
  {
    FVehicleControl Control;
    Control.Brake = 1.0f;
    Control.bHandBrake = true;
    Control.bReverse = bReverse;
    Car.ApplyVehicleControl(Control, EVehicleInputPriority::User);
  }

  void Drive(APlayerController &Controller, float DeltaSeconds)
  {
    ACarlaWheeledVehicle *Car = Vehicle.Get();

    // Page Up / Page Down tilt the interior view, a degree at a time.
    const bool bPitchUp = Pressed(Controller, EKeys::PageUp, bPitchUpKeyDown);
    const bool bPitchDown = Pressed(Controller, EKeys::PageDown, bPitchDownKeyDown);
    if ((bPitchUp || bPitchDown) && CameraMode == ECameraMode::Interior)
    {
      Settings.InteriorPitch = FMath::Clamp(Settings.InteriorPitch + (bPitchUp ? 1.0f : -1.0f), -30.0f, 30.0f);
      ApplyCameraMode();
      UE_LOG(LogCarlaServerPlayer, Log, TEXT("Server player: interior pitch %.0f deg (-server-player-pitch=%.0f)"),
          Settings.InteriorPitch, Settings.InteriorPitch);
    }
    if (Pressed(Controller, EKeys::C, bCameraKeyDown) | Pressed(Controller, EKeys::Gamepad_FaceButton_Top, bCameraPadDown) | Wheel->WasPressed(EServerWheelAction::Camera))
    {
      CameraMode = static_cast<ECameraMode>((static_cast<uint8>(CameraMode) + 1) % static_cast<uint8>(ECameraMode::Count));
      ApplyCameraMode();
    }
    if (Pressed(Controller, EKeys::BackSpace, bResetKeyDown) | Pressed(Controller, EKeys::Gamepad_Special_Left, bResetPadDown) | Wheel->WasPressed(EServerWheelAction::Reset))
    {
      ResetUpright(*Car);
    }

    const auto Down = [&Controller](const FKey &A, const FKey &B) {
      return Controller.IsInputKeyDown(A) || Controller.IsInputKeyDown(B);
    };
    const float Throttle = FMath::Max(Down(EKeys::W, EKeys::Up) ? 1.0f : 0.0f,
        FMath::Max(Analog(Controller, EKeys::Gamepad_RightTriggerAxis), Wheel->GetThrottle()));
    const float Brake = FMath::Max(Down(EKeys::S, EKeys::Down) ? 1.0f : 0.0f,
        FMath::Max(Analog(Controller, EKeys::Gamepad_LeftTriggerAxis), Wheel->GetBrake()));

    // Q toggles drive/reverse, only when nearly stopped; W drives in that gear.
    const float Speed = Car->GetVehicleForwardSpeed(); // cm/s, negative backwards
    if (Pressed(Controller, EKeys::Q, bGearKeyDown) | Pressed(Controller, EKeys::Gamepad_FaceButton_Left, bGearPadDown) | Wheel->WasPressed(EServerWheelAction::Gear))
    {
      if (FMath::Abs(Speed) < GearSwitchSpeed)
      {
        bReverse = !bReverse;
      }
      else
      {
        GearRefusedUntil = FPlatformTime::Seconds() + GearRefusedHudSeconds;
      }
    }

    // Less steering lock at speed, so the keyboard is drivable on a highway.
    const float SpeedKmh = FMath::Abs(Speed) * 0.036f;
    const float MaxSteer = FMath::Lerp(1.0f, HighSpeedSteer, FMath::Clamp(SpeedKmh / HighSpeedKmh, 0.0f, 1.0f));
    const float Stick = Analog(Controller, EKeys::Gamepad_LeftX);
    if (Wheel->IsConnected())
    {
      // A wheel steers one to one: its lock-to-lock is set on the base.
      Steer = Wheel->GetSteer();
    }
    else if (Stick != 0.0f)
    {
      Steer = Stick * MaxSteer;
    }
    else
    {
      const float Target = ((Down(EKeys::D, EKeys::Right) ? 1.0f : 0.0f) - (Down(EKeys::A, EKeys::Left) ? 1.0f : 0.0f)) * MaxSteer;
      const bool bReleasing = FMath::Abs(Target) < FMath::Abs(Steer) || Target * Steer < 0.0f;
      const float Rate = bReleasing ? SteerReleasePerSecond : SteerAttackPerSecond;
      Steer = FMath::FInterpConstantTo(Steer, Target, DeltaSeconds, Rate);
    }

    FVehicleControl Control;
    Control.Throttle = Throttle;
    Control.Brake = Brake;
    Control.Steer = FMath::Clamp(Steer, -1.0f, 1.0f);
    Control.bHandBrake = Controller.IsInputKeyDown(EKeys::SpaceBar) || Controller.IsInputKeyDown(EKeys::Gamepad_FaceButton_Bottom) ||
        Wheel->IsDown(EServerWheelAction::HandBrake);
    Control.bReverse = bReverse;
    Car->ApplyVehicleControl(Control, EVehicleInputPriority::User);
  }

  /// Z/X (or the d-pad) toggle the left/right turn signal; a signal cancels
  /// itself after a lane change or a turn, like a real one.
  void Signal(APlayerController &Controller)
  {
    ACarlaWheeledVehicle *Car = Vehicle.Get();
    FVehicleLightState Lights = Car->GetVehicleLightState();
    const bool bLeftPressed = Pressed(Controller, EKeys::Z, bLeftKeyDown) | Pressed(Controller, EKeys::Gamepad_DPad_Left, bLeftPadDown) | Wheel->WasPressed(EServerWheelAction::SignalLeft);
    const bool bRightPressed = Pressed(Controller, EKeys::X, bRightKeyDown) | Pressed(Controller, EKeys::Gamepad_DPad_Right, bRightPadDown) | Wheel->WasPressed(EServerWheelAction::SignalRight);
    bool bLeft = Lights.LeftBlinker;
    bool bRight = Lights.RightBlinker;
    if (bLeftPressed)
    {
      bLeft = !bLeft;
      bRight = false;
    }
    if (bRightPressed)
    {
      bRight = !bRight;
      bLeft = false;
    }
    if (bLeftPressed || bRightPressed)
    {
      SignalSince = Episode.GetElapsedGameTime();
      bSignalTurned = false;
    }
    else if (bLeft || bRight)
    {
      // Cancel after the lane change it announced...
      const double LaneChange = Report->GetLastSignalledLaneChange();
      const double Now = Episode.GetElapsedGameTime();
      const bool bLaneChangeDone = LaneChange >= SignalSince && Now - LaneChange >= SignalCancelAfterLaneChange;
      // ...or once the wheel comes back from a turn the way it points.
      const float Toward = bLeft ? -Steer : Steer;
      bSignalTurned = bSignalTurned || Toward > SignalTurnSteer;
      if (bLaneChangeDone || (bSignalTurned && FMath::Abs(Steer) < SignalCenteredSteer))
      {
        bLeft = false;
        bRight = false;
      }
    }
    if (bLeft != Lights.LeftBlinker || bRight != Lights.RightBlinker)
    {
      Lights.LeftBlinker = bLeft;
      Lights.RightBlinker = bRight;
      Car->SetVehicleLightState(Lights);
    }
  }

  /// B cycles the monitor's mode; a turn signal towards an occupied side is
  /// reported whatever the mode, so drives with and without it compare.
  void MonitorBlindSpot(APlayerController &Controller, float DeltaSeconds)
  {
    if (Pressed(Controller, EKeys::B, bBlindSpotKeyDown) | Wheel->WasPressed(EServerWheelAction::BlindSpot))
    {
      Settings.BlindSpot = NextServerWarningMode(Settings.BlindSpot);
      BlindSpot->SetMode(Settings.BlindSpot);
      Report->SetAidMode(BlindSpotAid, GetServerWarningModeName(Settings.BlindSpot));
      UE_LOG(LogCarlaServerPlayer, Log, TEXT("Server player: blind-spot monitor %s"), GetServerWarningModeName(Settings.BlindSpot));
    }
    const FVehicleLightState Lights = Vehicle->GetVehicleLightState();
    const int32 Started = BlindSpot->Tick(DeltaSeconds, Lights.LeftBlinker, Lights.RightBlinker);
    for (int32 Side = 0; Side < 2; ++Side)
    {
      if ((Started & (1 << Side)) == 0)
      {
        continue;
      }
      Report->AddInfo(TEXT("Signalled into occupied blind spot"),
          FString::Printf(TEXT("%s side, %s"), Side == FServerBlindSpot::Left ? TEXT("left") : TEXT("right"), DescribeWarning(Settings.BlindSpot)),
          BlindSpot->GetOccupant(Side));
    }
    Mirrors->SetBlindSpot(BlindSpot->GetIndicator(FServerBlindSpot::Left), BlindSpot->GetIndicator(FServerBlindSpot::Right));
  }

  /// P cycles the A-pillar monitor's mode; a pedestrian or cyclist going
  /// behind a pillar is reported whatever the mode.
  void MonitorPillars(APlayerController &Controller, float DeltaSeconds)
  {
    if (Pressed(Controller, EKeys::P, bPillarsKeyDown) | Wheel->WasPressed(EServerWheelAction::Pillars))
    {
      Settings.Pillars = NextServerWarningMode(Settings.Pillars);
      Pillars->SetMode(Settings.Pillars);
      Report->SetAidMode(PillarAid, GetServerWarningModeName(Settings.Pillars));
      UE_LOG(LogCarlaServerPlayer, Log, TEXT("Server player: A-pillar monitor %s"), GetServerWarningModeName(Settings.Pillars));
    }
    for (const FServerPillars::FHidden &Hidden : Pillars->Tick(DeltaSeconds))
    {
      Report->AddInfo(TEXT("Hidden by A-pillar"),
          FString::Printf(TEXT("%s pillar, %s"), Hidden.Side == FServerPillars::Left ? TEXT("left") : TEXT("right"), DescribeWarning(Settings.Pillars)),
          Hidden.Actor.Get());
    }
  }

  static const TCHAR *DescribeWarning(EServerWarningMode Mode)
  {
    switch (Mode)
    {
      case EServerWarningMode::Off:    return TEXT("not warned (monitor off)");
      case EServerWarningMode::Visual: return TEXT("warned: indicator");
      default:                         return TEXT("warned: indicator and chime");
    }
  }

  void ResetUpright(ACarlaWheeledVehicle &Car)
  {
    const FRotator Yaw(0.0f, Car.GetActorRotation().Yaw, 0.0f);
    const FVector Location = Car.GetActorLocation() + FVector(0.0f, 0.0f, 100.0f);
    Car.SetActorLocationAndRotation(Location, Yaw, false, nullptr, ETeleportType::ResetPhysics);
    if (UPrimitiveComponent *Root = Cast<UPrimitiveComponent>(Car.GetRootComponent()))
    {
      Root->SetPhysicsLinearVelocity(FVector::ZeroVector);
      Root->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
    }
    bReverse = false;
    Steer = 0.0f;
  }

  void DrawHud()
  {
    if (GEngine == nullptr)
    {
      return;
    }
    const float SpeedKmh = Vehicle->GetVehicleForwardSpeed() * 0.036f;
    const bool bRefused = FPlatformTime::Seconds() < GearRefusedUntil;
    HudColor = bRefused ? FColor::Orange : FColor::White;
    if (Mission && Mission->IsPlanning())
    {
      HudText.Reset(); // the planner draws its own
      HudSpeedMph = -1.0f;
      return;
    }
    // The speedometer and the limit (snapped to 5 mph, as speeding is judged).
    HudSpeedMph = FMath::Abs(SpeedKmh) / FServerDriveReport::KmhPerMph;
    const float LimitKmh = Report ? Report->GetSpeedLimitKmh() : -1.0f;
    HudLimitMph = LimitKmh > 0.0f ? FMath::RoundToFloat(LimitKmh / FServerDriveReport::KmhPerMph) : -1.0f;
    HudBlinkerLit[0] = Blinker && Blinker->IsLeftLit();
    HudBlinkerLit[1] = Blinker && Blinker->IsRightLit();
    const FVehicleLightState Lights = Vehicle->GetVehicleLightState();
    const TCHAR *SignalText = Lights.LeftBlinker ? TEXT("<< ") : (Lights.RightBlinker ? TEXT(" >>") : TEXT(" - "));
    HudText = FString::Printf(TEXT("gear %s%s [Q]    signal %s [Z/X]    camera: %s [C]    blind spot: %s [B]    A-pillars: %s [P]    reset [Backspace]    quit [Esc]    violations: %d"),
            bReverse ? TEXT("R") : TEXT("D"),
            bRefused ? TEXT(" (stop to shift)") : TEXT(""), SignalText, CameraModeName(CameraMode),
            GetServerWarningModeName(Settings.BlindSpot), GetServerWarningModeName(Settings.Pillars),
            Report ? Report->GetViolationCount() : 0);
    if (Wheel && Wheel->IsConnected())
    {
      // The last control pressed, to map buttons in Config/ServerWheel.json.
      const FString Last = Wheel->GetLastPressed();
      HudText += FString::Printf(TEXT("    wheel%s%s%s"), ForceFeedback && ForceFeedback->IsActive() ? TEXT(" (FFB)") : TEXT(""),
          Last.IsEmpty() ? TEXT("") : TEXT(": "), *Last);
    }
    // Outside the car the mirrors are not in view: the indicators go on the HUD.
    BlindSpotHud[0] = CameraMode != ECameraMode::Interior && BlindSpot ? BlindSpot->GetIndicator(FServerBlindSpot::Left) : 0.0f;
    BlindSpotHud[1] = CameraMode != ECameraMode::Interior && BlindSpot ? BlindSpot->GetIndicator(FServerBlindSpot::Right) : 0.0f;
    PillarHud[0] = CameraMode != ECameraMode::Interior && Pillars ? Pillars->GetIndicator(FServerPillars::Left) : 0.0f;
    PillarHud[1] = CameraMode != ECameraMode::Interior && Pillars ? Pillars->GetIndicator(FServerPillars::Right) : 0.0f;
  }

  /// Bottom centre of the driver's screen: the speed in mph (red above the
  /// limit) and, beside it, a US-style speed-limit sign.
  void DrawSpeedometer(UCanvas &Canvas, float ViewWidth)
  {
    UFont *Font = GEngine->GetLargeFont();
    const bool bOver = HudLimitMph > 0.0f && HudSpeedMph > HudLimitMph + SpeedometerOverMph;
    const float Height = Canvas.ClipY;
    const float CentreX = ViewWidth * 0.5f;

    const FString Speed = FString::Printf(TEXT("%.0f"), HudSpeedMph);
    float Width = 0.0f;
    float LineHeight = 0.0f;
    Canvas.StrLen(Font, Speed, Width, LineHeight);
    // Digits about 7% of the screen's height.
    const float SpeedScale = Height * 0.07f / FMath::Max(LineHeight, 1.0f);
    const float SpeedY = Height * 0.80f;
    Canvas.SetDrawColor(bOver ? FColor(255, 60, 40) : FColor::White);
    Canvas.DrawText(Font, Speed, CentreX - Width * SpeedScale * 0.5f, SpeedY, SpeedScale, SpeedScale);
    const float UnitScale = SpeedScale * 0.35f;
    float UnitWidth = 0.0f;
    float UnitHeight = 0.0f;
    Canvas.StrLen(Font, TEXT("MPH"), UnitWidth, UnitHeight);
    Canvas.DrawText(Font, TEXT("MPH"), CentreX - UnitWidth * UnitScale * 0.5f, SpeedY + LineHeight * SpeedScale * 0.92f, UnitScale, UnitScale);

    // Turn-signal arrows either side of the digits, spaced for three of
    // them so they do not move with the speed: dim, bright green when lit.
    float WideWidth = 0.0f;
    float WideHeight = 0.0f;
    Canvas.StrLen(Font, TEXT("188"), WideWidth, WideHeight);
    const float ArrowHeight = Height * 0.045f;
    const float ArrowLength = ArrowHeight * 1.1f;
    const float ArrowGap = WideWidth * SpeedScale * 0.5f + Height * 0.02f;
    const float ArrowY = SpeedY + LineHeight * SpeedScale * 0.5f;
    const auto DrawArrow = [&](bool bPointsLeft, bool bLit) {
      const float Direction = bPointsLeft ? -1.0f : 1.0f;
      const float Base = CentreX + Direction * ArrowGap;  // the tail's end
      const float Tip = Base + Direction * ArrowLength;
      const float Neck = Base + Direction * ArrowLength * 0.45f;
      const FLinearColor Color = bLit ? FLinearColor(0.1f, 1.0f, 0.25f, 1.0f) : FLinearColor(0.15f, 0.3f, 0.18f, 0.55f);
      FCanvasTriangleItem Head(FVector2D(Tip, ArrowY), FVector2D(Neck, ArrowY - ArrowHeight * 0.5f),
          FVector2D(Neck, ArrowY + ArrowHeight * 0.5f), GWhiteTexture);
      Head.SetColor(Color);
      Head.BlendMode = SE_BLEND_Translucent;
      Canvas.DrawItem(Head);
      const float Thickness = ArrowHeight * 0.38f;
      FCanvasTileItem Tail(FVector2D(FMath::Min(Base, Neck), ArrowY - Thickness * 0.5f), GWhiteTexture,
          FVector2D(FMath::Abs(Neck - Base), Thickness), Color);
      Tail.BlendMode = SE_BLEND_Translucent;
      Canvas.DrawItem(Tail);
    };
    DrawArrow(true, HudBlinkerLit[0]);
    DrawArrow(false, HudBlinkerLit[1]);

    if (HudLimitMph <= 0.0f)
    {
      return;
    }
    // The sign: a black-bordered white plate, SPEED / LIMIT over the number.
    const float SignWidth = Height * 0.075f;
    const float SignHeight = SignWidth * 1.25f;
    const float SignX = CentreX + ArrowGap + ArrowLength + Height * 0.03f;
    const float SignY = SpeedY + LineHeight * SpeedScale * 0.5f - SignHeight * 0.5f;
    const float Border = SignWidth * 0.06f;
    FCanvasTileItem Outer(FVector2D(SignX, SignY), GWhiteTexture, FVector2D(SignWidth, SignHeight), FLinearColor::Black);
    Canvas.DrawItem(Outer);
    FCanvasTileItem Inner(FVector2D(SignX + Border, SignY + Border), GWhiteTexture,
        FVector2D(SignWidth - 2.0f * Border, SignHeight - 2.0f * Border), FLinearColor::White);
    Canvas.DrawItem(Inner);
    Canvas.SetDrawColor(FColor::Black);
    const auto Centred = [&](const FString &Text, float Y, float Scale) {
      float TextWidth = 0.0f;
      float TextHeight = 0.0f;
      Canvas.StrLen(Font, Text, TextWidth, TextHeight);
      Canvas.DrawText(Font, Text, SignX + (SignWidth - TextWidth * Scale) * 0.5f, Y, Scale, Scale);
      return TextHeight * Scale;
    };
    // The words span 70% of the plate; the number fills what is left below.
    float WordWidth = 0.0f;
    float WordHeight = 0.0f;
    Canvas.StrLen(Font, TEXT("SPEED"), WordWidth, WordHeight);
    const float WordScale = SignWidth * 0.7f / FMath::Max(WordWidth, 1.0f);
    float Y = SignY + Border * 2.0f;
    Y += Centred(TEXT("SPEED"), Y, WordScale) * 0.95f;
    Y += Centred(TEXT("LIMIT"), Y, WordScale);
    const FString Number = FString::Printf(TEXT("%.0f"), HudLimitMph);
    float NumberWidth = 0.0f;
    float NumberHeight = 0.0f;
    Canvas.StrLen(Font, Number, NumberWidth, NumberHeight);
    const float Room = SignY + SignHeight - Border * 2.0f - Y;
    const float NumberScale = FMath::Min(Room / FMath::Max(NumberHeight, 1.0f), SignWidth * 0.8f / FMath::Max(NumberWidth, 1.0f));
    Centred(Number, Y + (Room - NumberHeight * NumberScale) * 0.5f, NumberScale);
  }

  /// Top-right frame counter: rendered frames per second, averaged over
  /// FpsWindowSeconds of wall-clock time, and the mean frame time.
  void DrawFps(UCanvas *Canvas, APlayerController *ViewController)
  {
    if (Canvas == nullptr || GEngine == nullptr)
    {
      return;
    }
    // Called once per player's view: draw on the driver's (the centre
    // screen in surround). The canvas origin is that view's corner.
    if (ViewController != nullptr && ViewController != UGameplayStatics::GetPlayerController(Episode.GetWorld(), 0))
    {
      return;
    }
    const float ViewWidth = bSurroundReady ? SurroundCentreWidth : Canvas->ClipX;
    float MissionY = 16.0f;
    if (!HudText.IsEmpty())
    {
      Canvas->SetDrawColor(HudColor);
      Canvas->DrawText(GEngine->GetMediumFont(), HudText, 24.0f, 16.0f, 1.3f, 1.3f);
      float LineWidth = 0.0f;
      float LineHeight = 0.0f;
      Canvas->StrLen(GEngine->GetMediumFont(), HudText, LineWidth, LineHeight);
      MissionY += LineHeight * 1.3f + 6.0f;
    }
    if (Mission && Vehicle.IsValid())
    {
      Mission->Draw(*Canvas, ViewWidth, MissionY);
    }
    for (int32 Index = 0; Index < 4; ++Index)
    {
      const int32 Side = Index % 2;
      const bool bPillar = Index >= 2;
      const float Level = bPillar ? PillarHud[Side] : BlindSpotHud[Side];
      if (Level <= 0.0f)
      {
        continue;
      }
      const FString Text = bPillar ? (Side == FServerPillars::Left ? TEXT("<< A-PILLAR") : TEXT("A-PILLAR >>"))
          : (Side == FServerBlindSpot::Left ? TEXT("<< BLIND SPOT") : TEXT("BLIND SPOT >>"));
      float TextWidth = 0.0f;
      float TextHeight = 0.0f;
      Canvas->StrLen(GEngine->GetMediumFont(), Text, TextWidth, TextHeight);
      const float Scale = 2.0f;
      const float X = Side == FServerBlindSpot::Left ? 24.0f : ViewWidth - TextWidth * Scale - 24.0f;
      Canvas->SetDrawColor(FLinearColor(1.0f, 0.42f, 0.02f, Level).ToFColor(true));
      Canvas->DrawText(GEngine->GetMediumFont(), Text, X, Canvas->ClipY * 0.5f + (bPillar ? TextHeight * Scale * 1.3f : 0.0f), Scale, Scale);
    }
    if (HudSpeedMph >= 0.0f)
    {
      DrawSpeedometer(*Canvas, ViewWidth);
    }
    const double Now = FPlatformTime::Seconds();
    if (Now - QuitPromptTime <= QuitConfirmSeconds)
    {
      const FString Text = TEXT("Press Esc again to quit");
      float TextWidth = 0.0f;
      float TextHeight = 0.0f;
      Canvas->StrLen(GEngine->GetMediumFont(), Text, TextWidth, TextHeight);
      const float Scale = 2.5f;
      Canvas->SetDrawColor(FColor::White);
      Canvas->DrawText(GEngine->GetMediumFont(), Text, (ViewWidth - TextWidth * Scale) * 0.5f,
          Canvas->ClipY * 0.35f, Scale, Scale);
    }
    if (FpsWindowStart == 0.0)
    {
      FpsWindowStart = Now;
    }
    ++FpsFrames;
    // Where the frame goes, as "stat unit" reports it (whose own readout
    // falls off the monitors in surround): the game thread (simulation,
    // traffic, physics), the render thread (draw calls), the GPU.
    TimingSums[0] += FPlatformTime::ToMilliseconds(GGameThreadTime);
    TimingSums[1] += FPlatformTime::ToMilliseconds(GRenderThreadTime);
    TimingSums[2] += FPlatformTime::ToMilliseconds(RHIGetGPUFrameCycles());
    const double Elapsed = Now - FpsWindowStart;
    if (Elapsed >= FpsWindowSeconds)
    {
      Fps = FpsFrames / Elapsed;
      for (int32 Index = 0; Index < 3; ++Index)
      {
        Timings[Index] = TimingSums[Index] / FpsFrames;
        TimingSums[Index] = 0.0;
      }
      FpsFrames = 0;
      FpsWindowStart = Now;
    }
    if (Fps <= 0.0)
    {
      return;
    }
    UFont *Font = GEngine->GetMediumFont();
    const FString Text = FString::Printf(TEXT("%.0f fps  %.1f ms"), Fps, 1000.0 / Fps);
    float Width = 0.0f;
    float Height = 0.0f;
    Canvas->StrLen(Font, Text, Width, Height);
    const float Scale = 1.5f;
    const FLinearColor Color = Fps >= 55.0 ? FLinearColor::Green : (Fps >= 30.0 ? FLinearColor::Yellow : FLinearColor::Red);
    Canvas->SetDrawColor(Color.ToFColor(true));
    Canvas->DrawText(Font, Text, ViewWidth - Width * Scale - 24.0f, 16.0f, Scale, Scale);

    // The upscaler, and the resolution the scene renders at before it.
    const FUpscalerMode &Mode = GetMode(Upscaler);
    FString UpscalerText = FString::Printf(TEXT("%s  [F9]"), Mode.Name);
    if (Mode.ScreenPercentage > 0.0f)
    {
      const float Fraction = Mode.ScreenPercentage / 100.0f;
      UpscalerText = FString::Printf(TEXT("%s %dx%d  [F9]"), Mode.Name,
          FMath::RoundToInt(ViewWidth * Fraction), FMath::RoundToInt(Canvas->ClipY * Fraction));
    }
    const float SmallScale = 1.1f;
    Canvas->StrLen(Font, UpscalerText, Width, Height);
    Canvas->SetDrawColor(FColor::White);
    Canvas->DrawText(Font, UpscalerText, ViewWidth - Width * SmallScale - 24.0f, 16.0f + Height * Scale + 4.0f,
        SmallScale, SmallScale);

    // The thread times, the slowest (what limits the frame rate) in yellow.
    const TCHAR *Names[3] = {TEXT("game"), TEXT("draw"), TEXT("gpu")};
    int32 Slowest = 0;
    for (int32 Index = 1; Index < 3; ++Index)
    {
      Slowest = Timings[Index] > Timings[Slowest] ? Index : Slowest;
    }
    float X = ViewWidth - 24.0f;
    const float Y = 16.0f + Height * Scale + 4.0f + Height * SmallScale + 2.0f;
    for (int32 Index = 2; Index >= 0; --Index)
    {
      const FString Part = FString::Printf(TEXT("%s %.1f  "), Names[Index], Timings[Index]);
      Canvas->StrLen(Font, Part, Width, Height);
      X -= Width * SmallScale;
      Canvas->SetDrawColor(Index == Slowest ? FColor::Yellow : FColor::White);
      Canvas->DrawText(Font, Part, X, Y, SmallScale, SmallScale);
    }
  }

  UCarlaEpisode &Episode;
  FServerPlayerSettings Settings;
  std::mt19937 RandomEngine;

  TWeakObjectPtr<ACarlaWheeledVehicle> Vehicle;
  TWeakObjectPtr<USpringArmComponent> Arm;
  TWeakObjectPtr<UCameraComponent> Camera;
  FVector BoundsOrigin = FVector::ZeroVector;
  FVector BoundsExtent = FVector(230.0f, 100.0f, 75.0f);

  ECameraMode CameraMode = ECameraMode::Chase;
  float Steer = 0.0f;
  bool bReverse = false;
  bool bCameraKeyDown = false;
  bool bPitchUpKeyDown = false;
  bool bPitchDownKeyDown = false;
  bool bCameraPadDown = false;
  bool bResetKeyDown = false;
  bool bQuitKeyDown = false;
  /// When Esc was first pressed; a second press within QuitConfirmSeconds quits.
  double QuitPromptTime = -1000.0;
  static constexpr double QuitConfirmSeconds = 3.0;
  bool bResetPadDown = false;
  bool bGearKeyDown = false;
  bool bGearPadDown = false;
  double GearRefusedUntil = 0.0;
  bool bSpawned = false;
  FString BlueprintId;
  std::unique_ptr<FServerDriveReport> Report;
  TUniquePtr<FServerMission> Mission;
  bool bMissionKeyDown = false;
  bool bRestartKeyDown = false;
  TUniquePtr<FServerMirrors> Mirrors;
  TUniquePtr<FServerBlindSpot> BlindSpot;
  bool bBlindSpotKeyDown = false;
  float BlindSpotHud[2] = {0.0f, 0.0f};
  TUniquePtr<FServerPillars> Pillars;
  TUniquePtr<FServerWheel> Wheel;
  TUniquePtr<FServerForceFeedback> ForceFeedback;
  bool bPillarsKeyDown = false;
  float PillarHud[2] = {0.0f, 0.0f};
  /// The drive report's names for the driver aids.
  const FString BlindSpotAid = TEXT("Blind-spot monitor");
  const FString PillarAid = TEXT("A-pillar monitor");
  bool bLeftKeyDown = false;
  bool bLeftPadDown = false;
  bool bRightKeyDown = false;
  bool bRightPadDown = false;
  double SignalSince = 0.0;
  bool bSignalTurned = false;
  EUpscaler Upscaler = EUpscaler::Tsr;
  bool bUpscalerKeyDown = false;
  FDelegateHandle FpsDrawHandle;
  FString HudText;
  FColor HudColor = FColor::White;
  /// The speedometer; -1 hides it (the mission planner is open).
  float HudSpeedMph = -1.0f;
  float HudLimitMph = -1.0f;
  /// The left and right turn-signal indicators, lit this frame.
  bool HudBlinkerLit[2] = {false, false};
  TUniquePtr<FServerBlinker> Blinker;
  bool bSurroundReady = false;
  bool bSurroundFailed = false;
  float SurroundFovs[SurroundScreens] = {60.0f, 60.0f, 60.0f};
  float SurroundYaws[SurroundScreens] = {-60.0f, 0.0f, 60.0f};
  float SurroundCentreWidth = 0.0f;
  TWeakObjectPtr<ACameraActor> SideCameras[2];
  double FpsWindowStart = 0.0;
  int32 FpsFrames = 0;
  double Fps = 0.0;
  double TimingSums[3] = {0.0, 0.0, 0.0};
  double Timings[3] = {0.0, 0.0, 0.0};
  bool bFailed = false;
};

FServerPlayer::FServerPlayer(UCarlaEpisode &Episode, const FServerPlayerSettings &Settings)
  : Impl(std::make_unique<FImpl>(Episode, Settings)) {}

FServerPlayer::~FServerPlayer() = default;

void FServerPlayer::Tick(float DeltaSeconds)
{
  Impl->Tick(DeltaSeconds);
}
