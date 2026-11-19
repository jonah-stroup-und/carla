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
#include "Carla/Game/ServerDriveReport.h"
#include "Carla/Game/ServerMirrors.h"
#include "Carla/Vehicle/CarlaWheeledVehicle.h"
#include "Carla/Vehicle/VehicleControl.h"
#include "Carla/Vehicle/VehicleInputPriority.h"

#include <util/ue-header-guard-begin.h>
#include "Camera/CameraActor.h"
#include "Camera/CameraComponent.h"
#include "Camera/PlayerCameraManager.h"
#include "Components/PrimitiveComponent.h"
#include "Debug/DebugDrawService.h"
#include "Engine/Canvas.h"
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
    if (Settings.Camera.Equals(TEXT("interior"), ESearchCase::IgnoreCase))
    {
      CameraMode = ECameraMode::Interior;
    }
    else if (Settings.Camera.Equals(TEXT("far"), ESearchCase::IgnoreCase))
    {
      CameraMode = ECameraMode::FarChase;
    }
  }

  ~FImpl()
  {
    Report.reset();
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
    if (!Vehicle.IsValid())
    {
      if (bSpawned)
      {
        UE_LOG(LogCarlaServerPlayer, Warning, TEXT("Server player: the vehicle was destroyed; spawning a new one"));
      }
      Report.reset(); // that drive is over
      Mirrors.Reset();
      if (!Spawn())
      {
        bFailed = true;
        return;
      }
      AttachCamera();
      Report = std::make_unique<FServerDriveReport>(Episode, *Vehicle, BlueprintId);
      Mirrors = MakeUnique<FServerMirrors>(*Vehicle, BlueprintId);
      bSpawned = true;
    }

    TakeView(*Controller);
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
    Drive(*Controller, DeltaSeconds);
    Signal(*Controller);
    Report->Tick();
    // The mirrors are aimed from the driver's eye, and only render while
    // the driver can see them.
    Mirrors->Tick(Vehicle->GetActorTransform().TransformPosition(GetEyeLocation()),
        CameraMode == ECameraMode::Interior);
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

  /// The driver's eyes, in actor space: left seat (UE's +Y is right), a
  /// little behind the middle of the car, at about three quarters of its
  /// height.
  FVector GetEyeLocation() const
  {
    return FVector(
        BoundsOrigin.X - 0.08f * BoundsExtent.X,
        BoundsOrigin.Y - 0.36f * BoundsExtent.Y,
        BoundsOrigin.Z + 0.5f * BoundsExtent.Z);
  }

  void TakeView(APlayerController &Controller)
  {
    ACarlaWheeledVehicle *Car = Vehicle.Get();
    Controller.bAutoManageActiveCameraTarget = false;
    if (Controller.GetViewTarget() != Car)
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
    if (Pressed(Controller, EKeys::C, bCameraKeyDown) | Pressed(Controller, EKeys::Gamepad_FaceButton_Top, bCameraPadDown))
    {
      CameraMode = static_cast<ECameraMode>((static_cast<uint8>(CameraMode) + 1) % static_cast<uint8>(ECameraMode::Count));
      ApplyCameraMode();
    }
    if (Pressed(Controller, EKeys::BackSpace, bResetKeyDown) | Pressed(Controller, EKeys::Gamepad_Special_Left, bResetPadDown))
    {
      ResetUpright(*Car);
    }

    const auto Down = [&Controller](const FKey &A, const FKey &B) {
      return Controller.IsInputKeyDown(A) || Controller.IsInputKeyDown(B);
    };
    const float Throttle = FMath::Max(Down(EKeys::W, EKeys::Up) ? 1.0f : 0.0f,
        Analog(Controller, EKeys::Gamepad_RightTriggerAxis));
    const float Brake = FMath::Max(Down(EKeys::S, EKeys::Down) ? 1.0f : 0.0f,
        Analog(Controller, EKeys::Gamepad_LeftTriggerAxis));

    // Q toggles drive/reverse, only when nearly stopped; W drives in that gear.
    const float Speed = Car->GetVehicleForwardSpeed(); // cm/s, negative backwards
    if (Pressed(Controller, EKeys::Q, bGearKeyDown) | Pressed(Controller, EKeys::Gamepad_FaceButton_Left, bGearPadDown))
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
    if (Stick != 0.0f)
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
    Control.bHandBrake = Controller.IsInputKeyDown(EKeys::SpaceBar) || Controller.IsInputKeyDown(EKeys::Gamepad_FaceButton_Bottom);
    Control.bReverse = bReverse;
    Car->ApplyVehicleControl(Control, EVehicleInputPriority::User);
  }

  /// Z/X (or the d-pad) toggle the left/right turn signal; a signal cancels
  /// itself after a lane change or a turn, like a real one.
  void Signal(APlayerController &Controller)
  {
    ACarlaWheeledVehicle *Car = Vehicle.Get();
    FVehicleLightState Lights = Car->GetVehicleLightState();
    const bool bLeftPressed = Pressed(Controller, EKeys::Z, bLeftKeyDown) | Pressed(Controller, EKeys::Gamepad_DPad_Left, bLeftPadDown);
    const bool bRightPressed = Pressed(Controller, EKeys::X, bRightKeyDown) | Pressed(Controller, EKeys::Gamepad_DPad_Right, bRightPadDown);
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
    const FVehicleLightState Lights = Vehicle->GetVehicleLightState();
    const TCHAR *SignalText = Lights.LeftBlinker ? TEXT("<< ") : (Lights.RightBlinker ? TEXT(" >>") : TEXT(" - "));
    HudText = FString::Printf(TEXT("%3.0f km/h  %s%s    signal %s [Z/X]    gear [Q]    camera: %s [C]    reset [Backspace]    violations: %d"),
            FMath::Abs(SpeedKmh), bReverse ? TEXT("R") : TEXT("D"),
            bRefused ? TEXT(" (stop to shift)") : TEXT(""), SignalText, CameraModeName(CameraMode),
            Report ? Report->GetViolationCount() : 0);
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
    if (!HudText.IsEmpty())
    {
      Canvas->SetDrawColor(HudColor);
      Canvas->DrawText(GEngine->GetMediumFont(), HudText, 24.0f, 16.0f, 1.3f, 1.3f);
    }
    const double Now = FPlatformTime::Seconds();
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
  bool bResetPadDown = false;
  bool bGearKeyDown = false;
  bool bGearPadDown = false;
  double GearRefusedUntil = 0.0;
  bool bSpawned = false;
  FString BlueprintId;
  std::unique_ptr<FServerDriveReport> Report;
  TUniquePtr<FServerMirrors> Mirrors;
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
