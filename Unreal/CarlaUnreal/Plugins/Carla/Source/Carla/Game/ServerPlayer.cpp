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
#include "Carla/Vehicle/CarlaWheeledVehicle.h"
#include "Carla/Vehicle/VehicleControl.h"
#include "Carla/Vehicle/VehicleInputPriority.h"

#include <util/ue-header-guard-begin.h>
#include "Camera/CameraComponent.h"
#include "Camera/PlayerCameraManager.h"
#include "Components/PrimitiveComponent.h"
#include "Debug/DebugDrawService.h"
#include "Engine/Canvas.h"
#include "Engine/Font.h"
#include "HAL/IConsoleManager.h"
#include "Engine/Engine.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/SpringArmComponent.h"
#include "InputCoreTypes.h"
#include "Kismet/GameplayStatics.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
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
  if (!FParse::Value(CommandLine, TEXT("-server-dlss="), Settings.Dlss) &&
      FParse::Param(CommandLine, TEXT("server-dlss")))
  {
    Settings.Dlss = TEXT("quality");
  }
  return Settings;
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
  constexpr uint64 HudMessageKey = 0x5E5E9A1E5ull;
  /// The FPS counter averages frames over this window.
  constexpr double FpsWindowSeconds = 0.5;

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
  }

  ~FImpl()
  {
    UDebugDrawService::Unregister(FpsDrawHandle);
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
      if (!Spawn())
      {
        bFailed = true;
        return;
      }
      AttachCamera();
      bSpawned = true;
    }

    TakeView(*Controller);
    if (Pressed(*Controller, EKeys::F9, bUpscalerKeyDown))
    {
      Upscaler = static_cast<EUpscaler>((static_cast<uint8>(Upscaler) + 1) % static_cast<uint8>(EUpscaler::Count));
      ApplyUpscaler();
    }
    Drive(*Controller, DeltaSeconds);
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
    NewCamera->RegisterComponent();
    Car->AddInstanceComponent(NewCamera);

    Arm = NewArm;
    Camera = NewCamera;
    ApplyCameraMode();
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
        SpringArm->SetRelativeLocation(FVector(
            BoundsOrigin.X - 0.08f * BoundsExtent.X,
            BoundsOrigin.Y - 0.36f * BoundsExtent.Y,
            BoundsOrigin.Z + 0.5f * BoundsExtent.Z));
        SpringArm->SetRelativeRotation(FRotator(-4.0f, 0.0f, 0.0f));
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

  void DrawHud() const
  {
    if (GEngine == nullptr)
    {
      return;
    }
    const float SpeedKmh = Vehicle->GetVehicleForwardSpeed() * 0.036f;
    const bool bRefused = FPlatformTime::Seconds() < GearRefusedUntil;
    GEngine->AddOnScreenDebugMessage(HudMessageKey, 1.0f, bRefused ? FColor::Orange : FColor::White,
        FString::Printf(TEXT("%3.0f km/h  %s%s    gear [Q]    camera: %s [C]    reset [Backspace]"),
            FMath::Abs(SpeedKmh), bReverse ? TEXT("R") : TEXT("D"),
            bRefused ? TEXT(" (stop to shift)") : TEXT(""), CameraModeName(CameraMode)),
        false, FVector2D(1.5f, 1.5f));
  }

  /// Top-right frame counter: rendered frames per second, averaged over
  /// FpsWindowSeconds of wall-clock time, and the mean frame time.
  void DrawFps(UCanvas *Canvas, APlayerController *)
  {
    if (Canvas == nullptr || GEngine == nullptr)
    {
      return;
    }
    const double Now = FPlatformTime::Seconds();
    if (FpsWindowStart == 0.0)
    {
      FpsWindowStart = Now;
    }
    ++FpsFrames;
    const double Elapsed = Now - FpsWindowStart;
    if (Elapsed >= FpsWindowSeconds)
    {
      Fps = FpsFrames / Elapsed;
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
    Canvas->DrawText(Font, Text, Canvas->ClipX - Width * Scale - 24.0f, 16.0f, Scale, Scale);

    // The upscaler, and the resolution the scene renders at before it.
    const FUpscalerMode &Mode = GetMode(Upscaler);
    FString UpscalerText = FString::Printf(TEXT("%s  [F9]"), Mode.Name);
    if (Mode.ScreenPercentage > 0.0f)
    {
      const float Fraction = Mode.ScreenPercentage / 100.0f;
      UpscalerText = FString::Printf(TEXT("%s %dx%d  [F9]"), Mode.Name,
          FMath::RoundToInt(Canvas->ClipX * Fraction), FMath::RoundToInt(Canvas->ClipY * Fraction));
    }
    const float SmallScale = 1.1f;
    Canvas->StrLen(Font, UpscalerText, Width, Height);
    Canvas->SetDrawColor(FColor::White);
    Canvas->DrawText(Font, UpscalerText, Canvas->ClipX - Width * SmallScale - 24.0f, 16.0f + Height * Scale + 4.0f,
        SmallScale, SmallScale);
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
  bool bCameraPadDown = false;
  bool bResetKeyDown = false;
  bool bResetPadDown = false;
  bool bGearKeyDown = false;
  bool bGearPadDown = false;
  double GearRefusedUntil = 0.0;
  bool bSpawned = false;
  EUpscaler Upscaler = EUpscaler::Tsr;
  bool bUpscalerKeyDown = false;
  FDelegateHandle FpsDrawHandle;
  double FpsWindowStart = 0.0;
  int32 FpsFrames = 0;
  double Fps = 0.0;
  bool bFailed = false;
};

FServerPlayer::FServerPlayer(UCarlaEpisode &Episode, const FServerPlayerSettings &Settings)
  : Impl(std::make_unique<FImpl>(Episode, Settings)) {}

FServerPlayer::~FServerPlayer() = default;

void FServerPlayer::Tick(float DeltaSeconds)
{
  Impl->Tick(DeltaSeconds);
}
