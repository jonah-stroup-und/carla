// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Game/ServerMirrors.h"
#include "Carla.h"

#include "Carla/Vehicle/CarlaWheeledVehicle.h"

#include <util/ue-header-guard-begin.h>
#include "Components/SceneCaptureComponent2D.h"
#include "Components/SkeletalMeshComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Dom/JsonObject.h"
#include "Engine/StaticMesh.h"
#include "Engine/TextureRenderTarget2D.h"
#include "HAL/FileManager.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "UObject/StrongObjectPtr.h"
#include <util/ue-header-guard-end.h>

DEFINE_LOG_CATEGORY_STATIC(LogCarlaServerMirrors, Log, All);

namespace
{
  const TCHAR *MirrorMaterialPath = TEXT("/Carla/Mirrors/M_ServerMirror.M_ServerMirror");
  const TCHAR *PlaneMeshPath = TEXT("/Engine/BasicShapes/Plane.Plane");
  /// The engine plane is 100 cm square, facing +Z.
  constexpr float PlaneSize = 100.0f;
  /// How often (wall-clock seconds) the config file is checked for changes.
  constexpr double ReloadCheckSeconds = 0.5;

  struct FMirrorSettings
  {
    FString Name;
    FString Component;
    FVector Center = FVector::ZeroVector;
    FVector2D Size = FVector2D(15.0, 10.0);
    FVector Normal = FVector(-1.0, 0.0, 0.0);
    FVector2D Aim = FVector2D::ZeroVector;
    float Fov = 30.0f;
    int32 Resolution = 512;
    float Offset = 0.3f;
    float FlipU = 1.0f;
    float FlipV = 0.0f;
    float SwapUV = 0.0f;
    float Brightness = 0.9f;
    float Exposure = 0.0f;
    float ViewDistance = 15000.0f;
    float LodFactor = 2.0f;
    bool bLumen = true;
    /// The blind-spot indicator this mirror shows: 0 left, 1 right, -1 none.
    int32 BlindSpotSide = -1;
    FVector Icon = FVector(0.12, 0.2, 0.12);
  };

  bool ReadVector(const TSharedPtr<FJsonObject> &Object, const TCHAR *Field, FVector &Out)
  {
    const TArray<TSharedPtr<FJsonValue>> *Values = nullptr;
    if (!Object->TryGetArrayField(Field, Values) || Values->Num() != 3)
    {
      return false;
    }
    Out = FVector((*Values)[0]->AsNumber(), (*Values)[1]->AsNumber(), (*Values)[2]->AsNumber());
    return true;
  }

  bool ReadVector2D(const TSharedPtr<FJsonObject> &Object, const TCHAR *Field, FVector2D &Out)
  {
    const TArray<TSharedPtr<FJsonValue>> *Values = nullptr;
    if (!Object->TryGetArrayField(Field, Values) || Values->Num() != 2)
    {
      return false;
    }
    Out = FVector2D((*Values)[0]->AsNumber(), (*Values)[1]->AsNumber());
    return true;
  }

  void ReadFloat(const TSharedPtr<FJsonObject> &Object, const TCHAR *Field, float &Out)
  {
    double Value = 0.0;
    if (Object->TryGetNumberField(Field, Value))
    {
      Out = static_cast<float>(Value);
    }
  }
}

class FServerMirrors::FImpl
{
public:

  FImpl(ACarlaWheeledVehicle &InVehicle, const FString &InBlueprintId)
    : Vehicle(&InVehicle),
      BlueprintId(InBlueprintId),
      ConfigPath(GetConfigPath())
  {
    Material.Reset(LoadObject<UMaterialInterface>(nullptr, MirrorMaterialPath));
    Plane.Reset(LoadObject<UStaticMesh>(nullptr, PlaneMeshPath));
    if (!Material.IsValid() || !Plane.IsValid())
    {
      UE_LOG(LogCarlaServerMirrors, Error, TEXT("Mirrors: cannot load %s or %s; no mirrors"), MirrorMaterialPath, PlaneMeshPath);
      return;
    }
    Reload();
  }

  ~FImpl()
  {
    DestroyMirrors();
  }

  void Tick(const FVector &EyeLocation, bool bCapture)
  {
    const double Now = FPlatformTime::Seconds();
    if (Now - LastReloadCheck >= ReloadCheckSeconds)
    {
      LastReloadCheck = Now;
      const FDateTime Stamp = IFileManager::Get().GetTimeStamp(*ConfigPath);
      if (Stamp != ConfigStamp)
      {
        Reload();
      }
    }

    ACarlaWheeledVehicle *Car = Vehicle.Get();
    if (Car == nullptr)
    {
      return;
    }
    const FVector Up = Car->GetActorUpVector();
    // One mirror renders per frame, in turn: each is a whole extra scene
    // render, and a mirror refreshing at a third of the frame rate still
    // looks live, where three every frame cost a third of the frame rate.
    NextCapture = Mirrors.Num() > 0 ? (NextCapture + 1) % Mirrors.Num() : 0;
    for (int32 Index = 0; Index < Mirrors.Num(); ++Index)
    {
      FMirror &Mirror = Mirrors[Index];
      USceneCaptureComponent2D *Capture = Mirror.Capture.Get();
      USceneComponent *Parent = Mirror.Parent.Get();
      if (Capture == nullptr || Parent == nullptr)
      {
        continue;
      }
      const FTransform MeshTransform = Parent->GetComponentTransform();
      Capture->bCaptureEveryFrame = bCapture && (bEveryFrame || Index == NextCapture);
      if (!Capture->bCaptureEveryFrame)
      {
        continue;
      }
      // The driver's line of sight to the glass, reflected off it.
      const FVector Center = MeshTransform.TransformPosition(Mirror.Settings.Center);
      const FVector Normal = MeshTransform.TransformVectorNoScale(Mirror.AimNormal);
      const FVector Sight = (Center - EyeLocation).GetSafeNormal();
      const FVector Reflected = (Sight - 2.0f * FVector::DotProduct(Sight, Normal) * Normal).GetSafeNormal();
      Capture->SetWorldLocationAndRotation(
          Center + Reflected * 3.0f, FRotationMatrix::MakeFromXZ(Reflected, Up).Rotator());
    }
  }

  void SetBlindSpot(float Left, float Right)
  {
    BlindSpotLevels[0] = Left;
    BlindSpotLevels[1] = Right;
    for (FMirror &Mirror : Mirrors)
    {
      const int32 Side = Mirror.Settings.BlindSpotSide;
      UMaterialInstanceDynamic *Instance = Mirror.Instance.Get();
      if (Side < 0 || Instance == nullptr || Mirror.Warning == BlindSpotLevels[Side])
      {
        continue;
      }
      Mirror.Warning = BlindSpotLevels[Side];
      Instance->SetScalarParameterValue(TEXT("Warning"), Mirror.Warning);
    }
  }

private:

  struct FMirror
  {
    FMirrorSettings Settings;
    FVector AimNormal = FVector(-1.0, 0.0, 0.0);
    TWeakObjectPtr<USceneComponent> Parent;
    TWeakObjectPtr<UStaticMeshComponent> Panel;
    TWeakObjectPtr<USceneCaptureComponent2D> Capture;
    TStrongObjectPtr<UTextureRenderTarget2D> Target;
    TStrongObjectPtr<UMaterialInstanceDynamic> Instance;
    float Warning = 0.0f;
  };

  void Reload()
  {
    ConfigStamp = IFileManager::Get().GetTimeStamp(*ConfigPath);
    TArray<FMirrorSettings> Settings;
    if (!ReadConfig(Settings))
    {
      return; // keep the mirrors we have
    }
    DestroyMirrors();
    for (const FMirrorSettings &Each : Settings)
    {
      CreateMirror(Each);
    }
    // No mirror shows the others' panels (the rear-view one would see the
    // backs of the side ones' otherwise).
    for (FMirror &Mirror : Mirrors)
    {
      for (FMirror &Other : Mirrors)
      {
        if (Mirror.Capture.IsValid() && Other.Panel.IsValid())
        {
          Mirror.Capture->HideComponent(Other.Panel.Get());
        }
      }
    }
    UE_LOG(LogCarlaServerMirrors, Log, TEXT("Mirrors: %d for %s from %s"), Mirrors.Num(), *BlueprintId, *ConfigPath);
  }

  bool ReadConfig(TArray<FMirrorSettings> &Out)
  {
    FString Text;
    if (!FFileHelper::LoadFileToString(Text, *ConfigPath))
    {
      UE_LOG(LogCarlaServerMirrors, Warning, TEXT("Mirrors: no config at %s; no mirrors"), *ConfigPath);
      return false;
    }
    TSharedPtr<FJsonObject> Root;
    if (!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text), Root) || !Root.IsValid())
    {
      UE_LOG(LogCarlaServerMirrors, Warning, TEXT("Mirrors: %s is not valid JSON; keeping the current mirrors"), *ConfigPath);
      return false;
    }
    const TSharedPtr<FJsonObject> *Vehicle = nullptr;
    const TArray<TSharedPtr<FJsonValue>> *Entries = nullptr;
    Eye.Reset();
    MirrorEye.Reset();
    Mesh.Reset();
    bEveryFrame = false;
    if (Root->TryGetObjectField(BlueprintId, Vehicle))
    {
      FVector Value;
      if (ReadVector(*Vehicle, TEXT("eye"), Value))
      {
        Eye = Value;
        UE_LOG(LogCarlaServerMirrors, Log, TEXT("Mirrors: driver's eye at %s"), *Value.ToString());
      }
      if (ReadVector(*Vehicle, TEXT("mirror_eye"), Value))
      {
        MirrorEye = Value;
      }
      (*Vehicle)->TryGetStringField(TEXT("mesh"), Mesh);
      (*Vehicle)->TryGetBoolField(TEXT("mirrors_every_frame"), bEveryFrame);
    }
    if (Vehicle == nullptr || !(*Vehicle)->TryGetArrayField(TEXT("mirrors"), Entries))
    {
      UE_LOG(LogCarlaServerMirrors, Log, TEXT("Mirrors: %s has no entry for %s; no mirrors"), *ConfigPath, *BlueprintId);
      Out.Reset();
      return true;
    }
    for (const TSharedPtr<FJsonValue> &Entry : *Entries)
    {
      const TSharedPtr<FJsonObject> *Object = nullptr;
      if (!Entry->TryGetObject(Object))
      {
        continue;
      }
      FMirrorSettings Mirror;
      (*Object)->TryGetStringField(TEXT("name"), Mirror.Name);
      (*Object)->TryGetStringField(TEXT("component"), Mirror.Component);
      if (!ReadVector(*Object, TEXT("center"), Mirror.Center))
      {
        UE_LOG(LogCarlaServerMirrors, Warning, TEXT("Mirrors: '%s' has no center; skipped"), *Mirror.Name);
        continue;
      }
      ReadVector2D(*Object, TEXT("size"), Mirror.Size);
      ReadVector(*Object, TEXT("normal"), Mirror.Normal);
      ReadVector2D(*Object, TEXT("aim"), Mirror.Aim);
      ReadFloat(*Object, TEXT("fov"), Mirror.Fov);
      float Resolution = static_cast<float>(Mirror.Resolution);
      ReadFloat(*Object, TEXT("resolution"), Resolution);
      Mirror.Resolution = FMath::Clamp(FMath::RoundToInt(Resolution), 64, 2048);
      ReadFloat(*Object, TEXT("offset"), Mirror.Offset);
      ReadFloat(*Object, TEXT("flip_u"), Mirror.FlipU);
      ReadFloat(*Object, TEXT("flip_v"), Mirror.FlipV);
      ReadFloat(*Object, TEXT("swap_uv"), Mirror.SwapUV);
      ReadFloat(*Object, TEXT("brightness"), Mirror.Brightness);
      ReadFloat(*Object, TEXT("exposure"), Mirror.Exposure);
      ReadFloat(*Object, TEXT("view_distance"), Mirror.ViewDistance);
      ReadFloat(*Object, TEXT("lod_factor"), Mirror.LodFactor);
      (*Object)->TryGetBoolField(TEXT("lumen"), Mirror.bLumen);
      FString BlindSpot;
      if ((*Object)->TryGetStringField(TEXT("blind_spot"), BlindSpot))
      {
        Mirror.BlindSpotSide = BlindSpot.Equals(TEXT("left"), ESearchCase::IgnoreCase) ? 0
            : (BlindSpot.Equals(TEXT("right"), ESearchCase::IgnoreCase) ? 1 : -1);
      }
      ReadVector(*Object, TEXT("icon"), Mirror.Icon);
      Out.Add(Mirror);
    }
    return true;
  }

  void CreateMirror(const FMirrorSettings &Settings)
  {
    ACarlaWheeledVehicle *Car = Vehicle.Get();
    if (Car == nullptr)
    {
      return;
    }
    USceneComponent *Body = Car->GetMesh();
    if (!Settings.Component.IsEmpty())
    {
      Body = FindComponent(*Car, Settings.Component);
      if (Body == nullptr)
      {
        UE_LOG(LogCarlaServerMirrors, Warning, TEXT("Mirrors: '%s': %s has no component '%s'; skipped"),
            *Settings.Name, *BlueprintId, *Settings.Component);
        return;
      }
    }
    FMirror &Mirror = Mirrors.AddDefaulted_GetRef();
    Mirror.Parent = Body;
    Mirror.Settings = Settings;
    const FVector Normal = Settings.Normal.GetSafeNormal(UE_SMALL_NUMBER, FVector(-1.0, 0.0, 0.0));
    // "aim" turns the mirror about the vertical, then tilts it, from the glass.
    Mirror.AimNormal = FRotator(Settings.Aim.Y, Settings.Aim.X, 0.0f).RotateVector(Normal).GetSafeNormal();

    const int32 Width = Settings.Resolution;
    const int32 Height = FMath::Max(16, FMath::RoundToInt(Width * Settings.Size.Y / FMath::Max(Settings.Size.X, 1.0)));
    UTextureRenderTarget2D *Target = NewObject<UTextureRenderTarget2D>(Car);
    Target->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA8_SRGB;
    Target->ClearColor = FLinearColor::Black;
    Target->InitAutoFormat(Width, Height);
    Target->UpdateResourceImmediate(true);
    Mirror.Target.Reset(Target);

    UMaterialInstanceDynamic *Instance = UMaterialInstanceDynamic::Create(Material.Get(), Car);
    Instance->SetTextureParameterValue(TEXT("Mirror"), Target);
    Instance->SetScalarParameterValue(TEXT("FlipU"), Settings.FlipU);
    Instance->SetScalarParameterValue(TEXT("FlipV"), Settings.FlipV);
    Instance->SetScalarParameterValue(TEXT("SwapUV"), Settings.SwapUV);
    Instance->SetScalarParameterValue(TEXT("Brightness"), Settings.Brightness);
    // The blind-spot icon, round on the glass: the image is Width x Height.
    Instance->SetVectorParameterValue(TEXT("IconCenter"), FLinearColor(Settings.Icon.X, Settings.Icon.Y, 0.0f));
    Instance->SetScalarParameterValue(TEXT("IconRadius"), Settings.Icon.Z);
    Instance->SetScalarParameterValue(TEXT("Aspect"), static_cast<float>(Width) / Height);
    Mirror.Warning = Settings.BlindSpotSide >= 0 ? BlindSpotLevels[Settings.BlindSpotSide] : 0.0f;
    Instance->SetScalarParameterValue(TEXT("Warning"), Mirror.Warning);
    Mirror.Instance.Reset(Instance);

    // The panel: the plane's +Z on the glass normal, its X up the glass.
    UStaticMeshComponent *Panel = NewObject<UStaticMeshComponent>(Car);
    Panel->SetStaticMesh(Plane.Get());
    Panel->SetMaterial(0, Instance);
    Panel->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Panel->SetCastShadow(false);
    Panel->bReceivesDecals = false;
    Panel->SetupAttachment(Body);
    Panel->SetRelativeLocation(Settings.Center + Normal * Settings.Offset);
    Panel->SetRelativeRotation(FRotationMatrix::MakeFromZX(Normal, FVector::UpVector).Rotator());
    Panel->SetRelativeScale3D(FVector(Settings.Size.Y / PlaneSize, Settings.Size.X / PlaneSize, 1.0f));
    Panel->RegisterComponent();
    Mirror.Panel = Panel;

    USceneCaptureComponent2D *Capture = NewObject<USceneCaptureComponent2D>(Car);
    Capture->SetupAttachment(Body);
    Capture->TextureTarget = Target;
    Capture->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
    // A plain capture has no auto exposure, so a physically lit scene comes
    // out nearly black: adapt like the player's view and grade like it (as
    // CARLA's RGB cameras do), without the blur a mirror would not show.
    Capture->ShowFlags.SetEyeAdaptation(true);
    Capture->ShowFlags.SetColorGrading(true);
    Capture->ShowFlags.SetTemporalAA(true);
    // A mirror is small: it is rendered cheaper than the main view, with
    // nothing past "view_distance" and coarser LODs; "lumen": false also
    // swaps Lumen for screen-space lighting.
    Capture->ShowFlags.SetAmbientOcclusion(false);
    Capture->ShowFlags.SetDistanceFieldAO(false);
    Capture->ShowFlags.SetVolumetricFog(false);
    Capture->MaxViewDistanceOverride = Settings.ViewDistance;
    Capture->LODDistanceFactor = FMath::Max(Settings.LodFactor, 1.0f);
    if (!Settings.bLumen)
    {
      Capture->PostProcessSettings.bOverride_DynamicGlobalIlluminationMethod = true;
      Capture->PostProcessSettings.DynamicGlobalIlluminationMethod = EDynamicGlobalIlluminationMethod::ScreenSpace;
      Capture->PostProcessSettings.bOverride_ReflectionMethod = true;
      Capture->PostProcessSettings.ReflectionMethod = EReflectionMethod::ScreenSpace;
    }
    Capture->ShowFlags.SetMotionBlur(false);
    Capture->ShowFlags.SetDepthOfField(false);
    // CARLA's RGB camera defaults: histogram auto exposure between EV100 10
    // and 12, which fits its maps' physical lighting, plus "exposure" (EV).
    FPostProcessSettings &Post = Capture->PostProcessSettings;
    Post.bOverride_AutoExposureMethod = true;
    Post.AutoExposureMethod = EAutoExposureMethod::AEM_Histogram;
    Post.bOverride_AutoExposureMinBrightness = true;
    Post.AutoExposureMinBrightness = 10.0f;
    Post.bOverride_AutoExposureMaxBrightness = true;
    Post.AutoExposureMaxBrightness = 12.0f;
    Post.bOverride_AutoExposureBias = true;
    Post.AutoExposureBias = Settings.Exposure;
    Capture->PostProcessSettings.bOverride_AutoExposureSpeedUp = true;
    Capture->PostProcessSettings.AutoExposureSpeedUp = 6.0f;
    Capture->PostProcessSettings.bOverride_AutoExposureSpeedDown = true;
    Capture->PostProcessSettings.AutoExposureSpeedDown = 6.0f;
    Capture->FOVAngle = Settings.Fov;
    Capture->bCaptureEveryFrame = false;
    Capture->bCaptureOnMovement = false;
    Capture->bAlwaysPersistRenderingState = true;
    Capture->HideComponent(Panel);
    Capture->RegisterComponent();
    Mirror.Capture = Capture;
  }

  /// A component of the vehicle by name: exact, else the first whose name
  /// starts with it (Blueprint components can carry suffixes).
  static USceneComponent *FindComponent(ACarlaWheeledVehicle &Car, const FString &Name)
  {
    TInlineComponentArray<USceneComponent *> Components;
    Car.GetComponents(Components);
    for (USceneComponent *Component : Components)
    {
      if (Component->GetName().Equals(Name, ESearchCase::IgnoreCase))
      {
        return Component;
      }
    }
    for (USceneComponent *Component : Components)
    {
      if (Component->GetName().StartsWith(Name, ESearchCase::IgnoreCase))
      {
        return Component;
      }
    }
    return nullptr;
  }

  void DestroyMirrors()
  {
    for (FMirror &Mirror : Mirrors)
    {
      if (Mirror.Panel.IsValid())
      {
        Mirror.Panel->DestroyComponent();
      }
      if (Mirror.Capture.IsValid())
      {
        Mirror.Capture->DestroyComponent();
      }
    }
    Mirrors.Reset();
  }

  TWeakObjectPtr<ACarlaWheeledVehicle> Vehicle;
  FString BlueprintId;
  FString ConfigPath;
  FDateTime ConfigStamp;
  double LastReloadCheck = 0.0;
  int32 NextCapture = 0;
  float BlindSpotLevels[2] = {0.0f, 0.0f};
  TStrongObjectPtr<UMaterialInterface> Material;
  TStrongObjectPtr<UStaticMesh> Plane;
  TArray<FMirror> Mirrors;

public:

  TOptional<FVector> Eye;
  TOptional<FVector> MirrorEye;
  FString Mesh;
  bool bEveryFrame = false;
};

FString FServerMirrors::GetConfigPath()
{
  FString Path;
  if (FParse::Value(FCommandLine::Get(), TEXT("-server-mirrors="), Path) && !Path.IsEmpty())
  {
    return FPaths::ConvertRelativePathToFull(Path);
  }
  return FPaths::ConvertRelativePathToFull(FPaths::ProjectConfigDir() / TEXT("ServerPlayerMirrors.json"));
}

FServerMirrors::FServerMirrors(ACarlaWheeledVehicle &Vehicle, const FString &BlueprintId)
  : Impl(MakeUnique<FImpl>(Vehicle, BlueprintId)) {}

FServerMirrors::~FServerMirrors() = default;

void FServerMirrors::Tick(const FVector &EyeLocation, bool bCapture)
{
  Impl->Tick(EyeLocation, bCapture);
}

void FServerMirrors::SetBlindSpot(float Left, float Right)
{
  Impl->SetBlindSpot(Left, Right);
}

bool FServerMirrors::GetEye(FVector &Eye) const
{
  if (Impl->Eye.IsSet())
  {
    Eye = Impl->Eye.GetValue();
    return true;
  }
  return false;
}

FString FServerMirrors::GetMesh() const
{
  return Impl->Mesh;
}

bool FServerMirrors::GetMirrorEye(FVector &MirrorEye) const
{
  if (Impl->MirrorEye.IsSet())
  {
    MirrorEye = Impl->MirrorEye.GetValue();
    return true;
  }
  return GetEye(MirrorEye);
}
