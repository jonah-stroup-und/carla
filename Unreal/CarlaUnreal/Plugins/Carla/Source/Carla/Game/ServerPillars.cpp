// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Game/ServerPillars.h"
#include "Carla.h"

#include "Carla/Game/ServerMirrors.h"
#include "Carla/Vehicle/CarlaWheeledVehicle.h"

#include <util/ue-header-guard-begin.h>
#include "Components/BoxComponent.h"
#include "Components/CapsuleComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Dom/JsonObject.h"
#include "Engine/StaticMesh.h"
#include "EngineUtils.h"
#include "GameFramework/Character.h"
#include "HAL/FileManager.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "Misc/FileHelper.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "UObject/StrongObjectPtr.h"
#include <util/ue-header-guard-end.h>

DEFINE_LOG_CATEGORY_STATIC(LogCarlaServerPillars, Log, All);

namespace
{
  /// The light is the mirror material showing only its indicator disc, on
  /// a black square: the light's housing.
  const TCHAR *PillarLightMaterialPath = TEXT("/Carla/Mirrors/M_ServerMirror.M_ServerMirror");
  const TCHAR *PillarPlaneMeshPath = TEXT("/Engine/BasicShapes/Plane.Plane");
  constexpr float PillarPlaneSize = 100.0f;
  constexpr double PillarReloadCheckSeconds = 0.5;

  /// Road users nearer or further than this (cm) are ignored.
  constexpr float PillarMinRange = 150.0f;
  constexpr float PillarMaxRange = 2000.0f; // about 65 ft
  /// Of the road user's centre and four edge points, how many the pillar
  /// must cover (the centre always among them).
  constexpr int32 PillarHiddenPoints = 3;
  /// The light stays on this long after the road user reappears.
  constexpr double PillarHoldSeconds = 0.4;
  /// The chime sounds at most this often per pillar.
  constexpr double PillarChimeCooldownSeconds = 3.0;
  /// A road user is reported at most this often.
  constexpr double PillarReportCooldownSeconds = 10.0;
  /// Vehicles narrower than this (half width, cm) are two-wheelers.
  constexpr float PillarTwoWheelerHalfWidth = 55.0f;

  // The chime: two low beeps, unlike the blind-spot monitor's three high ones.
  constexpr float PillarChimeHz = 880.0f;
  constexpr int32 PillarChimeBeeps = 2;
  constexpr float PillarChimeBeepSeconds = 0.14f;
  constexpr float PillarChimeGapSeconds = 0.08f;
  constexpr float PillarChimeVolume = 0.4f;

  struct FPillarSettings
  {
    bool bValid = false;
    TArray<FVector> Outline; // (elevation, azimuth min, azimuth max)
    FVector Light = FVector::ZeroVector;
    float Size = 3.0f;
    float Offset = 2.0f;
    FLinearColor Color = FLinearColor(1.0f, 0.42f, 0.02f);
  };

  bool ReadNumbers(const TSharedPtr<FJsonObject> &Object, const TCHAR *Field, TArray<double> &Out)
  {
    const TArray<TSharedPtr<FJsonValue>> *Values = nullptr;
    if (!Object->TryGetArrayField(Field, Values))
    {
      return false;
    }
    Out.Reset();
    for (const TSharedPtr<FJsonValue> &Value : *Values)
    {
      Out.Add(Value->AsNumber());
    }
    return true;
  }
}

class FServerPillars::FImpl
{
public:

  FImpl(ACarlaWheeledVehicle &InVehicle, const FString &InBlueprintId, EServerWarningMode InMode)
    : Vehicle(&InVehicle),
      BlueprintId(InBlueprintId),
      ConfigPath(FServerMirrors::GetConfigPath())
  {
    Mode = InMode;
    Material.Reset(LoadObject<UMaterialInterface>(nullptr, PillarLightMaterialPath));
    Plane.Reset(LoadObject<UStaticMesh>(nullptr, PillarPlaneMeshPath));
    Chime = MakeUnique<FServerChime>(InVehicle, PillarChimeHz, PillarChimeBeeps, PillarChimeBeepSeconds, PillarChimeGapSeconds, PillarChimeVolume);
    Reload();
  }

  ~FImpl()
  {
    DestroyLights();
  }

  EServerWarningMode Mode;
  bool bTest = false;

  bool HasPillars() const
  {
    return Pillars[Left].bValid || Pillars[Right].bValid;
  }

  float GetIndicator(int32 Side) const
  {
    return Indicator[Side];
  }

  TArray<FHidden> Tick(float DeltaSeconds)
  {
    Time += DeltaSeconds;
    const double Now = FPlatformTime::Seconds();
    if (Now - LastReloadCheck >= PillarReloadCheckSeconds)
    {
      LastReloadCheck = Now;
      if (IFileManager::Get().GetTimeStamp(*ConfigPath) != ConfigStamp)
      {
        Reload();
      }
    }

    TArray<FHidden> Reports;
    ACarlaWheeledVehicle *Car = Vehicle.Get();
    if (Car == nullptr || !HasPillars())
    {
      return Reports;
    }
    const FTransform Frame = Car->GetActorTransform();
    const FVector EyeWorld = Frame.TransformPosition(Eye);

    bool bHidden[2] = {false, false};
    for (TActorIterator<APawn> It(Car->GetWorld()); It; ++It)
    {
      APawn *Other = *It;
      if (Other == Car || !IsValid(Other))
      {
        continue;
      }
      // The road user's centre and the four ends of its width and length,
      // at the height the driver would look for it.
      TArray<FVector, TInlineAllocator<5>> Points;
      bool bVulnerable = false;
      if (const ACarlaWheeledVehicle *OtherVehicle = Cast<ACarlaWheeledVehicle>(Other))
      {
        const UBoxComponent *Box = OtherVehicle->GetVehicleBoundingBox();
        if (Box == nullptr)
        {
          continue;
        }
        const FVector Centre = Box->GetComponentLocation();
        const FVector Extent = Box->GetScaledBoxExtent();
        const FQuat Rotation = Box->GetComponentQuat();
        Points.Add(Centre);
        Points.Add(Centre + Rotation.RotateVector(FVector(Extent.X, 0.0f, 0.0f)));
        Points.Add(Centre - Rotation.RotateVector(FVector(Extent.X, 0.0f, 0.0f)));
        Points.Add(Centre + Rotation.RotateVector(FVector(0.0f, Extent.Y, 0.0f)));
        Points.Add(Centre - Rotation.RotateVector(FVector(0.0f, Extent.Y, 0.0f)));
        bVulnerable = Extent.Y < PillarTwoWheelerHalfWidth;
      }
      else if (const ACharacter *Walker = Cast<ACharacter>(Other))
      {
        const UCapsuleComponent *Capsule = Walker->GetCapsuleComponent();
        if (Capsule == nullptr)
        {
          continue;
        }
        const float Radius = Capsule->GetScaledCapsuleRadius();
        // Chest height: what shows over a parked car or a bonnet.
        const FVector Centre = Capsule->GetComponentLocation() + FVector(0.0f, 0.0f, 0.4f * Capsule->GetScaledCapsuleHalfHeight());
        const FVector Across = FVector::CrossProduct((Centre - EyeWorld).GetSafeNormal(), FVector::UpVector).GetSafeNormal();
        Points.Add(Centre);
        Points.Add(Centre + Across * Radius);
        Points.Add(Centre - Across * Radius);
        bVulnerable = true;
      }
      else
      {
        continue; // the spectator, or anything else that is not a road user
      }
      const float Distance = FVector::Dist(Points[0], EyeWorld);
      if (Distance < PillarMinRange || Distance > PillarMaxRange)
      {
        continue;
      }

      for (int32 Side = 0; Side < 2; ++Side)
      {
        if (!Pillars[Side].bValid || !Covers(Pillars[Side], Frame, Points[0]))
        {
          continue;
        }
        int32 Covered = 1;
        for (int32 Index = 1; Index < Points.Num(); ++Index)
        {
          Covered += Covers(Pillars[Side], Frame, Points[Index]) ? 1 : 0;
        }
        if (Covered < FMath::Min(PillarHiddenPoints, Points.Num()))
        {
          continue;
        }
        if (!bHidden[Side] && Time > HiddenUntil[Side])
        {
          UE_LOG(LogCarlaServerPillars, Verbose, TEXT("A-pillars: %s pillar hides %s at %.0f m"),
              Side == Left ? TEXT("left") : TEXT("right"), *Other->GetName(), Distance / 100.0f);
        }
        bHidden[Side] = true;
        double &LastReport = LastReported.FindOrAdd(Other, -PillarReportCooldownSeconds);
        if (bVulnerable && Time - LastReport >= PillarReportCooldownSeconds && !WasHidden.Contains(Other))
        {
          LastReport = Time;
          Reports.Add({Side, Other});
        }
        NowHidden.Add(Other);
      }
    }
    Swap(WasHidden, NowHidden);
    NowHidden.Reset();

    for (int32 Side = 0; Side < 2; ++Side)
    {
      const bool bWasOn = Time <= HiddenUntil[Side];
      if (bHidden[Side])
      {
        HiddenUntil[Side] = Time + PillarHoldSeconds;
        if (!bWasOn && Mode == EServerWarningMode::VisualAudio && Time - LastChime[Side] >= PillarChimeCooldownSeconds)
        {
          LastChime[Side] = Time;
          Chime->Play();
        }
      }
      const bool bOn = Time <= HiddenUntil[Side];
      SetLight(Side, bTest ? 1.0f : (Mode != EServerWarningMode::Off && bOn ? 1.0f : 0.0f));
    }
    return Reports;
  }

private:

  /// Whether the pillar covers the point (world space), seen from the eye.
  bool Covers(const FPillarSettings &Pillar, const FTransform &Frame, const FVector &Point) const
  {
    const FVector Local = Frame.InverseTransformPosition(Point) - Eye;
    const float Azimuth = FMath::RadiansToDegrees(FMath::Atan2(Local.Y, Local.X));
    const float Elevation = FMath::RadiansToDegrees(FMath::Atan2(Local.Z, FVector2D(Local.X, Local.Y).Size()));
    const TArray<FVector> &Rows = Pillar.Outline;
    const float HalfStep = Rows.Num() > 1 ? 0.5f * FMath::Abs(Rows[1].X - Rows[0].X) : 0.5f;
    if (Elevation < Rows[0].X - HalfStep || Elevation > Rows.Last().X + HalfStep)
    {
      return false;
    }
    const FVector *Nearest = &Rows[0];
    for (const FVector &Row : Rows)
    {
      if (FMath::Abs(Row.X - Elevation) < FMath::Abs(Nearest->X - Elevation))
      {
        Nearest = &Row;
      }
    }
    return Azimuth >= Nearest->Y && Azimuth <= Nearest->Z;
  }

  void SetLight(int32 Side, float Level)
  {
    Indicator[Side] = Level;
    UMaterialInstanceDynamic *Instance = Lights[Side].Instance.Get();
    if (Instance != nullptr && Lights[Side].Level != Level)
    {
      Lights[Side].Level = Level;
      Instance->SetScalarParameterValue(TEXT("Warning"), Level);
    }
  }

  void Reload()
  {
    ConfigStamp = IFileManager::Get().GetTimeStamp(*ConfigPath);
    FString Text;
    TSharedPtr<FJsonObject> Root;
    if (!FFileHelper::LoadFileToString(Text, *ConfigPath) ||
        !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text), Root) || !Root.IsValid())
    {
      UE_LOG(LogCarlaServerPillars, Warning, TEXT("A-pillars: cannot read %s; keeping the current pillars"), *ConfigPath);
      return;
    }
    DestroyLights();
    Pillars[Left] = FPillarSettings();
    Pillars[Right] = FPillarSettings();
    const TSharedPtr<FJsonObject> *VehicleEntry = nullptr;
    const TSharedPtr<FJsonObject> *Entry = nullptr;
    TArray<double> Numbers;
    if (!Root->TryGetObjectField(BlueprintId, VehicleEntry) || !(*VehicleEntry)->TryGetObjectField(TEXT("pillars"), Entry) ||
        !ReadNumbers(*Entry, TEXT("eye"), Numbers) || Numbers.Num() != 3)
    {
      UE_LOG(LogCarlaServerPillars, Log, TEXT("A-pillars: %s has no pillars (with an eye) for %s; no A-pillar monitor"),
          *ConfigPath, *BlueprintId);
      return;
    }
    Eye = FVector(Numbers[0], Numbers[1], Numbers[2]);
    const TCHAR *Names[2] = {TEXT("left"), TEXT("right")};
    for (int32 Side = 0; Side < 2; ++Side)
    {
      const TSharedPtr<FJsonObject> *PillarEntry = nullptr;
      if (!(*Entry)->TryGetObjectField(Names[Side], PillarEntry))
      {
        continue;
      }
      FPillarSettings &Pillar = Pillars[Side];
      const TArray<TSharedPtr<FJsonValue>> *Rows = nullptr;
      if ((*PillarEntry)->TryGetArrayField(TEXT("outline"), Rows))
      {
        for (const TSharedPtr<FJsonValue> &Row : *Rows)
        {
          const TArray<TSharedPtr<FJsonValue>> *Values = nullptr;
          if (Row->TryGetArray(Values) && Values->Num() == 3)
          {
            Pillar.Outline.Add(FVector((*Values)[0]->AsNumber(), (*Values)[1]->AsNumber(), (*Values)[2]->AsNumber()));
          }
        }
      }
      Pillar.Outline.Sort([](const FVector &A, const FVector &B) { return A.X < B.X; });
      if (ReadNumbers(*PillarEntry, TEXT("light"), Numbers) && Numbers.Num() == 3)
      {
        Pillar.Light = FVector(Numbers[0], Numbers[1], Numbers[2]);
      }
      double Size = Pillar.Size;
      if ((*PillarEntry)->TryGetNumberField(TEXT("size"), Size))
      {
        Pillar.Size = static_cast<float>(Size);
      }
      double Offset = Pillar.Offset;
      if ((*PillarEntry)->TryGetNumberField(TEXT("offset"), Offset))
      {
        Pillar.Offset = static_cast<float>(Offset);
      }
      if (ReadNumbers(*PillarEntry, TEXT("color"), Numbers) && Numbers.Num() == 3)
      {
        Pillar.Color = FLinearColor(Numbers[0], Numbers[1], Numbers[2]);
      }
      Pillar.bValid = Pillar.Outline.Num() > 0;
      if (Pillar.bValid && !Pillar.Light.IsZero())
      {
        CreateLight(Side);
      }
    }
    UE_LOG(LogCarlaServerPillars, Log, TEXT("A-pillars: left %d rows, right %d rows for %s"),
        Pillars[Left].Outline.Num(), Pillars[Right].Outline.Num(), *BlueprintId);
  }

  /// A small square panel on the pillar facing the driver's eye: black,
  /// with the indicator disc in the middle.
  void CreateLight(int32 Side)
  {
    ACarlaWheeledVehicle *Car = Vehicle.Get();
    if (Car == nullptr || !Material.IsValid() || !Plane.IsValid())
    {
      return;
    }
    const FPillarSettings &Pillar = Pillars[Side];
    const FVector Facing = (Eye - Pillar.Light).GetSafeNormal();

    UMaterialInstanceDynamic *Instance = UMaterialInstanceDynamic::Create(Material.Get(), Car);
    Instance->SetScalarParameterValue(TEXT("Brightness"), 0.0f);
    Instance->SetVectorParameterValue(TEXT("IconCenter"), FLinearColor(0.5f, 0.5f, 0.0f));
    Instance->SetScalarParameterValue(TEXT("IconRadius"), 0.36f);
    Instance->SetScalarParameterValue(TEXT("Aspect"), 1.0f);
    Instance->SetVectorParameterValue(TEXT("IconColor"), Pillar.Color);
    Instance->SetScalarParameterValue(TEXT("Warning"), 0.0f);

    UStaticMeshComponent *Panel = NewObject<UStaticMeshComponent>(Car);
    Panel->SetStaticMesh(Plane.Get());
    Panel->SetMaterial(0, Instance);
    Panel->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Panel->SetCastShadow(false);
    Panel->bReceivesDecals = false;
    Panel->SetupAttachment(Car->GetMesh());
    // Off the trim towards the eye, so the slanted trim never cuts into it;
    // from the driver's seat it still sits on the same spot.
    Panel->SetRelativeLocation(Pillar.Light + Facing * Pillar.Offset);
    Panel->SetRelativeRotation(FRotationMatrix::MakeFromZX(Facing, FVector::UpVector).Rotator());
    Panel->SetRelativeScale3D(FVector(Pillar.Size / PillarPlaneSize, Pillar.Size / PillarPlaneSize, 1.0f));
    Panel->RegisterComponent();

    Lights[Side].Panel = Panel;
    Lights[Side].Instance.Reset(Instance);
    Lights[Side].Level = 0.0f;
  }

  void DestroyLights()
  {
    for (FLight &Light : Lights)
    {
      if (Light.Panel.IsValid())
      {
        Light.Panel->DestroyComponent();
      }
      Light = FLight();
    }
  }

  struct FLight
  {
    TWeakObjectPtr<UStaticMeshComponent> Panel;
    TStrongObjectPtr<UMaterialInstanceDynamic> Instance;
    float Level = 0.0f;
  };

  TWeakObjectPtr<ACarlaWheeledVehicle> Vehicle;
  FString BlueprintId;
  FString ConfigPath;
  FDateTime ConfigStamp;
  double LastReloadCheck = 0.0;
  TStrongObjectPtr<UMaterialInterface> Material;
  TStrongObjectPtr<UStaticMesh> Plane;
  TUniquePtr<FServerChime> Chime;

  FVector Eye = FVector::ZeroVector;
  FPillarSettings Pillars[2];
  FLight Lights[2];
  float Indicator[2] = {0.0f, 0.0f};
  double Time = 0.0;
  double HiddenUntil[2] = {-1.0, -1.0};
  double LastChime[2] = {-100.0, -100.0};
  TMap<TWeakObjectPtr<AActor>, double> LastReported;
  TSet<TWeakObjectPtr<AActor>> WasHidden;
  TSet<TWeakObjectPtr<AActor>> NowHidden;
};

FServerPillars::FServerPillars(ACarlaWheeledVehicle &Vehicle, const FString &BlueprintId, EServerWarningMode Mode)
  : Impl(MakeUnique<FImpl>(Vehicle, BlueprintId, Mode)) {}

FServerPillars::~FServerPillars() = default;

EServerWarningMode FServerPillars::GetMode() const
{
  return Impl->Mode;
}

void FServerPillars::SetMode(EServerWarningMode Mode)
{
  Impl->Mode = Mode;
}

void FServerPillars::SetTest(bool bOn)
{
  Impl->bTest = bOn;
}

bool FServerPillars::HasPillars() const
{
  return Impl->HasPillars();
}

TArray<FServerPillars::FHidden> FServerPillars::Tick(float DeltaSeconds)
{
  return Impl->Tick(DeltaSeconds);
}

float FServerPillars::GetIndicator(int32 Side) const
{
  return Impl->GetIndicator(Side);
}
