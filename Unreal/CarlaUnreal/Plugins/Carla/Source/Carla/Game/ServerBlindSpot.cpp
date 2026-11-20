// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Game/ServerBlindSpot.h"
#include "Carla.h"

#include "Carla/Vehicle/CarlaWheeledVehicle.h"

#include <util/ue-header-guard-begin.h>
#include "Components/BoxComponent.h"
#include "EngineUtils.h"
#include <util/ue-header-guard-end.h>

namespace
{
  // The zone (cm), after ISO 17387: from 0.5 m to 3 m out from the side,
  // from the mirror to 3 m behind the rear bumper.
  constexpr float ZoneNear = 50.0f;
  constexpr float ZoneFar = 300.0f;
  constexpr float ZoneBehind = 300.0f;
  /// Closing vehicles: up to this far behind the zone, reaching it within
  /// ClosingSeconds.
  constexpr float ClosingRange = 3000.0f;
  constexpr float ClosingSeconds = 3.0f;
  /// Only traffic going the same way: cos of the largest heading difference.
  constexpr float SameWayCos = 0.5f;
  /// Vehicles further than this (cm) are not looked at.
  constexpr float ScanRadius = 6000.0f;
  /// The indicator stays on this long after the zone clears, so it does
  /// not flicker as a car's edge moves across the zone's.
  constexpr double HoldSeconds = 0.4;
  /// Flashing while signalling towards an occupied side.
  constexpr double FlashHz = 2.5;
  constexpr float FlashLow = 0.15f;

  // The chime: three short, high beeps.
  constexpr float ChimeHz = 1400.0f;
  constexpr int32 ChimeBeeps = 3;
  constexpr float ChimeBeepSeconds = 0.09f;
  constexpr float ChimeGapSeconds = 0.07f;
  constexpr float ChimeVolume = 0.35f;
}

FServerBlindSpot::FServerBlindSpot(ACarlaWheeledVehicle &InVehicle, const FVector &InBoundsOrigin,
    const FVector &InBoundsExtent, float InMirrorX, EServerWarningMode InMode)
  : Vehicle(&InVehicle),
    BoundsOrigin(InBoundsOrigin),
    BoundsExtent(InBoundsExtent),
    MirrorX(InMirrorX),
    Mode(InMode)
{
  ChimeSound = MakeUnique<FServerChime>(InVehicle, ChimeHz, ChimeBeeps, ChimeBeepSeconds, ChimeGapSeconds, ChimeVolume);
}

FServerBlindSpot::~FServerBlindSpot() = default;

int32 FServerBlindSpot::Tick(float DeltaSeconds, bool bSignalLeft, bool bSignalRight)
{
  Time += DeltaSeconds;
  Scan();
  int32 Started = 0;
  const bool bSignal[2] = {bSignalLeft, bSignalRight};
  for (int32 Side = 0; Side < 2; ++Side)
  {
    const bool bWarn = bOccupied[Side] && bSignal[Side] && !bTest;
    if (bWarn && !bWarning[Side])
    {
      Started |= 1 << Side;
      WarningSince[Side] = Time;
      if (Mode == EServerWarningMode::VisualAudio)
      {
        ChimeSound->Play();
      }
    }
    bWarning[Side] = bWarn;

    float Level = 0.0f;
    if (bTest)
    {
      Level = 1.0f;
    }
    else if (Mode != EServerWarningMode::Off && bOccupied[Side])
    {
      const bool bFlashOn = !bWarn || FMath::Frac((Time - WarningSince[Side]) * FlashHz) < 0.5;
      Level = bFlashOn ? 1.0f : FlashLow;
    }
    Indicator[Side] = Level;
  }
  return Started;
}

void FServerBlindSpot::Scan()
{
  ACarlaWheeledVehicle *Car = Vehicle.Get();
  if (Car == nullptr)
  {
    return;
  }
  const FTransform Frame = Car->GetActorTransform();
  const FVector Forward = Car->GetActorForwardVector();
  const FVector Location = Car->GetActorLocation();
  const FVector Velocity = Car->GetVelocity();

  const float Rear = BoundsOrigin.X - BoundsExtent.X;
  const float ZoneStart = Rear - ZoneBehind;
  // Actor space: +Y is right.
  const float SideY[2] = {BoundsOrigin.Y - BoundsExtent.Y, BoundsOrigin.Y + BoundsExtent.Y};

  bool bFound[2] = {false, false};
  AActor *Found[2] = {nullptr, nullptr};
  float FoundDistance[2] = {TNumericLimits<float>::Max(), TNumericLimits<float>::Max()};

  for (TActorIterator<ACarlaWheeledVehicle> It(Car->GetWorld()); It; ++It)
  {
    ACarlaWheeledVehicle *Other = *It;
    if (Other == Car || !IsValid(Other))
    {
      continue;
    }
    const FVector OtherLocation = Other->GetActorLocation();
    if (FVector::DistSquared(OtherLocation, Location) > FMath::Square(ScanRadius) ||
        FVector::DotProduct(Other->GetActorForwardVector(), Forward) < SameWayCos)
    {
      continue;
    }

    // The other vehicle's box, as a rectangle in this vehicle's space.
    FVector Centre = OtherLocation;
    FVector Extent(200.0f, 90.0f, 75.0f);
    FQuat Rotation = Other->GetActorQuat();
    if (const UBoxComponent *Box = Other->GetVehicleBoundingBox())
    {
      Centre = Box->GetComponentLocation();
      Extent = Box->GetScaledBoxExtent();
      Rotation = Box->GetComponentQuat();
    }
    FVector2D Min(TNumericLimits<float>::Max(), TNumericLimits<float>::Max());
    FVector2D Max(-TNumericLimits<float>::Max(), -TNumericLimits<float>::Max());
    for (int32 Corner = 0; Corner < 4; ++Corner)
    {
      const FVector Offset((Corner & 1) ? Extent.X : -Extent.X, (Corner & 2) ? Extent.Y : -Extent.Y, 0.0f);
      const FVector Local = Frame.InverseTransformPosition(Centre + Rotation.RotateVector(Offset));
      Min = FVector2D(FMath::Min(Min.X, Local.X), FMath::Min(Min.Y, Local.Y));
      Max = FVector2D(FMath::Max(Max.X, Local.X), FMath::Max(Max.Y, Local.Y));
    }

    for (int32 Side = 0; Side < 2; ++Side)
    {
      // Out from the side: the left zone is at smaller Y, the right at larger.
      const float Sign = Side == Left ? -1.0f : 1.0f;
      const float ZoneInner = SideY[Side] + Sign * ZoneNear;
      const float ZoneOuter = SideY[Side] + Sign * ZoneFar;
      const bool bLateral = Side == Left
          ? (Max.Y >= ZoneOuter && Min.Y <= ZoneInner)
          : (Min.Y <= ZoneOuter && Max.Y >= ZoneInner);
      if (!bLateral || Min.X > MirrorX)
      {
        continue; // not beside, or ahead of the mirror
      }
      bool bIn = Max.X >= ZoneStart;
      float Distance = FMath::Max(ZoneStart - Max.X, 0.0f);
      if (!bIn && ZoneStart - Max.X <= ClosingRange)
      {
        // Catching up from behind: closing speed along this vehicle (cm/s).
        const float Closing = FVector::DotProduct(Other->GetVelocity() - Velocity, Forward);
        bIn = Closing > 0.0f && Distance / Closing <= ClosingSeconds;
      }
      if (bIn && Distance < FoundDistance[Side])
      {
        bFound[Side] = true;
        Found[Side] = Other;
        FoundDistance[Side] = Distance;
      }
    }
  }

  for (int32 Side = 0; Side < 2; ++Side)
  {
    if (bFound[Side])
    {
      OccupiedUntil[Side] = Time + HoldSeconds;
      Occupant[Side] = Found[Side];
    }
    bOccupied[Side] = Time <= OccupiedUntil[Side];
    if (!bOccupied[Side])
    {
      Occupant[Side].Reset();
    }
  }
}
