// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <util/ue-header-guard-begin.h>
#include "CoreMinimal.h"
#include <util/ue-header-guard-end.h>

#include "Carla/Game/ServerWarning.h"

class AActor;
class ACarlaWheeledVehicle;

/// A blind-spot monitor for the server player's vehicle, modelled on the
/// ISO 17387 lane change decision aid: a vehicle travelling the same way in
/// the zone beside and behind the car (0.5 to 3 m out from its side, from
/// the driver's mirror back to 3 m behind the bumper), or one closing on
/// that zone from up to 30 m behind that would reach it within 3 s, lights
/// that side's indicator. Signalling towards a lit side flashes it, and in
/// VisualAudio mode sounds a chime.
///
/// It only decides what to show; FServerPlayer passes the indicators to the
/// mirrors (FServerMirrors::SetBlindSpot) and the HUD.
class FServerBlindSpot
{
public:

  static constexpr int32 Left = 0;
  static constexpr int32 Right = 1;

  /// BoundsOrigin/Extent: the vehicle's bounding box, actor space (cm);
  /// MirrorX: how far forward the zone starts, actor space (the driver's eye).
  FServerBlindSpot(ACarlaWheeledVehicle &Vehicle, const FVector &BoundsOrigin, const FVector &BoundsExtent,
      float MirrorX, EServerWarningMode Mode);
  ~FServerBlindSpot();

  FServerBlindSpot(const FServerBlindSpot &) = delete;
  FServerBlindSpot &operator=(const FServerBlindSpot &) = delete;

  EServerWarningMode GetMode() const { return Mode; }
  void SetMode(EServerWarningMode NewMode) { Mode = NewMode; }

  /// Lights both indicators steadily whatever is around, for lining the
  /// mirror icons up (-server-blind-spot-test).
  void SetTest(bool bOn) { bTest = bOn; }

  /// Scans the traffic; call once per frame. Returns a bit per side
  /// (1 << Left, 1 << Right) where a turn signal towards an occupied side
  /// started this frame, whatever the mode, for the drive report.
  int32 Tick(float DeltaSeconds, bool bSignalLeft, bool bSignalRight);

  bool IsOccupied(int32 Side) const { return bOccupied[Side]; }

  /// The vehicle that occupies the side, or null.
  AActor *GetOccupant(int32 Side) const { return Occupant[Side].Get(); }

  /// How bright the side's indicator is now, 0 to 1 (0 in Off mode).
  float GetIndicator(int32 Side) const { return Indicator[Side]; }

private:

  void Scan();

  TWeakObjectPtr<ACarlaWheeledVehicle> Vehicle;
  FVector BoundsOrigin;
  FVector BoundsExtent;
  float MirrorX;
  EServerWarningMode Mode;
  bool bTest = false;

  double Time = 0.0;
  bool bOccupied[2] = {false, false};
  double OccupiedUntil[2] = {-1.0, -1.0};
  TWeakObjectPtr<AActor> Occupant[2];
  bool bWarning[2] = {false, false};
  double WarningSince[2] = {0.0, 0.0};
  float Indicator[2] = {0.0f, 0.0f};

  TUniquePtr<FServerChime> ChimeSound;
};
