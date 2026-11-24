// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <util/ue-header-guard-begin.h>
#include "CoreMinimal.h"
#include <util/ue-header-guard-end.h>

class AActor;
class FServerChime;

/// The turn-signal flasher of the server player's car: the dashboard
/// indicator's on/off timing and the relay's sound, a sharp tick as the
/// lamp comes on and a softer tock as it goes off, about 85 flashes a
/// minute. A flash starts lit the moment a signal is switched on. The
/// sounds are generated, not assets.
class FServerBlinker
{
public:

  explicit FServerBlinker(AActor &Owner);
  ~FServerBlinker();

  /// Once a frame, with the car's signal state (both for hazards).
  void Tick(bool bLeft, bool bRight);

  /// Whether the left/right indicator is lit this frame.
  bool IsLeftLit() const { return bLit && bLeft; }
  bool IsRightLit() const { return bLit && bRight; }

private:

  TUniquePtr<FServerChime> TickSound;
  TUniquePtr<FServerChime> TockSound;
  bool bLeft = false;
  bool bRight = false;
  bool bLit = false;
  double Start = 0.0;
};
