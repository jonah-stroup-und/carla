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

/// An A-pillar monitor for the server player's vehicle: a light low on each
/// A-pillar, on the cabin side, lights while a road user (pedestrian,
/// cyclist or vehicle) is hidden behind that pillar from the driver's eye,
/// the classic risk when turning at a junction. In VisualAudio mode a low
/// chime sounds when one disappears behind it.
///
/// What each pillar hides is baked from the vehicle mesh, as seen from the
/// driver's eye, into the "pillars" entry of Config/ServerPlayerMirrors.json
/// (the vehicle's cockpit file, see FServerMirrors), reloaded when it changes:
///
///   "pillars": {
///     "eye":   [x, y, z]  the driver's eye the outlines are seen from, cm,
///                         vehicle space
///     "left":  {
///       "outline": [[elevation, azimuth min, azimuth max], ...]  degrees
///                  from the eye (azimuth + to the right, elevation + up),
///                  one row per elevation step, bottom to top
///       "light":   [x, y, z]  the light's centre on the pillar, cm
///       "size":    the light's width, cm (default 3)
///       "offset":  how far it stands off the trim towards the eye, cm
///                  (default 2)
///       "color":   [r, g, b] (default amber)
///     },
///     "right": { ... }
///   }
///
/// A road user counts as hidden when the pillar covers its centre and most
/// of its width.
class FServerPillars
{
public:

  static constexpr int32 Left = 0;
  static constexpr int32 Right = 1;

  FServerPillars(ACarlaWheeledVehicle &Vehicle, const FString &BlueprintId, EServerWarningMode Mode);
  ~FServerPillars();

  FServerPillars(const FServerPillars &) = delete;
  FServerPillars &operator=(const FServerPillars &) = delete;

  EServerWarningMode GetMode() const;
  void SetMode(EServerWarningMode Mode);

  /// Lights both pillars steadily, to see where the lights are.
  void SetTest(bool bOn);

  /// Whether the vehicle has pillars in the config.
  bool HasPillars() const;

  struct FHidden
  {
    int32 Side = Left;
    TWeakObjectPtr<AActor> Actor;
  };

  /// Looks for hidden road users and lights the pillars; call once per frame.
  /// Returns the pedestrians and cyclists that went behind a pillar this
  /// frame (each at most once every few seconds), whatever the mode, for the
  /// drive report.
  TArray<FHidden> Tick(float DeltaSeconds);

  /// How bright the side's light is now, 0 to 1 (0 in Off mode).
  float GetIndicator(int32 Side) const;

private:

  class FImpl;
  TUniquePtr<FImpl> Impl;
};
