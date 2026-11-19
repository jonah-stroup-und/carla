// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <util/ue-header-guard-begin.h>
#include "CoreMinimal.h"
#include <util/ue-header-guard-end.h>

class ACarlaWheeledVehicle;

/// Working mirrors for the server player's vehicle.
///
/// The CARLA vehicle meshes have no separate mirror material, so each mirror
/// is a thin panel laid over the glass, showing a scene capture through
/// M_ServerMirror (/Carla/Mirrors, built by Util/Tools/
/// CreateServerMirrorMaterial.py) flipped left to right. Every frame the
/// capture sits on the glass and looks along the driver's line of sight
/// reflected off it, so the view moves with the driver's eye like a real
/// mirror. The mirrors render one per frame, in turn.
///
/// Where the glass is comes from Config/ServerPlayerMirrors.json, keyed by
/// blueprint id (or -server-mirrors=FILE). The file is reloaded when it
/// changes, so a mirror can be lined up while driving. Each mirror:
///
///   "name":       label for the log
///   "component":  the vehicle part the mirror is on, by component name
///                 (e.g. "Door_FL", so it swings with the door); default the
///                 vehicle mesh. "center" and "normal" are in its space.
///   "center":     [x, y, z] centre of the glass, cm
///   "size":       [width, height] of the glass, cm
///   "normal":     [x, y, z] the way the glass faces (towards the driver)
///   "aim":        [yaw, pitch] degrees to turn the mirror from "normal",
///                 like adjusting it from the driver's seat (default 0, 0)
///   "fov":        horizontal field of view, degrees (a convex mirror is wider)
///   "resolution": render target width in pixels (height follows the size)
///   "offset":     cm to lift the panel off the glass (default 0.3)
///   "exposure":   EV added to the auto exposure (default 0)
///   "view_distance": cm beyond which nothing is drawn (default 15000)
///   "lod_factor": scales LOD distances, >1 is coarser and cheaper (default 2)
///   "lumen":      false for screen-space GI and reflections instead of
///                 Lumen (default true, like the main view; measured no
///                 clear saving, and a visibly darker mirror)
///   "flip_u", "flip_v", "swap_uv", "brightness": material parameters
class FServerMirrors
{
public:

  static FString GetConfigPath();

  FServerMirrors(ACarlaWheeledVehicle &Vehicle, const FString &BlueprintId);
  ~FServerMirrors();

  FServerMirrors(const FServerMirrors &) = delete;
  FServerMirrors &operator=(const FServerMirrors &) = delete;

  /// Aims the captures from the driver's eye (world space); they only render
  /// while bCapture is set, e.g. in the interior camera.
  void Tick(const FVector &EyeLocation, bool bCapture);

private:

  class FImpl;
  TUniquePtr<FImpl> Impl;
};
