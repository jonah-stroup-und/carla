// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

// CARLA's Recast/Detour fork, compiled inside carla::servercrowd::detour.
//
// Unreal Engine ships its own, modified Detour (the Navmesh module) with the
// same class and function names (dtNavMesh, dtCrowd, dtAlloc...) but other
// layouts, and CARLA's fork adds fields of its own (dtCrowdAgent::paused,
// dead, dtCrowdAgentParams::useObb). Linked into the same binary, the global
// names would collide or, worse, silently bind to the engine's copy. In a
// namespace they are distinct symbols. The sources are compiled the same way,
// by the Detour*.cpp files in this directory.
//
// Include this header only from LibCarla sources, never next to Unreal's
// navigation headers: both sets use the same include guards.

// Everything Detour includes from the standard library, first, so the
// include guards keep it out of the namespace below.
#include <assert.h>
#include <float.h>
#include <math.h>
#include <new>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace carla {
namespace servercrowd {
namespace detour {

#include <DetourAlloc.h>
#include <DetourAssert.h>
#include <DetourStatus.h>
#include <DetourMath.h>
#include <DetourCommon.h>
#include <DetourNavMesh.h>
#include <DetourNavMeshBuilder.h>
#include <DetourNavMeshQuery.h>
#include <DetourNode.h>
#include <DetourLocalBoundary.h>
#include <DetourObstacleAvoidance.h>
#include <DetourPathCorridor.h>
#include <DetourPathQueue.h>
#include <DetourProximityGrid.h>
#include <DetourCrowd.h>

} // namespace detour

  using namespace detour;

} // namespace servercrowd
} // namespace carla
