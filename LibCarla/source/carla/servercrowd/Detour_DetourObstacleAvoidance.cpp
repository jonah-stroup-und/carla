// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

// Compiles Recast's DetourObstacleAvoidance.cpp inside carla::servercrowd::detour (see Detour.h).
#include "carla/servercrowd/Detour.h"

#ifdef _MSC_VER
#pragma warning(push, 0)
#endif

namespace carla {
namespace servercrowd {
namespace detour {
#include <../Source/DetourObstacleAvoidance.cpp>
} // namespace detour
} // namespace servercrowd
} // namespace carla

#ifdef _MSC_VER
#pragma warning(pop)
#endif
