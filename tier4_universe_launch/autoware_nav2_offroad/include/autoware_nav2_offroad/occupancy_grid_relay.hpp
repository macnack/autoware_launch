// Copyright 2026 Maciej Krupka maciej.krupka@put.poznan.pl
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE_NAV2_OFFROAD__OCCUPANCY_GRID_RELAY_HPP_
#define AUTOWARE_NAV2_OFFROAD__OCCUPANCY_GRID_RELAY_HPP_

#include <cstdint>
#include <vector>

namespace autoware::nav2_offroad
{

/// Adapt occupancy-grid cell values for a Nav2 costmap. When `unknown_as_free`
/// is set, unknown cells (-1) are mapped to free (0) so the planner can route
/// through unobserved space; all other values pass through unchanged.
std::vector<int8_t> remapOccupancy(const std::vector<int8_t> & data, bool unknown_as_free);

}  // namespace autoware::nav2_offroad

#endif  // AUTOWARE_NAV2_OFFROAD__OCCUPANCY_GRID_RELAY_HPP_
