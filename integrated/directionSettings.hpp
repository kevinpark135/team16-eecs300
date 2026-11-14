#pragma once

#include "directionCounter.hpp"

namespace lidar {
// Per-sensor mounting settings for the existing integrated sketch.
inline direction_counter::DirectionConfig leftSettings() {
  direction_counter::DirectionConfig config;
  config.axis = direction_counter::COLUMNS;
  config.reverse = true; // Reverse the left sensor's axis to match its mounting.
  return config;
}
inline direction_counter::DirectionConfig rightSettings() {
  direction_counter::DirectionConfig config;
  config.axis = direction_counter::COLUMNS;
  config.reverse = false; // Keep the right sensor's native axis direction.
  return config;
}
}
