#pragma once

#include "directionCounter.hpp"

namespace lidar {
// Adjust these after checking each sensor's physical orientation.
inline direction_counter::DirectionConfig leftSettings() {
  direction_counter::DirectionConfig config;
  config.axis = direction_counter::COLUMNS;
  config.reverse = false;
  return config;
}
inline direction_counter::DirectionConfig rightSettings() {
  direction_counter::DirectionConfig config;
  config.axis = direction_counter::COLUMNS;
  config.reverse = false;
  return config;
}
}
