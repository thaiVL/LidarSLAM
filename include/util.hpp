#pragma once

#include <utility>
#include "types.hpp"

std::pair<double, double> transformPoint(const Pose2D& pose, double x, double y);