#pragma once
#include <vector>
#include <optional>
#include "simulator.hpp"

void visualizePoints(const std::vector<std::pair<double, double>>& points, const World* world = nullptr, const Pose2D* robot = nullptr);
void replayScans(const RobotState& state, bool accumulate = false, double speed = 1.0);
