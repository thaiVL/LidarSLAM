#pragma once

#include <utility>
#include "types.hpp"

template<class... Ts>
struct overloaded : Ts... { using Ts::operator()...; };

template<class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

std::pair<double, double> transformPoint(const Pose2D& pose, double x, double y);