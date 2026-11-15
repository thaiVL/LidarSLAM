#include <utility>
#include <types.hpp>

Point transformPoint(const Pose2D& pose, double x, double y) {
    const double c = std::cos(pose.theta), s = std::sin(pose.theta);
    return { c * x - s * y + pose.x,
             s * x + c * y + pose.y };
}