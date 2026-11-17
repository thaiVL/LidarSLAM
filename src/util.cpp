#include <utility>
#include <types.hpp>

Point transformPoint(const Pose2D& pose, double x, double y) {
    // transform a point (x, y) relative to robot's frame/coords to world frame/coords given pose of robot
    const double c = std::cos(pose.theta), s = std::sin(pose.theta);
    return { c * x - s * y + pose.x,
             s * x + c * y + pose.y };
}

