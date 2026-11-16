#include "simulator.hpp"
#include "types.hpp"

LaserScan simulateScan(const World& world, LaserType type, const Pose2D& pose, double timestamp, int numBeams, double maxRange) {
    LaserScan scan;
    scan.type = type;
    scan.robot = pose;
    scan.odom = pose;
    scan.metadata = MessageMetadata{timestamp, "simulator", timestamp};
    scan.angleIncrement = M_PI / numBeams;
    scan.ranges.reserve(numBeams);
    for (int i = 0; i < numBeams; i += 1) {
        double worldAngle = pose.theta + scan.beamAngle(i);
        double range = world.castRay(pose.x, pose.y, worldAngle, maxRange);
        scan.ranges.push_back(range);
    }
    return scan;
}