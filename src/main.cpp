#include <iostream>
#include "data_parser.hpp"
#include "visualizer.hpp"
#include "simulator.hpp"
#include "util.hpp"

void simulate() {
    World world;
    world.addRect(0, 0, 10, 8);
    world.addRect(4, 3, 6, 5);
    world.addRect(7.5, 6, 9, 7.5);

    Pose2D pose{2, 2, 0};
    LaserScan s = simulateScan(world, LaserType::Front, pose, 0);
    std::vector<Point> worldPoints;
    for (const auto& [x, y] : s.toCartesianPoints(80)) {
        worldPoints.push_back(transformPoint(pose, x, y));
    }
    visualizePoints(worldPoints, &world, &pose);
}

void visualizeOneMessage(const RobotState& state, int i) {
    std::visit(overloaded {
        [&](const LaserScan& scan) {
            double maxRange = (scan.type == LaserType::Rear) 
                ? state.rearMaxRange 
                : state.frontMaxRange;
            visualizePoints(scan.toCartesianPoints(maxRange), nullptr, &scan.robot);
        },

        [&](const Odometry& odom) {
            std::cout << "Cannot visualize the " << i << "th message, it is odometry" << std::endl;
        }
    }, state.messages[i]);
}


int main() {
    RobotState state = parseFile("datasets/slam_benchmarking/Intel_Research_Lab.clf");

    // replayScans(state, false, 50.0);
    // simulate();
    visualizeOneMessage(state, 1);
}