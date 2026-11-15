#include "data_parser.hpp"
#include "visualizer.hpp"
#include <iostream>

int main() {
    RobotState state = parseFile("datasets/slam_benchmarking/Intel_Research_Lab.clf");
    // std::cout << "Parsed " << state.messages.size() << " messages" << std::endl;
    // if (std::holds_alternative<LaserScan>(state.messages[0])) {
    //     LaserScan& scan = std::get<LaserScan>(state.messages[0]);
    //     std::vector<std::pair<double, double>> pts = scan.toCartesianPoints(scan.type == LaserType::Rear ? state.rearMaxRange : state.frontMaxRange);
    //     visualizeScan(pts);
    //     std::cout << "";
    // }

    // std::vector<Point> points;
    // for (const auto& msg: state.messages) {
    //     std::visit([&](const auto& data) {
    //         using T = std::decay_t<decltype(data)>;
    //         if constexpr (std::is_same_v<T, LaserScan>) {
    //             const LaserScan& scan = data;
    //             std::vector<Point> pts = scan.toCartesianPoints(scan.type == LaserType::Rear ? state.rearMaxRange : state.frontMaxRange);
    //             points.insert(points.end(), pts.begin(), pts.end());
    //         }
    //     }, msg);
    // }
    // visualizeScan(points);
    replayScans(state, false, 50.0);
}