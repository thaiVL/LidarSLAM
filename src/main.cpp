#include "data_parser.hpp"
#include "visualizer.hpp"
#include <iostream>

int main() {
    std::vector<CLFMessage> messages = parseFile("datasets/slam_benchmarking/Intel_Research_Lab.clf");
    std::cout << "Parsed " << messages.size() << " messages" << std::endl;
    if (std::holds_alternative<LaserScan>(messages[0])) {
        LaserScan& scan = std::get<LaserScan>(messages[0]);
        std::vector<std::pair<double, double>> pts = scan.toCartesianPoints();
        visualizeScan(pts);
        std::cout << "";
    }
    
}