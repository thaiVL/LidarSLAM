#include "data_parser.hpp"
#include <iostream>

int main() {
    std::vector<CLFMessage> messages = parseFile("datasets/slam_benchmarking/Intel_Research_Lab.clf");
    std::cout << "Parsed " << messages.size() << " messages" << std::endl;
    
}