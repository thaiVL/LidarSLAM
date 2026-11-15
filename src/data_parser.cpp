#include "types.hpp"
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <unordered_map>
#include <functional>
#include <cmath>
#include <filesystem>

double parseDouble(std::istringstream& iss) {
    std::string token;

    if (!(iss >> token)) {
        throw std::runtime_error("Unexpected end of message");
    }

    std::size_t pos = 0;
    double value = std::stod(token, &pos);

    if (pos != token.length()) {
        throw std::runtime_error("Invalid numeric value: " + token);
    }

    if (!std::isfinite(value)) {
        throw std::runtime_error("Non-finite numeric value: " + token);
    }

    return value;
}

int parseInt(std::istringstream& iss) {
    std::string token;

    if (!(iss >> token)) {
        throw std::runtime_error("Unexpected end of message");
    }

    std::size_t pos = 0;
    int value = std::stoi(token, &pos);

    if (pos != token.length()) {
        throw std::runtime_error("Invalid integer: " + token);
    }

    return value;
}

LaserScan parseLaserScan(std::istringstream& iss, LaserType type) {
    LaserScan scan = LaserScan();
    scan.type = type;

    int numReadings = parseInt(iss);
    if (numReadings < 0) {
        throw std::runtime_error("Negative number of readings");
    }
    scan.ranges.reserve(numReadings);

    for (int i = 0; i < numReadings; i += 1) {
        double reading = parseDouble(iss);
        scan.ranges.push_back(reading);
    }

    scan.robot = Pose2D{parseDouble(iss), parseDouble(iss), parseDouble(iss)};
    scan.odom = Pose2D{parseDouble(iss), parseDouble(iss), parseDouble(iss)};

    double ipcTimestamp = parseDouble(iss);
    std::string ipcHostname;
    iss >> ipcHostname;
    double loggerTimestamp = parseDouble(iss);
    scan.metadata = MessageMetadata{ipcTimestamp, ipcHostname, loggerTimestamp};

    std::string extra;
    if (iss >> extra) {
        throw std::runtime_error("Unexpected trailing field: " + extra);
    }

    return scan;
}

Odometry parseOdometry(std::istringstream& iss) {
    double x = parseDouble(iss);
    double y = parseDouble(iss);
    double theta = parseDouble(iss);
    Pose2D pose = {x, y, theta};

    double transVel = parseDouble(iss);
    double rotVel = parseDouble(iss);
    double accel = parseDouble(iss);

    double ipcTimestamp = parseDouble(iss);
    std::string ipcHostname; iss >> ipcHostname;
    double loggerTimestamp = parseDouble(iss);
    MessageMetadata metaata{ipcTimestamp, ipcHostname, loggerTimestamp};

    return Odometry{metaata, pose, transVel, rotVel, accel};
}

RobotState parseFile(const std::string& filePath) {
    std::ifstream file(filePath);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + filePath);
    }

    std::string line;
    std::vector<CLFMessage> messages;
    using Parser = std::function<CLFMessage(std::istringstream&)>;

    std::unordered_map<std::string, Parser> dictFunc = {
        {"RLASER", [](std::istringstream& iss) -> CLFMessage {
            return parseLaserScan(iss, LaserType::Rear);
        }},
        {"FLASER", [](std::istringstream& iss) -> CLFMessage {
            return parseLaserScan(iss, LaserType::Front);
        }},
        {"ODOM", [](std::istringstream& iss) -> CLFMessage {
            return parseOdometry(iss);
        }}
    };

    std::unordered_map<std::string, int> skipped;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string type;
        iss >> type;
        if (dictFunc.count(type) > 0) {
            messages.push_back(dictFunc[type](iss));
        } else {
            skipped[type] += 1;
        }
    }
    
    int totalSkipped = 0;
    for (const std::pair<std::string, int>& entry: skipped) {
        std::cout << entry.first << " not supported, skipped " << entry.second << " entries." << std::endl;
        totalSkipped += entry.second;
    }

    std::cout << "Skipped a total of " << totalSkipped << " entries." << std::endl;
    double rearMaxRange = 0;
    double frontMaxRange = 0;
    for (const auto& msg: messages) {
        std::visit([&](const auto& data) {
            using T = std::decay_t<decltype(data)>;
            if constexpr (std::is_same_v<T, LaserScan>) {
                const LaserScan& scan = data;
                if (scan.type == LaserType::Rear) {
                    rearMaxRange = std::max(*std::max_element(scan.ranges.begin(), scan.ranges.end()), rearMaxRange);
                } else if (scan.type == LaserType::Front) {
                    frontMaxRange = std::max(*std::max_element(scan.ranges.begin(), scan.ranges.end()), frontMaxRange);
                }
            }
        }, msg);
    }

    RobotState state = {messages, frontMaxRange, rearMaxRange};

    return state;
}