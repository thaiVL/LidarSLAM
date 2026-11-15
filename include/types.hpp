#pragma once

#include <vector>
#include <string>
#include <variant>
#include <math.h>

struct MessageMetadata {
    double ipcTimestamp;
    std::string hostname;
    double loggerTimestamp;
};

enum class LaserType {
    Front,
    Rear
};

struct Pose2D {
    double x;
    double y;
    double theta;
};

struct LaserScan {
    LaserType type;
    MessageMetadata metadata;
    Pose2D odom;
    Pose2D robot;
    std::vector<double> ranges;
    double angleIncrement = M_PI / 180.0;  // rad

    double beamAngle(size_t i) const {
        double angleMin = (type == LaserType::Front) ? -M_PI / 2.0 : M_PI / 2.0;
        return angleMin + i * angleIncrement;
    }

    std::vector<std::pair<double, double>> toCartesianPoints(double maxRange) const {
        std::vector<std::pair<double, double>> points;
        points.reserve(ranges.size());

        for (std::size_t i = 0; i < ranges.size(); i += 1) {
            if (ranges[i] >= maxRange - 1e-3) {
                continue;
            }
            
            double theta = beamAngle(i);
            double x = ranges[i] * std::cos(theta);
            double y = ranges[i] * std::sin(theta);
            points.emplace_back(x, y); 
        }

        return points;
    }
};

struct Odometry {
    MessageMetadata metadata;
    Pose2D pose;
    double transVel;
    double rotVel;
    double accel;
};

using CLFMessage = std::variant<LaserScan, Odometry>;

struct RobotState {
    std::vector<CLFMessage> messages;
    double frontMaxRange;
    double rearMaxRange;
};