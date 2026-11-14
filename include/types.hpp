#include <vector>
#include <string>
#include <variant>

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
};

struct Odometry {
    MessageMetadata metadata;
    Pose2D pose;
    double transVel;
    double rotVel;
    double accel;
};

using CLFMessage = std::variant<LaserScan, Odometry>;
