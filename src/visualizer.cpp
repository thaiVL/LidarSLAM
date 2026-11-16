#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <optional>
#include "types.hpp"
#include "util.hpp"
#include "simulator.hpp"

void visualizePoints(const std::vector<std::pair<double, double>>& points, const World* world, const Pose2D* robot) {
    const int width = 800;
    const int height = 800;

    // Scale so that everything drawn fits; the origin is at the image center.
    double maxDist = 1.0;
    for (const auto& [x, y] : points) maxDist = std::max(maxDist, std::hypot(x, y));
    if (world) {
        for (const Line& line : world->lines) {
            maxDist = std::max(maxDist, std::hypot(line.x0, line.y0));
            maxDist = std::max(maxDist, std::hypot(line.x1, line.y1));
        }
    }
    const double scale = 0.45 * std::min(width, height) / maxDist;  // pixels per meter

    const int centerX = width / 2;
    const int centerY = height / 2;

    // One world-to-pixel mapping for every layer (y flipped so +y is up).
    auto toPixel = [&](double x, double y) {
        return cv::Point(centerX + static_cast<int>(std::lround(x * scale)),
                         centerY - static_cast<int>(std::lround(y * scale)));
    };

    cv::Mat image(height, width, CV_8UC3, cv::Scalar(0, 0, 0));

    // Axes through the world origin.
    cv::line(image, cv::Point(0, centerY), cv::Point(width, centerY), cv::Scalar(70, 70, 70));
    cv::line(image, cv::Point(centerX, 0), cv::Point(centerX, height), cv::Scalar(70, 70, 70));

    if (world) {
        for (const Line& line : world->lines) {
            cv::line(image, toPixel(line.x0, line.y0), toPixel(line.x1, line.y1),
                     cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        }
    }

    for (const auto& [x, y] : points) {
        cv::Point p = toPixel(x, y);
        if (p.x < 0 || p.x >= width || p.y < 0 || p.y >= height) continue;
        cv::circle(image, p, 2, cv::Scalar(0, 255, 0), -1);
    }

    if (robot) {
        cv::Point rp = toPixel(robot->x, robot->y);
        cv::Point tip = toPixel(robot->x + 0.5 * std::cos(robot->theta),
                                robot->y + 0.5 * std::sin(robot->theta));
        cv::circle(image, rp, 5, cv::Scalar(0, 0, 255), -1);
        cv::line(image, rp, tip, cv::Scalar(0, 0, 255), 2);   // heading
    }

    cv::imshow("LiDAR Scan", image);
    cv::waitKey(0);
}


void replayScans(const RobotState& state, bool accumulate = false, double speed = 1.0) {
    const int W = 1200, H = 900;
    const double margin = 30.0;

    const std::vector<CLFMessage> messages = state.messages;
    std::vector<const LaserScan*> scans;
    scans.reserve(messages.size());

    for (const auto& msg: messages) {
        if (const auto& scan = std::get_if<LaserScan>(&msg)) {
            scans.push_back(scan);
        }
    }
    
    std::vector<std::vector<Point>> world = std::vector<std::vector<Point>>(scans.size());
    double minX = 1e18, maxX = -1e18, minY = 1e18, maxY = -1e18;
    
    for (size_t i = 0; i < scans.size(); i += 1) {
        const double maxRange = (scans[i]->type == LaserType::Rear) ? state.rearMaxRange : state.frontMaxRange;
        for (auto [x, y] : scans[i]->toCartesianPoints(maxRange)) {
            Point p = transformPoint(scans[i]->robot, x, y);
            world[i].push_back(p);
            minX = std::min(minX, p.first);  maxX = std::max(maxX, p.first);
            minY = std::min(minY, p.second); maxY = std::max(maxY, p.second);
        }
    }

    const double scale = std::min((W - (2 * margin)) / (maxX - minX), (H - (2 * margin)) / (maxY - minY));
    auto toPx = [&](double x, double y) {
        return cv::Point(static_cast<int>(std::lround(margin + (x - minX) * scale)),
                         static_cast<int>(std::lround(H - margin - (y - minY) * scale)));
    };

    cv::Mat trail(H, W, CV_8UC3, cv::Scalar(0, 0, 0));   // accumulated points
    bool paused = false;
    size_t i = 0;
    while (i < scans.size()) {
        cv::Mat frame;
        if (accumulate) {
            for (auto [x, y] : world[i]) {
                cv::Point p = toPx(x, y);
                if (p.x >= 0 && p.x < W && p.y >= 0 && p.y < H)
                    trail.at<cv::Vec3b>(p) = cv::Vec3b(0, 120, 0);   // dim green
            }
            frame = trail.clone();
        } else {
            frame = cv::Mat(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
        }

        for (auto [x, y] : world[i]) {
            cv::Point p = toPx(x, y);
            if (p.x >= 0 && p.x < W && p.y >= 0 && p.y < H)
                cv::circle(frame, p, 1, cv::Scalar(0, 255, 0), -1);
        }
        const Pose2D& r = scans[i]->robot;
        cv::Point rp = toPx(r.x, r.y);
        cv::Point tip = toPx(r.x + 0.6 * std::cos(r.theta), r.y + 0.6 * std::sin(r.theta));
        cv::circle(frame, rp, 4, cv::Scalar(0, 0, 255), -1);
        cv::line(frame, rp, tip, cv::Scalar(0, 0, 255), 2);

        cv::putText(frame, "scan " + std::to_string(i) + "/" + std::to_string(scans.size()),
                    cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
        cv::imshow("Replay", frame);

        // Delay from the real timestamps so playback runs at (speed x) real time.
        double dt = (i + 1 < scans.size())
            ? scans[i+1]->metadata.loggerTimestamp - scans[i]->metadata.loggerTimestamp : 0.0;
        int delayMs = std::clamp(static_cast<int>(1000.0 * dt / speed), 1, 500);

        int key = cv::waitKey(paused ? 0 : delayMs) & 0xFF;
        if (key == 'q' || key == 27) break;
        if (key == ' ') { paused = !paused; continue; }   // toggle pause
        if (key == '+' || key == '=') speed *= 1.5;
        if (key == '-') speed /= 1.5;
        if (key == 'a' && paused && i > 0) { --i; continue; }  // step back while paused
        ++i;
    }
}