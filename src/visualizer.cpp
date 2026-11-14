#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>

void visualizeScan(const std::vector<std::pair<double, double>>& points) {
    const int width = 800;
    const int height = 800;

    // Number of pixels representing one meter.
    double maxDist = 1.0;
    for (const auto& [x, y] : points) maxDist = std::max(maxDist, std::hypot(x, y));
    const double scale = 0.45 * std::min(width, height) / maxDist;

    // Robot's position in the image.
    const int centerX = width / 2;
    const int centerY = height / 2;

    // Create a black image.
    cv::Mat image(
        height,
        width,
        CV_8UC3,
        cv::Scalar(0, 0, 0)
    );

    // Draw coordinate axes.
    cv::line(
        image,
        cv::Point(0, centerY),
        cv::Point(width, centerY),
        cv::Scalar(70, 70, 70)
    );

    cv::line(
        image,
        cv::Point(centerX, 0),
        cv::Point(centerX, height),
        cv::Scalar(70, 70, 70)
    );

    // Draw robot.
    cv::circle(
        image,
        cv::Point(centerX, centerY),
        5,
        cv::Scalar(0, 0, 255),
        -1
    );

    // Draw LiDAR points.
    for (const auto& [x, y] : points) {
        int pixelX = centerX + static_cast<int>(x * scale);
        int pixelY = centerY - static_cast<int>(y * scale);

        // Ignore points outside the image.
        if (pixelX < 0 || pixelX >= width ||
            pixelY < 0 || pixelY >= height) {
            continue;
        }

        cv::circle(
            image,
            cv::Point(pixelX, pixelY),
            2,
            cv::Scalar(0, 255, 0),
            -1
        );
    }

    cv::imshow("LiDAR Scan", image);
    cv::waitKey(0);
    while (cv::getWindowProperty("LidarSLAM", cv::WND_PROP_AUTOSIZE) >= 0) {
        cv::waitKey(30);
    }
}