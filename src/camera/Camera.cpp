#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "camera/Camera.h"

Camera::Camera(const std::string &filePath) {
    area = cv::imread(filePath, cv::IMREAD_COLOR);
}

cv::Mat Camera::GetFrame(float x, float y, int direction) {
    float division = 16;
    float side = float(area.cols) / division;

    bool collisionX = false;
    bool collisionY = false;
    float collisionCoordinateX;
    float collisionCoordinateY;
    float radius = std::ceil(side / 2);
    if ((x + radius) > float(area.cols)) {
        collisionCoordinateX = float(area.cols) - radius;
        collisionX = true;
    } else if (x < 0) {
        collisionCoordinateX = 0;
        collisionX = true;
    }
    if ((y + radius) > float(area.rows)) {
        collisionCoordinateY = float(area.rows) - radius;
        collisionY = true;
    } else if (y < 0) {
        collisionCoordinateY = 0;
        collisionY = true;
    }

    if (collisionX) {
        roi.center.x = float(collisionCoordinateX);
    } else {
        roi.center.x = x;
    }
    if (collisionY) {
        roi.center.y = collisionCoordinateY;
    } else {
        roi.center.y = y;
    }
    roi.size = cv::Size2f(side, side);
    roi.angle = float(direction);

    cv::Mat M, rotated, cropped;
    float angle = roi.angle;
    cv::Size rect_size = roi.size;
    angle += 90.0;

    M = getRotationMatrix2D(roi.center, angle, 1.0);
    warpAffine(area, rotated, M, area.size(), cv::INTER_CUBIC);
    getRectSubPix(rotated, rect_size, roi.center, cropped);

    cv::Mat frame = cropped;
    return frame;
}

cv::RotatedRect Camera::GetRoi() {
    if (roi.size.empty()) {
        return {};
    }
    return roi;
}

cv::Mat Camera::GetArea() {
    if (area.empty()) {
        return {};
    }
    return area;
}
