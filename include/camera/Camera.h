#pragma once

#include <opencv2/videoio.hpp>

class Camera {
public:
    explicit Camera(const std::string &filePath);

    cv::Mat GetFrame(float x, float y, int direction);

    cv::RotatedRect GetRoi();

    cv::Mat GetArea();

private:
    cv::Mat area;
    cv::RotatedRect roi;
};
