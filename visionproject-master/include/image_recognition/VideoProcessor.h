#pragma once

#include <memory>
#include <opencv2/videoio.hpp>

namespace vision {
namespace network {
class Sender;
}

namespace video {
class VideoProcessor {
 public:
    explicit VideoProcessor();
    void Run();

 private:
    cv::Mat otsu_threshold(const cv::Mat &img) const;
    cv::Mat calculate_position(const std::vector<std::pair<cv::Point2d, cv::Point2d>> &points, int centre_x, int centre_y);
    std::vector<cv::RotatedRect> find_ellipses(const cv::Mat &img);
    std::vector<std::pair<cv::Point2d, cv::Point2d>> find_points(std::vector<cv::RotatedRect> &ellipses);
    std::vector<std::vector<cv::Point2f>> convert_contours(const cv::Mat &img, const std::vector<std::vector<cv::Point>> &contours) const;
    std::vector<std::vector<cv::Point>> check_contours(const cv::Mat &img, const std::vector<std::vector<cv::Point>> &contours) const;

    cv::VideoCapture _cap;
    cv::Mat camera_matrix;
    cv::Mat dis_coeffs;
};
}  // namespace video
}  // namespace vision
