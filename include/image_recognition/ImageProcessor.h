#pragma once

#include <memory>
#include <opencv2/videoio.hpp>

namespace vision::image {
class ImageRecognizer {
 public:
    explicit ImageRecognizer();
    void RecognizeFromPattern(const std::string& path);
    std::vector<std::pair<cv::Point, cv::Point>> Recognize(const cv::Mat&);

private:
    std::vector<std::pair<cv::Point, cv::Point>> recognize(cv::Mat &, const std::string& path);
    cv::Mat cannyImage(const cv::Mat&);
    cv::Mat prepareImage(const cv::Mat&);
    std::vector<std::pair<cv::Point, cv::Point>> filterContours(std::vector<std::vector<cv::Point>>&, cv::Mat &);
};
}  // namespace vision::image
