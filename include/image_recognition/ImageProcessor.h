#pragma once

#include <memory>
#include <opencv2/videoio.hpp>

namespace vision::image {
class ImageRecognizer {
 public:
    explicit ImageRecognizer();
    void RecognizeFromPattern(const std::string& path);
    bool Recognize(const cv::Mat&);

private:
    bool recognize(cv::Mat &, const std::string& path);
    cv::Mat cannyImage(const cv::Mat&);
    cv::Mat prepareImage(const cv::Mat&);
    void filterCounters(std::vector<std::vector<cv::Point>>&, cv::Mat &);
};
}  // namespace vision::image
