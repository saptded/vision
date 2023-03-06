#include "image_recognition/ImageProcessor.h"

#include "exception/BaseException.h"

#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

namespace vision::image {
    ImageProcessor::ImageProcessor() = default;

    void ImageProcessor::filterCounters(std::vector<std::vector<cv::Point>> &contours) {
        contours.erase(std::remove_if(contours.begin(), contours.end(), [](const std::vector<cv::Point> &c) {
                                          return c.size() < 90;
                                      }
                       ),
                       contours.end());

        for (auto contour: contours) {
            for (auto point: contour) {

            }
        }
    }

    void ImageProcessor::ProcessFromPattern(const std::string &pattern) {
        std::vector<cv::String> imagePaths;
        cv::glob(pattern, imagePaths);

        for (auto &imagePath: imagePaths) {
            cv::Mat frame;
            frame = cv::imread(imagePath, cv::IMREAD_COLOR);
            cv::Mat preparedImage = prepareImage(frame);
            cv::Mat processedImage = processImage(preparedImage);

            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(processedImage, contours, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
            cv::Mat contourImage(processedImage.size(), CV_8UC4, cv::Scalar(0, 0, 0));
            cv::Scalar colors[3];
            colors[0] = cv::Scalar(255, 0, 0);
            colors[1] = cv::Scalar(0, 255, 0);
            colors[2] = cv::Scalar(0, 0, 255);

            filterCounters(contours);
            for (size_t idx = 0; idx < contours.size(); idx++) {
                cv::drawContours(contourImage, contours, idx, colors[idx % 3]);
                cv::putText(contourImage, std::to_string(idx), contours[idx][0], cv::FONT_HERSHEY_SIMPLEX, 0.4,
                            {0, 255, 255});
            }

            if (contours.size() > 4) {
                cv::circle(preparedImage, cv::Point(preparedImage.cols - 50, 20), 10, cv::Scalar(0, 128, 0), -1);
            } else {
                cv::circle(preparedImage, cv::Point(preparedImage.cols - 50, 20), 10, cv::Scalar(255, 0, 0), -1);
            }

            imshow("con", contourImage);
            cv::putText(preparedImage, imagePath, cv::Point(20, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        {0, 255, 255});
            cv::Mat concat;
            cv::hconcat(preparedImage, processedImage, concat);
            imshow("concat", concat);
            cv::waitKey(0);
            cv::destroyAllWindows();
        }
    }

    cv::Mat ImageProcessor::processImage(const cv::Mat &frame) {
        cv::Mat otsu = otsu_threshold(frame);

        cv::Mat canny;
        cv::Canny(frame, canny, 10, 50, 3);

        return canny;
    }

    cv::Mat ImageProcessor::prepareImage(const cv::Mat &frame) {
        cv::Mat new_frame, blur;
        cv::cvtColor(frame, new_frame, cv::COLOR_RGB2GRAY);
//        medianBlur( new_frame, blur, 7);
        GaussianBlur(new_frame, blur, cv::Size(5, 5), 0);


//        cv::Mat imageContrastHigh2;
//        frame.convertTo(imageContrastHigh2, -1, 1.5, -70);

        return blur;
    }

    void ImageProcessor::Process() {

    }


    cv::Mat ImageProcessor::otsu_threshold(const cv::Mat &img) const {
        cv::Mat blur;
        static constexpr int KERNEL_SIZE = 21;
        cv::medianBlur(img, blur, KERNEL_SIZE);

        cv::Mat otsu;
        static constexpr double THRESH = 250;
        static constexpr double MAX_VALUE = 255;
        cv::threshold(blur, otsu, THRESH, MAX_VALUE, cv::THRESH_BINARY + cv::THRESH_OTSU);
        return otsu;
    }


} // namespace vision::image
// поиск линий
//            std::vector<cv::Vec4i> linesP; // will hold the results of the detection
//            HoughLinesP(processedImage, linesP, 1, CV_PI/180, 50, 50, 10 ); // runs the actual detection
//            cv::Mat dst, cdst, cdstP;
//            for( size_t i = 0; i < linesP.size(); i++ )
//            {
//                cv::Vec4i l = linesP[i];
//                line( contourImage, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, cv::LINE_AA);
//            }