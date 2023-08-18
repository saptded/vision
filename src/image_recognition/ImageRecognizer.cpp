#include "image_recognition/ImageProcessor.h"

#include "exception/BaseException.h"

#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

namespace vision::image {
    ImageRecognizer::ImageRecognizer() = default;

    std::vector<std::pair<cv::Point, cv::Point>>
    ImageRecognizer::filterContours(std::vector<std::vector<cv::Point>> &contours, cv::Mat &frame) {
        std::vector<std::vector<cv::Point>> new_contours;
        double totalLength = 0;
        contours.erase(std::remove_if(contours.begin(), contours.end(), [](const std::vector<cv::Point> &c) {
                                          return c.size() < 5;
                                      }
                       ),
                       contours.end());
        for (const auto & contour : contours) {
            double length = cv::arcLength(contour, false);
            if (length > (frame.cols + frame.rows) * 3) {
            } else {
                new_contours.push_back(contour);
                totalLength += length;
            }
        }
        std::cout << std::endl;

        std::vector<cv::Vec4i> lines;
        HoughLinesP(frame, lines, 1, 2 * CV_PI / 180, 50, 1, 10);

        cv::Mat cdstP;
        cvtColor(frame, cdstP, cv::COLOR_GRAY2BGR);
        for (size_t i = 0; i < lines.size(); i++) {
            cv::Vec4i l = lines[i];
            line(cdstP, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
        }
        if (totalLength > (frame.cols + frame.rows) * 5) {
            contours = std::vector<std::vector<cv::Point>>{};
        }

        std::vector<std::pair<cv::Point, cv::Point>> convertedLines;
        for (auto line: lines) {
            std::pair<cv::Point, cv::Point> convertedLine{
                    cv::Point{line[0], line[1]},
                    cv::Point{line[2], line[3]}
            };
            convertedLines.push_back(convertedLine);
        }

        contours = new_contours;
        return convertedLines;
    }

    std::vector<std::pair<cv::Point, cv::Point>>
    ImageRecognizer::recognize(cv::Mat &frame, const std::string &imagePath) {
        cv::Mat ROI(frame, cv::Rect(10, 10, frame.cols - 30, frame.rows - 30));
        ROI.copyTo(frame);

        imshow("image", ROI);
        auto begin = std::chrono::steady_clock::now();
        cv::Mat preparedImage = prepareImage(frame);
        cv::Mat processedImage = cannyImage(preparedImage);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(processedImage, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        cv::Mat contourImage(processedImage.size(), CV_8UC4, cv::Scalar(0, 0, 0));
        for (size_t idx = 0; idx < contours.size(); idx++) {
            cv::drawContours(processedImage, contours, idx, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
            cv::putText(processedImage, std::to_string(idx), contours[idx][5], cv::FONT_HERSHEY_SIMPLEX, 0.4,
                        {255, 255, 255});
        }
//        imshow("frame_1", processedImage);

        for (auto &contour: contours) {
            auto sm = cv::arcLength(contour, false);
            auto dst = contour;
            cv::approxPolyDP(contour, dst, 0.0005 * sm, false);
            contour = dst;
        }


        std::vector<std::pair<cv::Point, cv::Point>> lines = filterContours(contours, processedImage);
        for (auto line: lines) {
            cv::circle(preparedImage, line.first, 2, cv::Scalar(0, 128, 0), -1);
            cv::circle(preparedImage, line.second, 2, cv::Scalar(0, 128, 0), -1);
        }

        for (size_t idx = 0; idx < contours.size(); idx++) {
            cv::drawContours(contourImage, contours, idx, cv::Scalar(255, 0, 0), 1, cv::LINE_AA);
            cv::putText(contourImage, std::to_string(idx), contours[idx][5], cv::FONT_HERSHEY_SIMPLEX, 0.4,
                        {0, 255, 255});
        }

        cv::Mat concat;
        cv::hconcat(preparedImage, processedImage, concat);

        return lines;
    }

    void ImageRecognizer::RecognizeFromPattern(const std::string &pattern) {
        std::vector<cv::String> imagePaths;
        cv::glob(pattern, imagePaths);

        for (auto &imagePath: imagePaths) {
            cv::Mat frame;
            frame = cv::imread(imagePath, cv::IMREAD_COLOR);

            recognize(frame, imagePath);

            cv::waitKey(0);
            cv::destroyAllWindows();
        }
    }

    std::vector<std::pair<cv::Point, cv::Point>> ImageRecognizer::Recognize(const cv::Mat &mat) {
        cv::Mat frame = mat;

        return recognize(frame, "imagePath");
    }

    cv::Mat ImageRecognizer::cannyImage(const cv::Mat &frame) {
        cv::Mat canny;
        cv::Canny(frame, canny, 50, 150, 3);

        return canny;
    }

    cv::Mat ImageRecognizer::prepareImage(const cv::Mat &frame) {
        cv::Mat new_frame, blur;
        cv::cvtColor(frame, new_frame, cv::COLOR_RGB2GRAY);

//        GaussianBlur(new_frame, blur, cv::Size(5, 5), 0);
        medianBlur(new_frame, blur, 21);
        return blur;
    }

} // namespace vision::image
