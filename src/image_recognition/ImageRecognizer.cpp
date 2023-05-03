#include "image_recognition/ImageProcessor.h"

#include "exception/BaseException.h"

#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

namespace vision::image {
    ImageRecognizer::ImageRecognizer() = default;

    std::vector<std::pair<cv::Point, cv::Point>> ImageRecognizer::filterContours(std::vector<std::vector<cv::Point>> &contours, cv::Mat &frame) {
        std::vector<std::vector<cv::Point>> new_contours;
        double totalLength = 0;
        std::cout << "NEW FRAME: contours total amount: " << contours.size() << std::endl;

        contours.erase(std::remove_if(contours.begin(), contours.end(), [](const std::vector<cv::Point> &c) {
//                                          std::cout << "counter size " << c.size() << " ";
                                          return c.size() < 5;
                                      }
                       ),
                       contours.end());
        std::cout << std::endl;
        for (int i = 0; i < contours.size(); i++) {
            double length = cv::arcLength(contours[i], false);
//            std::cout << "counter length: " << length << " ";
            if (length > (frame.cols + frame.rows) * 3) {
//                std::cout << std::endl << "contour killed: length: " << length << "; max: " << (frame.cols + frame.rows)
//                          << std::endl;
            } else {
                new_contours.push_back(contours[i]);
                totalLength += length;
            }
        }
        std::cout << std::endl;

        cv::Mat cdstP;
        cvtColor(frame, cdstP, cv::COLOR_GRAY2BGR);

        std::vector<cv::Vec4i> lines;
        HoughLinesP(frame, lines, 1, 2 * CV_PI / 180, 20, 1, 5);
//        std::cout << "lines size: " << lines.size() << std::endl;

//        lines.erase(std::remove_if(lines.begin(), lines.end(), [](const cv::Vec4i &l) {
//                                       int length = sqrt(pow(l[0] - l[1], 2) - pow(l[2] - l[3], 2));
//                                       return length < 5;
//                                   }
//                    ),
//                    lines.end());
//        std::cout << "lines filtered size: " << lines.size() << std::endl;


        for (size_t i = 0; i < lines.size(); i++) {
            cv::Vec4i l = lines[i];
            line(cdstP, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
        }


        imshow("lines", cdstP);
        std::cout << totalLength << "|||" << (frame.cols + frame.rows) * 5 << std::endl;
        if (totalLength > (frame.cols + frame.rows) * 5) {
            contours = std::vector<std::vector<cv::Point>>{};
        } else {

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
        std::cout << "FRAME DONE: contours total amount: " << contours.size() << std::endl;
        return convertedLines;
    }

    std::vector<std::pair<cv::Point, cv::Point>> ImageRecognizer::recognize(cv::Mat &frame, const std::string &imagePath) {
        cv::Mat ROI(frame, cv::Rect(10, 10, frame.cols - 30, frame.rows - 30));
        ROI.copyTo(frame);

        auto begin = std::chrono::steady_clock::now();
        cv::Mat preparedImage = prepareImage(frame);
        cv::Mat processedImage = cannyImage(preparedImage);

        std::vector<std::vector<cv::Point>> contours;


        cv::findContours(processedImage, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        cv::Mat contourImage(processedImage.size(), CV_8UC4, cv::Scalar(0, 0, 0));
        cv::Scalar colors[3];
        colors[0] = cv::Scalar(255, 0, 0);
        colors[1] = cv::Scalar(0, 255, 0);
        colors[2] = cv::Scalar(0, 0, 255);

        for (size_t idx = 0; idx < contours.size(); idx++) {
            cv::drawContours(processedImage, contours, idx, colors[0], 2, cv::LINE_AA);
        }
        imshow("c1", processedImage);


        std::vector<std::pair<cv::Point, cv::Point>> lines = filterContours(contours, processedImage);
        for (auto line: lines) {
            cv::circle(preparedImage, line.first, 2, cv::Scalar(0, 128, 0), -1);
            cv::circle(preparedImage, line.second, 2, cv::Scalar(0, 128, 0), -1);
        }

        bool isInteresting = false;
        if (!contours.empty()) {
            isInteresting = true;
//            cv::circle(preparedImage, cv::Point(preparedImage.cols - 50, 20), 10, cv::Scalar(0, 128, 0), -1);
        } else {
//            cv::circle(preparedImage, cv::Point(preparedImage.cols - 50, 20), 10, cv::Scalar(255, 0, 0), -1);
        }

        for (size_t idx = 0; idx < contours.size(); idx++) {

            cv::drawContours(contourImage, contours, idx, colors[0], 3, cv::LINE_AA);
            cv::putText(contourImage, std::to_string(idx), contours[idx][5], cv::FONT_HERSHEY_SIMPLEX, 0.4,
                        {0, 255, 255});
        }

        imshow("contourImage", contourImage);
//        cv::putText(preparedImage, imagePath, cv::Point(20, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5,
//                    {0, 255, 255});
        cv::Mat concat;
        cv::hconcat(preparedImage, processedImage, concat);
        imshow("concat", concat);


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
        medianBlur(new_frame, blur, 9);
        return blur;
    }

} // namespace vision::image
