#include <iostream>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

int CHECKERBOARD[2]{6, 9};

int main() {
    std::vector<std::vector<cv::Point3f>> objpoints;

    std::vector<std::vector<cv::Point2f>> imgpoints;

    std::vector<cv::Point3f> objp;
    for (int i{0}; i < CHECKERBOARD[1]; i++) {
        for (int j{0}; j < CHECKERBOARD[0]; j++)
            objp.emplace_back(j, i, 0);
    }

    std::vector<cv::String> images;
    std::string path = "../../calibration_images/*.jpg";

    cv::glob(path, images);

    cv::Mat frame, gray;
    std::vector<cv::Point2f> corner_pts;
    bool success;

    for (auto & image : images) {
        frame = cv::imread(image);
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        success = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts,
                                            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

        if (success) {
            cv::TermCriteria criteria(cv::TermCriteria::Type::EPS | cv::TermCriteria::Type::MAX_ITER, 30, 0.001);
            cv::cornerSubPix(gray, corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);
            cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);

            objpoints.push_back(objp);
            imgpoints.push_back(corner_pts);
        }

        cv::imshow("Image", frame);
        cv::waitKey(0);
    }

    cv::destroyAllWindows();

    cv::Mat cameraMatrix, distCoeffs, R, T;
    cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows, gray.cols), cameraMatrix, distCoeffs, R, T);

    std::cout << "cameraMatrix : " << cameraMatrix << std::endl;
    std::cout << "distCoeffs : " << distCoeffs << std::endl;
    return 0;
}
