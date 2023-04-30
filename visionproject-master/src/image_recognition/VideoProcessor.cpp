#include "image_recognition/ImageProcessor.h"

#include "exception/BaseException.h"

#include <chrono>
#include <opencv2/opencv.hpp>

namespace vision {
namespace video {

VideoProcessor::VideoProcessor() {
    cv::VideoCapture cap(2);
    if (!cap.isOpened()) {
        throw BaseException("Cannot open the video file");
    }
    _cap = cap;
    static const std::vector<std::vector<double>> row_data = {
        {952.4204052685315, 0, 268.3456363975023}, {0, 958.7259233325196, 203.3911172814298}, {0, 0, 1}};
    camera_matrix = cv::Mat(3, 3, CV_64F);
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            camera_matrix.at<double>(i, j) = row_data[i][j];
        }
    }

    static const std::vector<double> row_dis_coeffs = {-0.1058045063637636, 1.372922033707199, -0.02106754941927615, -0.009828858225732397,
                                                       -4.270011072842507};
    dis_coeffs = cv::Mat(1, 5, CV_64F);
    for (size_t i = 0; i < 5; ++i) {
        dis_coeffs.at<double>(0, i) = row_dis_coeffs[i];
    }
    std::cout << dis_coeffs << std::endl;
    std::cout << camera_matrix << std::endl;
}

void VideoProcessor::Run() {
    cv::Mat frame;
    while (_cap.read(frame)) {
        std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
        cv::Mat new_frame;
        cv::cvtColor(frame, new_frame, cv::COLOR_BGR2GRAY);

        cv::Mat otsu = otsu_threshold(new_frame);

        std::vector<cv::RotatedRect> ellipses = find_ellipses(otsu);
        if (ellipses.empty()) {
            continue;
        }
        auto points = find_points(ellipses);

        cv::Mat coord = calculate_position(points, frame.rows / 2, frame.cols / 2);
        std::stringstream result;
        result.precision(2);
        result << "X, Y, Z: " << coord.at<double>(0, 0) << ", " << coord.at<double>(1, 0) << ", " << coord.at<double>(2, 0);

        cv::Point2d centre{(points[0].first.x + points[0].second.x) / 2, (points[0].first.y + points[0].second.y) / 2};

        cv::circle(frame, centre, 10, {0, 0, 255}, -1);
        cv::circle(frame, points[0].first, 10, {0, 0, 255}, -1);
        cv::putText(frame, "x1", points[1].first, cv::FONT_HERSHEY_SIMPLEX, 2, {0, 255, 255});
        cv::circle(frame, points[0].second, 10, {0, 0, 255}, -1);
        cv::putText(frame, "x2", points[0].first, cv::FONT_HERSHEY_SIMPLEX, 2, {0, 255, 255});
        cv::circle(frame, points[1].first, 10, {0, 0, 255}, -1);
        cv::putText(frame, "x3", points[1].second, cv::FONT_HERSHEY_SIMPLEX, 2, {0, 255, 255});
        cv::circle(frame, points[1].second, 10, {0, 0, 255}, -1);
        cv::putText(frame, "x4", points[0].second, cv::FONT_HERSHEY_SIMPLEX, 2, {0, 255, 255});

        cv::putText(frame, result.str(), {3, frame.rows - 8}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 255});

        std::chrono::time_point<std::chrono::system_clock> end = std::chrono::system_clock::now();
        std::chrono::milliseconds dur = std::chrono::duration_cast<std::chrono::milliseconds>(end - now);
        std::cout << dur.count() << std::endl;
        imshow("TestPhoto", frame);
        if (cv::waitKey(30) == 27) {
            break;
        }
    }
}

cv::Mat VideoProcessor::otsu_threshold(const cv::Mat &img) const {
    cv::Mat blur;
    static constexpr int KERNEL_SIZE = 21;
    cv::medianBlur(img, blur, KERNEL_SIZE);

    cv::Mat otsu;
    static constexpr double THRESH = 250;
    static constexpr double MAX_VALUE = 255;
    cv::threshold(blur, otsu, THRESH, MAX_VALUE, cv::THRESH_BINARY + cv::THRESH_OTSU);
    return otsu;
}

std::vector<cv::RotatedRect> VideoProcessor::find_ellipses(const cv::Mat &img) {
    cv::Mat copy_img;
    img.copyTo(copy_img);

    cv::Mat img_one_channel;
    copy_img.convertTo(img_one_channel, CV_8UC1);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(img_one_channel, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    contours.erase(std::remove_if(contours.begin(), contours.end(), [](const std::vector<cv::Point> &c) { return c.size() < 6; }),
                   contours.end());
    auto new_contours = check_contours(img, contours);

    std::vector<cv::RotatedRect> ellipses;
    cv::Mat drawing = cv::Mat::zeros(img.size(), CV_8UC3);
    for (auto &contour : new_contours) {
        if (contour.size() > 5) {
            ellipses.emplace_back(cv::fitEllipse(contour));
        }
    }
    return ellipses;
}

std::vector<std::pair<cv::Point2d, cv::Point2d>> VideoProcessor::find_points(std::vector<cv::RotatedRect> &ellipses) {
    float sum = 0;
    std::for_each(ellipses.begin(), ellipses.end(), [&sum](const cv::RotatedRect &elem) { sum += elem.angle; });
    float average_angle = sum / ellipses.size();

    std::vector<std::pair<cv::Point2d, cv::Point2d>> points;
    std::sort(ellipses.begin(), ellipses.end(),
              [](const cv::RotatedRect &a, const cv::RotatedRect &b) { return a.size.width > b.size.width; });
    for (const auto &ellipse : ellipses) {
        average_angle = ellipse.angle;
        double x1 = 0;
        double y1 = 0;
        double x3 = 0;
        double y3 = 0;
        if ((average_angle > 55 && average_angle < 125) || (average_angle < 15) || average_angle > 165) {
            x1 = std::cos(average_angle * float(M_PI) / 180) * ellipse.size.width / 2 + ellipse.center.x;
            y1 = ellipse.center.y - std::sqrt(std::pow(ellipse.size.width / 2, 2) - std::pow(x1 - ellipse.center.x, 2));
            if (average_angle < 90) {
                x3 = std::cos((90 + average_angle) * float(M_PI) / 180) * ellipse.size.height / 2 + ellipse.center.x;
            } else {
                x3 = std::cos((average_angle - 90) * float(M_PI) / 180) * ellipse.size.height / 2 + ellipse.center.x;
            }
            y3 = ellipse.center.y - std::sqrt(std::pow(ellipse.size.height / 2, 2) - std::pow(x3 - ellipse.center.x, 2));
        } else {
            x1 = std::cos(average_angle * float(M_PI) / 180) * ellipse.size.height / 2 + ellipse.center.x;
            y1 = ellipse.center.y - std::sqrt(std::pow(ellipse.size.height / 2, 2) - std::pow(x1 - ellipse.center.x, 2));
            if (average_angle < 90) {
                x3 = std::cos((90 + average_angle) * float(M_PI) / 180) * ellipse.size.width / 2 + ellipse.center.x;
            } else {
                x3 = std::cos((average_angle - 90) * float(M_PI) / 180) * ellipse.size.width / 2 + ellipse.center.x;
            }
            y3 = ellipse.center.y - std::sqrt(std::pow(ellipse.size.width / 2, 2) - std::pow(x3 - ellipse.center.x, 2));
        }

        double x2 = 2 * ellipse.center.x - x1;
        double y2 = 2 * ellipse.center.y - y1;
        double x4 = 2 * ellipse.center.x - x3;
        double y4 = 2 * ellipse.center.y - y3;
        std::pair<cv::Point2d, cv::Point2d> first;
        std::pair<cv::Point2d, cv::Point2d> second;

        if (x1 > x2 && x1 > x3 && x1 > x4) {
            first = {cv::Point2d(x1, y1), cv::Point2d(x2, y2)};
            if (y3 < y4) {
                second = {cv::Point2d(x3, y3), cv::Point2d(x4, y4)};
            } else {
                second = {cv::Point2d(x4, y4), cv::Point2d(x3, y3)};
            }
        } else if (x1 < x2 && x2 > x3 && x2 > x4) {
            first = {cv::Point2d(x2, y2), cv::Point2d(x1, y1)};
            if (y3 < y4) {
                second = {cv::Point2d(x3, y3), cv::Point2d(x4, y4)};
            } else {
                second = {cv::Point2d(x4, y4), cv::Point2d(x3, y3)};
            }
        } else if (x3 > x4 && x3 > x2 && x3 > x1) {
            first = {cv::Point2d(x3, y3), cv::Point2d(x4, y4)};
            if (y1 < y2) {
                second = {cv::Point2d(x1, y1), cv::Point2d(x2, y2)};
            } else {
                second = {cv::Point2d(x2, y2), cv::Point2d(x1, y1)};
            }
        } else {
            first = {cv::Point2d(x4, y4), cv::Point2d(x3, y3)};
            if (y1 < y2) {
                second = {cv::Point2d(x1, y1), cv::Point2d(x2, y2)};
            } else {
                second = {cv::Point2d(x2, y2), cv::Point2d(x1, y1)};
            }
        }
        points.emplace_back(first);
        points.emplace_back(second);
    }

    return points;
}

std::vector<std::vector<cv::Point2f>> VideoProcessor::convert_contours(const cv::Mat &img,
                                                                       const std::vector<std::vector<cv::Point>> &contours) const {
    static constexpr int margin = 2;
    std::vector<std::vector<cv::Point2f>> points;

    for (const auto &contour : contours) {
        size_t count = contour.size();
        if (count < 6) {
            continue;
        }

        cv::Mat pointsf;
        cv::Mat(contour).convertTo(pointsf, CV_32F);

        std::vector<cv::Point2f> pts;
        for (int j = 0; j < pointsf.rows; j++) {
            cv::Point2f pnt = cv::Point2f(pointsf.at<float>(j, 0), pointsf.at<float>(j, 1));
            if ((pnt.x > margin && pnt.y > margin && pnt.x < img.cols - margin && pnt.y < img.rows - margin)) {
                if (j % 20 == 0) {
                    pts.push_back(pnt);
                }
            }
        }
        points.push_back(pts);
    }
    return points;
}
std::vector<std::vector<cv::Point>> VideoProcessor::check_contours(const cv::Mat &img,
                                                                   const std::vector<std::vector<cv::Point>> &contours) const {
    if (contours.size() == 2) {
        return contours;
    }
    std::vector<std::vector<cv::Point>> new_contours;
    for (auto &cont : contours) {
        std::vector<cv::Point> new_cont;
        new_cont.reserve(cont.size());
        for (const auto &p : cont) {
            if (p.x == 0 || p.x == img.cols - 1 || p.y == 0 || p.y == img.rows - 1) {
                if (!new_cont.empty()) {
                    new_contours.emplace_back(new_cont);
                    new_cont.clear();
                }
                continue;
            }
            new_cont.emplace_back(p);
        }
    }

    return new_contours;
}
cv::Mat VideoProcessor::calculate_position(const std::vector<std::pair<cv::Point2d, cv::Point2d>> &points, int centre_x, int centre_y) {
    static const std::vector<cv::Point3d> objPoints{{0, -4, 0}, {-4, 0, 0}, {0, 4, 0}, {4, 0, 0}};
    std::vector<cv::Point2d> corners = {points[1].first, points[0].first, points[1].second, points[0].second};
    cv::Mat rvec, tvec;
    cv::solvePnP(objPoints, corners, camera_matrix, dis_coeffs, rvec, tvec);
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cv::Mat R_inv = R.inv();
    cv::Mat cam_mat_inv = camera_matrix.inv();

    cv::Mat point_1(3, 1, CV_64F);
    point_1.at<double>(0, 0) = points[0].first.x;
    point_1.at<double>(1, 0) = points[0].first.y;
    point_1.at<double>(2, 0) = 1;

    cv::Mat coord_point_1_xyz_c = cam_mat_inv * point_1.mul(150);
    coord_point_1_xyz_c = coord_point_1_xyz_c - tvec;
    cv::Mat point_1_xyz = R_inv * coord_point_1_xyz_c;

    cv::Mat point_2(3, 1, CV_64F);
    point_2.at<double>(0, 0) = points[0].second.x;
    point_2.at<double>(1, 0) = points[0].second.y;
    point_2.at<double>(2, 0) = 1;

    cv::Mat coord_point_2_xyz_c = cam_mat_inv * point_2.mul(150);
    coord_point_2_xyz_c = coord_point_2_xyz_c - tvec;

    cv::Mat point_2_xyz = R_inv * coord_point_2_xyz_c;

    cv::Mat res(3, 1, CV_64F);
    res.at<double>(0, 0) = (point_1_xyz.at<double>(0, 0) + point_2_xyz.at<double>(0, 0)) / 2;
    res.at<double>(1, 0) = (point_1_xyz.at<double>(1, 0) + point_2_xyz.at<double>(1, 0)) / 2;
    res.at<double>(2, 0) = (point_1_xyz.at<double>(2, 0) + point_2_xyz.at<double>(2, 0)) / 2;

    return res;
}

}  // namespace video
}  // namespace vision
