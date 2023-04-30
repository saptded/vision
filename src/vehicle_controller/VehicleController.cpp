#include <cmath>
#include "vehicle_controller/VehicleController.h"

#include <opencv2/opencv.hpp>

namespace vision::controller {

    VehicleController::VehicleController(
            int baseSpeed,
            int baseDepth,
            int cameraAngle
    ) : baseSpeed(baseSpeed), baseDepth(baseDepth), cameraAngle(cameraAngle) {
        visibilityArea = 2 * tan(cameraAngle / 2.0 * M_PI / 180) * baseDepth;
        direction = 0;
        x = 0;
        y = 0;
        ship = cv::imread("./../../images/pipelines/pipeline_lowquality.jpeg", cv::IMREAD_COLOR);
        imageRecognizer = image::ImageRecognizer();
    }

    void VehicleController::Run() {
        bool run = true;
        x = 420;
        y = 10;
        for (; run;) {


            std::pair<cv::Mat, cv::Rect> walked = vehicleWalk();

            cv::Mat frame = ship.clone();
            cv::rectangle(frame, walked.second, cv::Scalar(0, 0, 255), 2);
            bool recognized = imageRecognizer.Recognize(walked.first);
            imshow("ship", frame);
            int k = cv::waitKey(0);
            if (k == 27) {
                run = false;
                cv::destroyAllWindows();
            }
            if (k == 32) {

                direction += 90;
            }

            std::cout << x << " " << y << std::endl;
        }

    }

    std::pair<cv::Mat, cv::Rect> VehicleController::vehicleWalk() {
        int divisionX = 10;
        int divisionY = 10;
        int width = ship.cols / divisionX;
        int height = ship.rows / divisionY;

        int newX = x + baseSpeed * cos(direction * M_PI / 180);
        int newY = y + baseSpeed * sin(direction * M_PI / 180);


        cv::Rect roi;
        bool collision = false;
        if ((newX + width) > ship.cols) {
            roi = cv::Rect(ship.cols - width, newY, width, height);
            x = ship.cols - width;
            y = newY;
            collision = true;
        } else if (newX < 0) {
            roi = cv::Rect(0, newY, width, height);
            x = 0;
            y = newY;
            collision = true;
        }
        if ((newY + height) > ship.rows) {
            roi = cv::Rect(newX, ship.rows - height, width, height);
            x = newX;
            y = ship.rows - height;
            collision = true;
        } else if (newY < 0) {
            roi = cv::Rect(newX, 0, width, height);
            x = newX;
            y = 0;
            collision = true;
        }

        if (!collision) {
            roi = cv::Rect(newX, newY, width, height);
            x = newX;
            y = newY;
        }

        cv::Mat frame = ship(roi);

        return std::pair<cv::Mat, cv::Rect>{frame, roi};
    }

}
