#pragma once

#include <opencv2/videoio.hpp>
#include "image_recognition/ImageProcessor.h"

namespace vision::controller {
    class VehicleController {
    public:
        VehicleController(
                int baseSpeed,
                int baseDepth,
                int cameraAngle
        );

        void Run();

    private:
        std::pair<cv::Mat, cv::Rect> vehicleWalk();

        int direction;
        int baseSpeed;
        int baseDepth;
        int cameraAngle;
        double visibilityArea;

        int x;
        int y;
        cv::Mat ship;

        image::ImageRecognizer imageRecognizer;
    };
}  // namespace vision::controller
