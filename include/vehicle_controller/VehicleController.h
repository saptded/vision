#pragma once

#include <opencv2/videoio.hpp>
#include "image_recognition/ImageProcessor.h"
#include "camera/Camera.h"

enum Direction {
    FORWARD,
    BACKWARD,
    RIGHT,
    LEFT,
    NONE
};

enum Mode {
    MANUAL,
    AUTOMATIC,
    EXIT
};

struct Pipeline {
    std::pair<cv::Point, cv::Point> firstBorder;
    std::pair<cv::Point, cv::Point> secondBorder;
    std::pair<cv::Point, cv::Point> middleBorder;
    int angle;
};

struct Mistake {
    int coordinate;
    int angle;
};

namespace vision::controller {
    class VehicleController {
    public:
        VehicleController(
                int baseVelocity,
                int baseAngularVelocity,
                int baseDepth,
                int cameraAngle
        );

        void Run();

    private:
        void updateCoordinates();

        Mode handleKey();

        void showArea(cv::Mat area, cv::RotatedRect roi, bool isRecognized, Pipeline pipeline, Mistake mistake) const;

        Pipeline getPipeline(std::vector<std::pair<cv::Point, cv::Point>> lines);

        static int calculateLinesMedianAngle(const std::vector<std::pair<cv::Point, cv::Point>> &lines);

        static void extendLines(std::vector<std::pair<cv::Point, cv::Point>> &lines, int medianLineAngle);

        static Pipeline getBorderLines(const std::vector<std::pair<cv::Point, cv::Point>> &lines, int medianLineAngle);

        [[nodiscard]] Mistake calculateMistake(Pipeline pipeline) const;
        void handleAutomaticStep(Mistake mistake);
        void handleErrors();

        int direction;
        Direction directionLine;
        int baseVelocity;
        int baseAngularVelocity;
        int baseDepth;
        int cameraAngle;
        std::pair<int, int> visibleArea;

        float xf;
        float yf;
        int x;
        int y;

        Camera camera;
        image::ImageRecognizer imageRecognizer;

        Mode mode;
        int missingAutomaticSteps;
        int recognizedMistakes;
        int automaticAngleRetries;
        int automaticCoordinateRetries;
        int automaticStepWithoutRetries;
    };
}  // namespace vision::controller
