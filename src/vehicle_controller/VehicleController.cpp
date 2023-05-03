#include <cmath>
#include <unistd.h>
#include <opencv2/opencv.hpp>

#include "vehicle_controller/VehicleController.h"

namespace vision::controller {
    VehicleController::VehicleController(
            int baseVelocity,
            int baseAngularVelocity,
            int baseDepth,
            int cameraAngle
    ) :
            baseVelocity(baseVelocity), baseAngularVelocity(baseAngularVelocity), baseDepth(baseDepth),
            cameraAngle(cameraAngle),
            camera(Camera("./../../images/pipelines/pipeline_middlequality.jpeg")),
//            visibleArea(2 * tan(cameraAngle / 2.0 * M_PI / 180) * baseDepth),
            visibleArea(std::pair<int, int>{0, 0}),
            direction(-70), x(0), y(0), xf(0), yf(0), directionLine(NONE), missingAutomaticSteps(0),
            recognizedMistakes(0), mode(MANUAL), automaticCoordinateRetries(0), automaticAngleRetries(0),
            automaticStepWithoutRetries(0) {
    }

    void VehicleController::Run() {
        x = 1290;
        xf = 1290;
        y = 1650;
        yf = 1650;
        cv::Mat frame;

        for (bool exit = false; !exit;) {
            frame = camera.GetFrame(float(x), float(y), direction);
            visibleArea.first = frame.cols;
            visibleArea.second = frame.rows;

            bool recognized = false;
            std::vector<std::pair<cv::Point, cv::Point>> lines = imageRecognizer.Recognize(frame);
            if (!lines.empty()) {
                recognized = true;
            }

            auto pipeline = getPipeline(lines);
            auto mistake = calculateMistake(pipeline);

            showArea(camera.GetArea().clone(), camera.GetRoi(), recognized, pipeline, mistake);
            switch (mode) {
                case EXIT:
                    cv::destroyAllWindows();
                    exit = true;
                    break;
                case AUTOMATIC:
                    handleAutomaticStep(mistake);
                    break;
                case MANUAL:
                    mode = handleKey();
                    break;
            }

            updateCoordinates();
            handleErrors();
            if (mode == AUTOMATIC) {
                cv::waitKey(10) & 0XFF;
                auto k = cv::pollKey();
                if (k == 27) {
                    mode = MANUAL;
                    baseVelocity = 20;
                    baseAngularVelocity = 5;
                }
            }
        }
    }

    void VehicleController::handleErrors() {
        bool toManual = false;
        if (automaticAngleRetries > 70) {
            automaticAngleRetries = 0;
            toManual = true;
        }
        if (automaticCoordinateRetries > 70) {
            automaticCoordinateRetries = 0;
            toManual = true;
        }
        if (missingAutomaticSteps > 10) {
            missingAutomaticSteps = 0;
            toManual = true;
        }
        if (recognizedMistakes > 10) {
            recognizedMistakes = 0;
            toManual = true;
        }

        if (automaticStepWithoutRetries > 20) {
            automaticCoordinateRetries = 0;
            automaticAngleRetries = 0;
        }

        if (toManual) {
            directionLine = NONE;
            mode = MANUAL;
            baseVelocity = 20;
            baseAngularVelocity = 5;
        }
    }

    void VehicleController::handleAutomaticStep(Mistake mistake) {
        if (std::abs(mistake.angle) >= 45) {
            missingAutomaticSteps++;
            return;
        }
        baseVelocity = 1;
        baseAngularVelocity = 1;
        if (mistake.angle != 0) {
            automaticStepWithoutRetries = 0;
            direction -= 1 * int(copysign(1, mistake.angle));
            automaticAngleRetries++;
            return;
        }
        if (std::abs(mistake.coordinate) > 4) {
            automaticStepWithoutRetries = 0;
            if (mistake.coordinate < 0) {
                directionLine = RIGHT;
            } else {
                directionLine = LEFT;
            }
            automaticCoordinateRetries++;
            return;
        }
        missingAutomaticSteps = 0;
        automaticStepWithoutRetries++;
        baseVelocity = 10;
        directionLine = FORWARD;
    }

    void VehicleController::updateCoordinates() {
        switch (directionLine) {
            case FORWARD:
                xf = xf + baseVelocity * cos(direction * M_PI / 180);
                yf = yf + baseVelocity * sin(direction * M_PI / 180);
                break;
            case BACKWARD:
                xf = xf - baseVelocity * cos(direction * M_PI / 180);
                yf = yf - baseVelocity * sin(direction * M_PI / 180);
                break;
            case RIGHT:
                xf = xf + baseVelocity * cos((direction + 90) * M_PI / 180);
                yf = yf + baseVelocity * sin((direction + 90) * M_PI / 180);
                break;
            case LEFT:
                xf = xf + baseVelocity * cos((direction - 90) * M_PI / 180);
                yf = yf + baseVelocity * sin((direction - 90) * M_PI / 180);
                break;
            case NONE:
                break;
        }
        x = int(xf);
        y = int(yf);
    }

    Mode VehicleController::handleKey() {
        int k = cv::waitKey();
        switch (k) {
            case 113:
            case 81: // Q/q
                direction -= baseAngularVelocity;
                directionLine = NONE;
                return MANUAL;
            case 101:
            case 69: // E/e
                direction += baseAngularVelocity;
                directionLine = NONE;
                return MANUAL;
            case 119:
            case 87: // W/w
                directionLine = FORWARD;
                return MANUAL;
            case 97:
            case 65: // A/a
                directionLine = LEFT;
                return MANUAL;
            case 115:
            case 83: // S/s
                directionLine = BACKWARD;
                return MANUAL;
            case 100:
            case 68: // D/d
                directionLine = RIGHT;
                return MANUAL;
            case 32: // space
                return AUTOMATIC;
            case 27: // esc
                return EXIT;
            default:
                directionLine = NONE;
                std::cout << "wrong key: " << k << std::endl;
        }
        return MANUAL;
    }

    void VehicleController::showArea(cv::Mat area, cv::RotatedRect roi, bool isRecognized, Pipeline pipeline,
                                     Mistake mistake) const {
        std::string info = std::string("x: " + std::to_string(cvRound(x)) +
                                       " y: " + std::to_string(cvRound(y)) +
                                       " direction: " + std::to_string(direction) +
                                       " lineAngle: " + std::to_string(pipeline.angle) +
                                       " mistakeAngle: " + std::to_string(mistake.angle) +
                                       " mistakeCoordinate: " + std::to_string(mistake.coordinate));


        int lineLength = 150;
        cv::Point pointCurs(
                cvRound(roi.center.x) + int(lineLength * cos(direction * M_PI / 180)),
                cvRound(roi.center.y) + int(lineLength * sin(direction * M_PI / 180))
        );

        arrowedLine(area, roi.center, pointCurs, cv::Scalar(255, 0, 0), 2, cv::LINE_AA, 0);

        cv::Scalar color;
        if (isRecognized) {
            color = cv::Scalar(0, 128, 0);
        } else {
            color = cv::Scalar(0, 0, 255);
        }
        cv::circle(area, cv::Point(area.cols - 50, 50), 20, color, -1);
        bool enableLookForwardIndicator = false;
        if (pipeline.angle <= 135 && pipeline.angle >= 45) {
            enableLookForwardIndicator = true;
        }
        if (enableLookForwardIndicator) {
            color = cv::Scalar(0, 128, 0);
        } else {
            color = cv::Scalar(0, 0, 255);
        }
        cv::circle(area, cv::Point(area.cols - 50, 100), 20, color, -1);


        cv::Point2f vertices[4];
        roi.points(vertices);
        for (int i = 0; i < 4; i++) {
            line(area, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 0, 255), 2);
        }
        cv::Mat pipelineFrame(127, 127, CV_8UC3, cv::Scalar(0, 0, 0));
        line(pipelineFrame, pipeline.firstBorder.first, pipeline.firstBorder.second, cv::Scalar(0, 0, 255), 1,
             cv::LINE_AA);
        line(pipelineFrame, pipeline.secondBorder.first, pipeline.secondBorder.second, cv::Scalar(255, 0, 0), 1,
             cv::LINE_AA);
        line(pipelineFrame, pipeline.middleBorder.first, pipeline.middleBorder.second, cv::Scalar(0, 255, 0), 1,
             cv::LINE_AA);
        imshow("pipelineFrame", pipelineFrame);

        cv::putText(area, info, cv::Point(50, 60), cv::FONT_HERSHEY_SIMPLEX, 1.1,
                    {0, 0, 0});

        imshow("area", area);
    }

    Pipeline VehicleController::getPipeline(std::vector<std::pair<cv::Point, cv::Point>> lines) {
        if (lines.size() <= 2) {
            recognizedMistakes++;
            return {};
        }
        int lineAngle = calculateLinesMedianAngle(lines);
        if (lineAngle > 135 || lineAngle < 45) {
            calculateLinesMedianAngle(lines);
        }

        extendLines(lines, lineAngle);

        auto pipeline = getBorderLines(lines, lineAngle);
        pipeline.angle = lineAngle;

        return pipeline;
    }

    int VehicleController::calculateLinesMedianAngle(const std::vector<std::pair<cv::Point, cv::Point>> &lines) {
        std::vector<double> angleArr;
        for (auto line: lines) {
            double angle1 = 0;
            int subX;
            int subY;
            if (line.first.y < line.second.y) {
                subX = line.first.x - line.second.x;
                subY = line.first.y - line.second.y;
            } else {
                subX = line.second.x - line.first.x;
                subY = line.second.y - line.first.y;
            }
            if (line.first.x == line.second.x) {
                angle1 = 90;
            } else if (line.first.y == line.second.y) {
                angle1 = 180;
            } else {
                angle1 = -atan2(subY, subX) * 180 / M_PI;
            }

            angleArr.emplace_back(angle1);
        }
        sort(angleArr.begin(), angleArr.end());

        int lineAngle;
        if (angleArr.size() % 2 == 1) {
            lineAngle = int(angleArr[angleArr.size() / 2]);
        } else {
            lineAngle = int((angleArr[angleArr.size() / 2 - 1] + angleArr[angleArr.size() / 2]) / 2);
        }

        return lineAngle;
    }

    void VehicleController::extendLines(std::vector<std::pair<cv::Point, cv::Point>> &lines, int medianLineAngle) {
        if (medianLineAngle <= 135 && medianLineAngle >= 45) {
            int highestY = 100000;
            int lowestY = 0;
            for (auto line: lines) {
                highestY = std::min(highestY, line.first.y);
                highestY = std::min(highestY, line.second.y);
                lowestY = std::max(lowestY, line.first.y);
                lowestY = std::max(lowestY, line.second.y);
            }
            for (auto &line: lines) {
                auto highY = std::min(line.first.y, line.second.y);
                auto lowY = std::max(line.first.y, line.second.y);
                double tang = tan((medianLineAngle - 90) * M_PI / 180);
                if (highY != highestY) {
                    int differenceHigh = std::abs(highestY - highY);
                    if (highY == line.first.y) {
                        line.first.x = line.first.x - int(differenceHigh * tang);
                        line.first.y = highestY;
                    } else {
                        line.second.x = line.second.x - int(differenceHigh * tang);
                        line.second.y = highestY;
                    }
                }
                if (lowY != lowestY) {
                    int differenceLow = std::abs(lowestY - lowY);
                    if (lowY == line.first.y) {
                        line.first.x = line.first.x + int(differenceLow * tang);
                        line.first.y = lowestY;
                    } else {
                        line.second.x = line.second.x + int(differenceLow * tang);
                        line.second.y = lowestY;
                    }
                }
            }
        } else {
            int highestX = 0;
            int lowestX = 100000;
            for (auto line: lines) {
                highestX = std::max(highestX, line.first.x);
                highestX = std::max(highestX, line.second.x);
                lowestX = std::min(lowestX, line.first.x);
                lowestX = std::min(lowestX, line.second.x);
            }
            for (auto &line: lines) {
                auto highX = std::max(line.first.x, line.second.x);
                auto lowX = std::min(line.first.x, line.second.x);
                double tang = tan((medianLineAngle) * M_PI / 180);
                if (highX != highestX) {
                    int differenceHigh = std::abs(highestX - highX);
                    if (highX == line.first.x) {
                        line.first.y = line.first.y - int(differenceHigh * tang);
                        line.first.x = highestX;
                    } else {
                        line.second.y = line.second.y - int(differenceHigh * tang);
                        line.second.x = highestX;
                    }
                }
                if (lowX != lowestX) {
                    int differenceLow = std::abs(lowestX - lowX);
                    if (lowX == line.first.x) {
                        line.first.y = line.first.y + int(differenceLow * tang);
                        line.first.x = lowestX;
                    } else {
                        line.second.y = line.second.y + int(differenceLow * tang);
                        line.second.x = lowestX;
                    }
                }
            }
        }

    }

    Pipeline
    VehicleController::getBorderLines(const std::vector<std::pair<cv::Point, cv::Point>> &lines, int medianLineAngle) {
        Pipeline p;
        std::pair<cv::Point, cv::Point> middleLine;
        if (medianLineAngle <= 135 && medianLineAngle >= 45) {
            std::pair<std::pair<cv::Point, cv::Point>, std::pair<cv::Point, cv::Point>> borders{
                    std::pair<cv::Point, cv::Point>{cv::Point{10000, 0}, cv::Point{10000, 0}},
                    std::pair<cv::Point, cv::Point>{cv::Point{-10000, 0}, cv::Point{-10000, 0}}
            };
            for (auto line: lines) {
                int maxY = std::min(line.first.y, line.second.y);
                int minY = std::max(line.first.y, line.second.y);
                borders.first.first.y = std::min(maxY, borders.first.first.y);
                borders.second.first.y = borders.first.first.y;
                borders.first.second.y = std::max(minY, borders.first.second.y);
                borders.second.second.y = borders.first.second.y;
                if (line.first.y == maxY) {
                    if (borders.first.first.x > line.first.x) {
                        borders.first.first.x = line.first.x;
                    }
                    if (borders.second.first.x < line.first.x) {
                        borders.second.first.x = line.first.x;
                    }
                } else if (line.first.y == minY) {
                    if (borders.first.second.x > line.first.x) {
                        borders.first.second.x = line.first.x;
                    }
                    if (borders.second.second.x < line.first.x) {
                        borders.second.second.x = line.first.x;
                    }
                }
                if (line.second.y == maxY) {
                    if (borders.first.first.x > line.second.x) {
                        borders.first.first.x = line.second.x;
                    }
                    if (borders.second.first.x < line.second.x) {
                        borders.second.first.x = line.second.x;
                    }
                } else if (line.second.y == minY) {
                    if (borders.first.second.x > line.second.x) {
                        borders.first.second.x = line.second.x;
                    }
                    if (borders.second.second.x < line.second.x) {
                        borders.second.second.x = line.second.x;
                    }
                }
            }
            int highY = borders.first.first.y;
            int lowX = borders.first.second.y;
            int middleHighX = borders.first.first.x + std::abs((borders.second.first.x - borders.first.first.x) / 2);
            int middleLowX = borders.first.second.x + std::abs((borders.second.second.x - borders.first.second.x) / 2);
            middleLine = std::pair<cv::Point, cv::Point>{
                    cv::Point{middleHighX, highY},
                    cv::Point{middleLowX, lowX}
            };
            p = Pipeline{borders.first, borders.second, middleLine};
        } else {
            std::pair<std::pair<cv::Point, cv::Point>, std::pair<cv::Point, cv::Point>> borders{
                    std::pair<cv::Point, cv::Point>{cv::Point{0, 10000}, cv::Point{0, 10000}},
                    std::pair<cv::Point, cv::Point>{cv::Point{0, -10000}, cv::Point{0, -10000}}
            };
            for (auto line: lines) {
                int maxX = std::max(line.first.x, line.second.x);
                int minX = std::min(line.first.x, line.second.x);
                borders.first.first.x = std::min(minX, borders.first.first.x);
                borders.second.first.x = borders.first.first.x;
                borders.first.second.x = std::max(maxX, borders.first.second.x);
                borders.second.second.x = borders.first.second.x;
                if (line.first.x == minX) {
                    if (borders.first.first.y > line.first.y) {
                        borders.first.first.y = line.first.y;
                    }
                    if (borders.second.first.y < line.first.y) {
                        borders.second.first.y = line.first.y;
                    }
                } else if (line.first.x == maxX) {
                    if (borders.first.second.y > line.first.y) {
                        borders.first.second.y = line.first.y;
                    }
                    if (borders.second.second.y < line.first.y) {
                        borders.second.second.y = line.first.y;
                    }
                }
                if (line.second.x == minX) {
                    if (borders.first.first.y > line.second.y) {
                        borders.first.first.y = line.second.y;
                    }
                    if (borders.second.first.y < line.second.y) {
                        borders.second.first.y = line.second.y;
                    }
                } else if (line.second.x == maxX) {
                    if (borders.first.second.y > line.second.y) {
                        borders.first.second.y = line.second.y;
                    }
                    if (borders.second.second.y < line.second.y) {
                        borders.second.second.y = line.second.y;
                    }
                }
            }
            int leftX = borders.first.first.x;
            int rightX = borders.first.second.x;
            int middleLeftY = borders.first.first.y + std::abs((borders.second.first.y - borders.first.first.y) / 2);
            int middleRightY =
                    borders.first.second.y + std::abs((borders.second.second.y - borders.first.second.y) / 2);
            middleLine = std::pair<cv::Point, cv::Point>{
                    cv::Point{leftX, middleLeftY},
                    cv::Point{rightX, middleRightY}
            };
            p = Pipeline{borders.first, borders.second, middleLine};
        }

        return p;
    }

    Mistake VehicleController::calculateMistake(Pipeline pipeline) const {
        int mistakeAngle = pipeline.angle - 90;
        if (mistakeAngle != 0) {
            return Mistake{0, mistakeAngle};
        }

        int mistakeCoordinate = (visibleArea.first / 2) - pipeline.middleBorder.first.x;
        if (mistakeCoordinate != 0) {
            return Mistake{mistakeCoordinate, 0};
        }

        return {};
    }
}
