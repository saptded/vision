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
            camera(Camera("./../../images/pipelines/test.jpg")),
//            visibleArea(2 * tan(cameraAngle / 2.0 * M_PI / 180) * baseDepth),
            visibleArea(std::pair<int, int>{0, 0}),
            direction(0), x(0), y(0), xf(0), yf(0), directionLine(NONE), missingAutomaticSteps(0),
            recognizedMistakes(0), mode(MANUAL), automaticCoordinateRetries(0), automaticAngleRetries(0),
            automaticStepWithoutRetries(0) {
    }

    void VehicleController::Run() {
        x = 2665;
        xf = x;
        y = 2426;
        yf = y;
        direction = 221;
        cv::Mat frame;

        for (bool exit = false; !exit;) {
            clock_t start = clock();
            frame = camera.GetFrame(float(x), float(y), direction);
            _frame = frame;
            visibleArea.first = frame.cols;
            visibleArea.second = frame.rows;

            bool recognized = false;
            std::vector<std::pair<cv::Point, cv::Point>> lines = imageRecognizer.Recognize(frame);
            if (!lines.empty()) {
                recognized = true;
            }

            roiSize = camera.GetRoi().size;
            auto pipeline = getPipeline(lines);
            auto mistake = calculateMistake(pipeline);

            showArea(camera.GetArea().clone(), camera.GetRoi(), recognized, pipeline, mistake, mode == AUTOMATIC);
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
                clock_t end = clock();
                double seconds = (double)(end - start) / CLOCKS_PER_SEC;
                std::cout << seconds << std::endl;
            }

        }
    }

    void VehicleController::handleErrors() {
        bool toManual = false;
        if (automaticAngleRetries > 70) {
            automaticAngleRetries = 0;
            std::cout << "turn to manual by automaticAngleRetries" << std::endl;
            toManual = true;
        }
        if (automaticCoordinateRetries > 70) {
            automaticCoordinateRetries = 0;
            std::cout << "turn to manual by automaticCoordinateRetries" << std::endl;
            toManual = true;
        }
        if (missingAutomaticSteps > 10) {
            missingAutomaticSteps = 0;
            std::cout << "turn to manual by missingAutomaticSteps" << std::endl;
            toManual = true;
        }
        if (recognizedMistakes > 10) {
            recognizedMistakes = 0;
            std::cout << "turn to manual by recognizedMistakes" << std::endl;
            toManual = true;
        }

        if (automaticStepWithoutRetries > 10) {
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
        auto allowedMistakeAngle = 5;
        if (mistake.angle != 0) {
            direction -= 1 * int(copysign(1, mistake.angle));
            if (std::abs(mistake.angle) > allowedMistakeAngle) {
                automaticStepWithoutRetries = 0;
                automaticAngleRetries++;
            } else {
                automaticAngleRetries = 0;
                automaticStepWithoutRetries++;
            }
        }

        auto allowedMistakeCoordinate = roiSize.width / 100 * 15;
        if (std::abs(mistake.coordinate) > allowedMistakeCoordinate) {
            automaticStepWithoutRetries = 0;
            if (mistake.coordinate < 0) {
                directionLine = RIGHT;
            } else {
                directionLine = LEFT;
            }
            automaticCoordinateRetries++;
        } else {
            automaticCoordinateRetries = 0;
            automaticStepWithoutRetries++;
            baseVelocity = 10;
            directionLine = FORWARD;
        }
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
                                     Mistake mistake, bool isAutopilotOn) const {
        std::string info = std::string("x: " + std::to_string(cvRound(x)) +
                                       " y: " + std::to_string(cvRound(y)) +
                                       " direction: " + std::to_string(direction) +
                                       " linesAngle: " + std::to_string(pipeline.angle) +
                                       " mistakeAngle: " + std::to_string(mistake.angle) +
                                       " mistakeCoordinate: " + std::to_string(mistake.coordinate));
        int sizeOfOnePercentWidth = area.cols / 100;
        int sizeOfOnePercentHeight = area.rows / 100;

        int lineLength = int(roi.size.width);
        cv::Point pointCurs(
                cvRound(roi.center.x) + int(lineLength * cos(direction * M_PI / 180)),
                cvRound(roi.center.y) + int(lineLength * sin(direction * M_PI / 180))
        );

        arrowedLine(area, roi.center, pointCurs, cv::Scalar(255, 0, 0), 2, cv::LINE_AA, 0);

        float fontSize = sizeOfOnePercentHeight / 10 * 1.3;
        int thickness = sizeOfOnePercentHeight / 20 * 6;
        drawStatusCircleWithText(area,
                                 cv::Point(area.cols - sizeOfOnePercentWidth * 3, sizeOfOnePercentHeight * 3),
                                 sizeOfOnePercentHeight * 2,
                                 fontSize, thickness, isRecognized, "got object"
        );
        bool enableLookForwardIndicator = false;
        if (pipeline.angle <= 135 && pipeline.angle >= 45) {
            enableLookForwardIndicator = true;
        }
        drawStatusCircleWithText(area,
                                 cv::Point(area.cols - sizeOfOnePercentWidth * 3, sizeOfOnePercentHeight * 8),
                                 sizeOfOnePercentHeight * 2,
                                 fontSize,
                                 thickness,
                                 enableLookForwardIndicator, "got direction"
        );
        drawStatusCircleWithText(area,
                                 cv::Point(area.cols - sizeOfOnePercentWidth * 3, sizeOfOnePercentHeight * 13),
                                 sizeOfOnePercentHeight * 2,
                                 fontSize,
                                 thickness,
                                 isAutopilotOn, "automatic on"
        );

        cv::Point2f vertices[4];
        roi.points(vertices);
        for (int i = 0; i < 4; i++) {
            line(area, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 0, 255), 2);
        }

        line(_frame, pipeline.middleBorder.first, pipeline.middleBorder.second, cv::Scalar(0, 255, 0), 1,
             cv::LINE_AA);
        imshow("frame", _frame);

        cv::Mat pipelineFrame(roi.size.height, roi.size.width, CV_8UC3, cv::Scalar(0, 0, 0));
        line(pipelineFrame, pipeline.firstBorder.first, pipeline.firstBorder.second, cv::Scalar(255, 255, 255), 1,
             cv::LINE_AA);
        line(pipelineFrame, pipeline.secondBorder.first, pipeline.secondBorder.second, cv::Scalar(255, 255, 255), 1,
             cv::LINE_AA);
        line(pipelineFrame, pipeline.middleBorder.first, pipeline.middleBorder.second, cv::Scalar(255, 255, 255), 1,
             cv::LINE_AA);
        roi.points(vertices);
        arrowedLine(pipelineFrame, cv::Point(roi.size.width / 2, roi.size.height), cv::Point(roi.size.width / 2, 0), cv::Scalar(255, 255, 255), 2, cv::LINE_AA, 0);
        imshow("pipelineFrame", pipelineFrame);

        cv::putText(area, info,
                    cv::Point(sizeOfOnePercentWidth * 1, sizeOfOnePercentHeight * 4),
                    cv::FONT_HERSHEY_SIMPLEX, fontSize,
                    {0, 0, 0}, thickness);

        imshow("area", area);
    }

    void VehicleController::drawStatusCircleWithText(cv::Mat &area, cv::Point circleCenter, int radius, float fontScale,
                                                     float fontThickness, bool status, std::string text) const {
        cv::Scalar color;
        if (status) {
            color = cv::Scalar(0, 128, 0);
        } else {
            color = cv::Scalar(0, 0, 255);
        }
        cv::circle(
                area,
                circleCenter,
                radius, color,
                -1
        );
        cv::putText(area, text,
                    cv::Point(circleCenter.x - (text.length() / 10) * radius * 11, circleCenter.y + radius * 0.3),
                    cv::FONT_HERSHEY_SIMPLEX, fontScale,
                    {0, 0, 0}, fontThickness);
    }

    Pipeline VehicleController::getPipeline(std::vector<std::pair<cv::Point, cv::Point>> lines) {
        if (lines.size() <= 2) {
            recognizedMistakes++;
            return {};
        }
        int lineAngle = calculateLinesMedianAngle(lines);
        cv::Mat pipelineFrame(roiSize.height, roiSize.width, CV_8UC3, cv::Scalar(0, 0, 0));
        for (auto ln: lines) {
            line(pipelineFrame, ln.first, ln.second, cv::Scalar(0, 0, 255), 1,
                 cv::LINE_AA);
        }
        imshow("TESTMAIN1", pipelineFrame);

        filterLines(lines, lineAngle);
        extendLines(lines, lineAngle);

        cv::Mat pipelineFrame1(roiSize.height, roiSize.width, CV_8UC3, cv::Scalar(0, 0, 0));
        for (auto ln: lines) {
            line(pipelineFrame1, ln.first, ln.second, cv::Scalar(0, 0, 255), 1,
                 cv::LINE_AA);
        }
        imshow("extended", pipelineFrame1);

        auto pipeline = getBorderLines(lines, lineAngle);
        pipeline.angle = lineAngle;

        return pipeline;
    }

    void VehicleController::filterLines(std::vector<std::pair<cv::Point, cv::Point>> &lines, int lineAngle) {
        std::vector<float> lengthArr;
        float lenSum = 0;
        for (auto line: lines) {
            float len = sqrt(pow(line.first.x - line.second.x, 2) + pow(line.first.y - line.second.y, 2));
            lenSum += len;
            lengthArr.emplace_back(len);
        }
        sort(lengthArr.begin(), lengthArr.end());

        float mediumLen = lenSum / lines.size();
        float lineMedianLen;
        if (lengthArr.size() % 2 == 1) {
            lineMedianLen = lengthArr[lengthArr.size() / 2];
        } else {
            lineMedianLen = (lengthArr[lengthArr.size() / 2 - 1] + lengthArr[lengthArr.size() / 2]) / 2;
        }
        for (int i = 0; i < lines.size(); i++) {
            float len = sqrt(
                    pow(lines[i].first.x - lines[i].second.x, 2) + pow(lines[i].first.y - lines[i].second.y, 2));
            if (len < mediumLen) {
                lines.erase(lines.begin() + i);
                i--;
            }
        }

        for (int i = 0; i < lines.size(); i++) {
            double angle1 = 0;
            int subX;
            int subY;
            if (lines[i].first.y < lines[i].second.y) {
                subX = lines[i].first.x - lines[i].second.x;
                subY = lines[i].first.y - lines[i].second.y;
            } else {
                subX = lines[i].second.x - lines[i].first.x;
                subY = lines[i].second.y - lines[i].first.y;
            }
            if (lines[i].first.x == lines[i].second.x) {
                angle1 = 90;
            } else if (lines[i].first.y == lines[i].second.y) {
                angle1 = 180;
            } else {
                angle1 = -atan2(subY, subX) * 180 / M_PI;
            }
            int angleBorder = 5;
            if (angle1 < lineAngle - angleBorder || angle1 > lineAngle + angleBorder) {
                lines.erase(lines.begin() + i);
                i--;
            }
        }
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
        Mistake mistake{};
        if (mistakeAngle != 0) {
            mistake.angle = mistakeAngle;
        }

        int mistakeCoordinate = (visibleArea.first / 2) - pipeline.middleBorder.first.x;
        if (mistakeCoordinate != 0) {
            mistake.coordinate = mistakeCoordinate;
        }

        return mistake;
    }
}
