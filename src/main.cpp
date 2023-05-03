#include <iostream>

#include "image_recognition/ImageProcessor.h"
#include "exception/BaseException.h"
#include "vehicle_controller/VehicleController.h"

int main() {
    try {
        vision::controller::VehicleController controller(20, 5, 50, 5);
        vision::image::ImageRecognizer imageRecognizer;
//        imageRecognizer.RecognizeFromPattern("../../images/ships/*");
        controller.Run();
    } catch (BaseException &e) {
        std::cerr << e.what() << std::endl;
    }
    return 0;
}