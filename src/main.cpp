#include <iostream>

#include "image_recognition/ImageProcessor.h"
#include "exception/BaseException.h"

int main() {
    try {
        vision::image::ImageProcessor image;
        image.ProcessFromPattern("../../images/*.png");
    } catch (BaseException &e) {
        std::cerr << e.what() << std::endl;
    }
    return 0;
}