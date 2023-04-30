#include <iostream>

#include "image_recognition/ImageProcessor.h"
#include "exception/BaseException.h"

int main() {
    try {
        vision::video::VideoProcessor video;
        video.Run();
    } catch (BaseException &e) {
        std::cerr << e.what() << std::endl;
    }
    return 0;
}