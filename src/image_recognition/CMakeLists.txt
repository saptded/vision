add_library(recognizer STATIC ImageRecognizer.cpp)
target_link_libraries(recognizer PUBLIC ${OpenCV_LIBS} exception)