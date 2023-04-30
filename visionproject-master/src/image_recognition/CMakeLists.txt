add_library(video STATIC VideoProcessor.cpp)
target_link_libraries(video PUBLIC ${OpenCV_LIBS} exception)