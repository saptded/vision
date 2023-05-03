add_library(camera STATIC Camera.cpp)
target_link_libraries(camera PUBLIC ${OpenCV_LIBS})