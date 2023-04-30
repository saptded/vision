add_library(controller STATIC VehicleController.cpp)
target_link_libraries(controller PUBLIC ${OpenCV_LIBS} exception recognizer)