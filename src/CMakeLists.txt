include_directories(${PROJECT_SOURCE_DIR}/include)

add_subdirectory(exception)
add_subdirectory(image_recognition)
add_subdirectory(vehicle_controller)
add_subdirectory(camera)

add_executable(main main.cpp)
target_link_libraries(main PUBLIC recognizer exception controller camera)
