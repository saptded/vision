include_directories(${PROJECT_SOURCE_DIR}/include)

add_subdirectory(exception)
add_subdirectory(image_recognition)

add_executable(main main.cpp)
target_link_libraries(main PUBLIC sender video exception)
