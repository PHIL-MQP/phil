project(phil)
cmake_minimum_required(VERSION 2.8.11)


find_package(OpenCV REQUIRED core videoio highgui)

add_executable(main main.cpp)
target_link_libraries(main ${OpenCV_LIBS})
