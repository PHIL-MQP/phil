project(cpp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -std=c++14")

find_package(OpenCV REQUIRED core videoio imgproc highgui)

add_executable(unwarp_fisheye unwarp_fisheye.cpp)
target_link_libraries(unwarp_fisheye ${OpenCV_LIBS})

add_executable(log_polar log_polar.cpp)
target_link_libraries(log_polar ${OpenCV_LIBS})
