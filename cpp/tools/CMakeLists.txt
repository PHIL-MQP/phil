add_executable(udp_server udp_server.cpp ${common_src})
target_include_directories(udp_server PRIVATE ${phil_include_dir})

add_executable(udp_client udp_client.cpp ${common_src})
target_include_directories(udp_client PRIVATE ${phil_include_dir})

# anything using opencv won't compile on the RIO
if (NOT ${RIO})
    add_executable(fisheye unwarp_fisheye.cpp)
    target_link_libraries(fisheye opencv_core opencv_videoio opencv_imgproc opencv_highgui)

    add_executable(record_video record_video.cpp)
    target_link_libraries(record_video aruco opencv_core opencv_videoio opencv_imgproc opencv_highgui opencv_imgcodecs)
    target_include_directories(record_video PRIVATE ${OpenCV_INCLUDE_DIRS})
endif ()

