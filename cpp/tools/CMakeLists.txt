add_executable(udp_server udp_server.cpp ${phil_common_src})
target_include_directories(udp_server PRIVATE ${phil_include_dir})

add_executable(udp_client udp_client.cpp ${phil_common_src})
target_include_directories(udp_client PRIVATE ${phil_include_dir})

add_executable(test_udp_trigger_tk1 test_udp_trigger_tk1.cpp ${phil_common_src})
target_include_directories(test_udp_trigger_tk1 PRIVATE ${phil_include_dir})

add_executable(ntcore_main ntcore_main.cpp)
target_include_directories(ntcore_main PRIVATE ${WPIUTIL_INCLUDE_DIR} ${NTCORE_INCLUDE_DIR})
target_link_libraries(ntcore_main ntcore)

if (NOT ${RIO})
    add_executable(imu_calibrate csvReader.cpp)
endif ()

# TK specific because opencv libraries are different there
if (${TK1})
    add_executable(fisheye unwarp_fisheye.cpp)
    target_link_libraries(fisheye
            opencv_core
            opencv_imgproc
            opencv_highgui)

    add_executable(mocap_record mocap_record.cpp ${phil_common_src})
    target_link_libraries(mocap_record
            aruco
            cscore
            opencv_core
            opencv_imgproc
            opencv_highgui)
    target_include_directories(mocap_record PRIVATE ${phil_include_dir} ${CSCORE_INCLUDE_DIR} ${NTCORE_INCLUDE_DIR} ${WPIUTIL_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
endif ()

# Ye old linux build
if (NOT ${RIO} AND NOT ${TK1})
    add_executable(fisheye unwarp_fisheye.cpp)
    target_link_libraries(fisheye
            opencv_core
            opencv_imgproc
            opencv_videoio
            opencv_highgui)

    add_executable(mocap_record mocap_record.cpp ${phil_common_src})
    target_link_libraries(mocap_record
            aruco
            cscore
            opencv_core
            opencv_videoio
            opencv_imgproc
            opencv_highgui)
    target_include_directories(mocap_record PRIVATE ${phil_include_dir} ${CSCORE_INCLUDE_DIR} ${NTCORE_INCLUDE_DIR} ${WPIUTIL_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
endif ()

