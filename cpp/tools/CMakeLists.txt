if ((NOT WIN32) AND (NOT CMAKE_HOST_WIN32))
    add_executable(udp_server udp_server.cpp ${phil_common_src})
    target_include_directories(udp_server PRIVATE ${phil_include_dir} ${WPIUTIL_INCLUDE_DIR})

    add_executable(udp_client udp_client.cpp ${phil_common_src})
    target_include_directories(udp_client PRIVATE ${phil_include_dir} ${WPIUTIL_INCLUDE_DIR})

    add_executable(test_udp_trigger test_udp_trigger_tk1.cpp ${phil_common_src})
    target_include_directories(test_udp_trigger PRIVATE ${phil_include_dir} ${WPIUTIL_INCLUDE_DIR})
endif ()

if (NOT ${RIO})
    add_executable(ntcore_main ntcore_main.cpp)
    target_include_directories(ntcore_main PRIVATE ${WPIUTIL_INCLUDE_DIR} ${NTCORE_INCLUDE_DIR})
    target_link_libraries(ntcore_main ntcore)

    #    add_executable(imu_calibrate csvReader.cpp)
endif ()

# main or desktop programs
if (NOT ${RIO})
    add_executable(detect_markers detect_markers.cpp)
    target_link_libraries(detect_markers aruco ${phil_opencv_libs})

    add_executable(fisheye unwarp_fisheye.cpp)
    target_link_libraries(fisheye ${phil_opencv_libs})

    add_executable(mocap_record mocap_record.cpp ${phil_common_src})
    target_link_libraries(mocap_record aruco cscore ${phil_opencv_libs})
    target_include_directories(mocap_record PRIVATE ${phil_include_dir} ${CSCORE_INCLUDE_DIR} ${NTCORE_INCLUDE_DIR} ${WPIUTIL_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
endif ()

