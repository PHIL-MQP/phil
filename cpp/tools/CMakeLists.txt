if (NOT WIN32)
    add_executable(udp_server udp_server.cpp ${common_src})
    target_include_directories(udp_server PRIVATE ${phil_include_dir} ${WPIUTIL_INCLUDE_DIR})

    add_executable(udp_client udp_client.cpp ${common_src})
    target_include_directories(udp_client PRIVATE ${phil_include_dir} ${WPIUTIL_INCLUDE_DIR})

    add_executable(test_udp_trigger test_udp_trigger.cpp ${common_src})
    target_include_directories(test_udp_trigger PRIVATE ${phil_include_dir} ${WPIUTIL_INCLUDE_DIR})
endif ()

if (NOT ${RIO})
    add_executable(ntcore_main ntcore_main.cpp)
    target_include_directories(ntcore_main PRIVATE ${WPIUTIL_INCLUDE_DIR} ${NTCORE_INCLUDE_DIR})
    target_link_libraries(ntcore_main ntcore)

    add_executable(detect_markers detect_markers.cpp)
    target_link_libraries(detect_markers aruco ${phil_opencv_libs})
    target_include_directories(detect_markers PRIVATE ${OpenCV_INCLUDE_DIRS})

    add_executable(marker_detection_analysis marker_detection_analysis.cpp)
    target_link_libraries(marker_detection_analysis aruco ${phil_opencv_libs})
    target_include_directories(marker_detection_analysis PRIVATE ${phil_include_dir} ${OpenCV_INCLUDE_DIRS})

    add_executable(fisheye unwarp_fisheye.cpp)
    target_link_libraries(fisheye ${phil_opencv_libs})
    target_include_directories(fisheye PRIVATE ${OpenCV_INCLUDE_DIRS})

    add_executable(record_cameras_on_udp_trigger record_cameras_on_udp_trigger.cpp ${common_src})
    target_link_libraries(record_cameras_on_udp_trigger aruco cscore ${phil_opencv_libs} yaml-cpp)
    target_include_directories(record_cameras_on_udp_trigger PRIVATE ${phil_include_dir} ${CSCORE_INCLUDE_DIR} ${NTCORE_INCLUDE_DIR} ${WPIUTIL_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS})

    if (NOT CMAKE_HOST_WIN32)
        add_executable(record_imu_calibration_data record_imu_calibration_data.cpp)
        target_include_directories(record_imu_calibration_data PRIVATE ${LINUX_NAVX_DRIVER_INCLUDE_DIR})
        target_link_libraries(record_imu_calibration_data linux_navx_driver)
    endif ()

    add_executable(localize localize.cpp ${common_src})
    target_link_libraries(localize ${phil_opencv_libs} aruco cscore)
    target_include_directories(localize PRIVATE ${phil_include_dir} ${CSCORE_INCLUDE_DIR} ${NTCORE_INCLUDE_DIR} ${WPIUTIL_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})

    add_executable(localize_from_mm localize_from_mm.cpp ${common_src})
    target_link_libraries(localize_from_mm ${phil_opencv_libs} aruco cscore)
    target_include_directories(localize_from_mm PRIVATE ${phil_include_dir} ${CSCORE_INCLUDE_DIR} ${NTCORE_INCLUDE_DIR} ${WPIUTIL_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
endif ()

