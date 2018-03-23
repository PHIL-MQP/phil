if (NOT WIN32)
    add_executable(test_udp_trigger test_udp_trigger.cpp ../src/common/udp.cpp)
    target_include_directories(test_udp_trigger PRIVATE ${phil_include_dir} ${WPIUTIL_INCLUDE_DIR})
endif ()

if (NOT ${RIO})
    add_executable(ntcore_main ntcore_main.cpp)
    target_include_directories(ntcore_main PRIVATE ${WPIUTIL_INCLUDE_DIR} ${NTCORE_INCLUDE_DIR})
    target_link_libraries(ntcore_main ntcore phil_common)

    add_executable(phil_kalman_filter phil_kalman_filter.cpp)
    target_include_directories(phil_kalman_filter PRIVATE ${orocos-bfl_INCLUDE_DIRS})
    target_link_libraries(phil_kalman_filter phil_common phil_localization orocos-bfl)

    target_link_libraries(phil_kalman_filter phil_common phil_localization ${orocos-bfl_LIBRARIES})
    target_compile_options(phil_kalman_filter PRIVATE -Wall -Wextra)
    target_compile_definitions(phil_kalman_filter PRIVATE CSV_IO_NO_THREAD)

    add_executable(annotate_video annotate_video.cpp)
    target_link_libraries(annotate_video aruco ${phil_opencv_libs})
    target_include_directories(annotate_video PRIVATE ${phil_include_dir} ${OpenCV_INCLUDE_DIRS})

    add_executable(publish_rio_data publish_rio_data.cpp)
    target_link_libraries(publish_rio_data phil_common)

    add_executable(detect_markers_in_video detect_markers_in_video.cpp)
    target_link_libraries(detect_markers_in_video aruco phil_common)

    add_executable(record_cameras_on_udp_trigger record_cameras_on_udp_trigger.cpp)
    target_link_libraries(record_cameras_on_udp_trigger aruco cscore ${phil_opencv_libs} yaml-cpp phil_common)
    target_include_directories(record_cameras_on_udp_trigger PRIVATE ${CSCORE_INCLUDE_DIR} ${YAML_CPP_INCLUDE_DIRS})

    add_executable(aruco_localize_from_mm aruco_localize_from_mm.cpp)
    target_link_libraries(aruco_localize_from_mm aruco ${phil_opencv_libs} phil_common)

    add_executable(play_video_with_cscore play_video_with_cscore.cpp)
    target_include_directories(play_video_with_cscore PRIVATE ${CSCORE_INCLUDE_DIR})
    target_link_libraries(play_video_with_cscore phil_common cscore)

    add_executable(cscore_webcam cscore_webcam.cpp)
    target_include_directories(cscore_webcam PRIVATE ${CSCORE_INCLUDE_DIR})
    target_link_libraries(cscore_webcam phil_common cscore)

    add_executable(localize localize.cpp)
    target_link_libraries(localize ${phil_opencv_libs} aruco cscore phil_common)
    target_include_directories(localize PRIVATE ${phil_include_dir} ${CSCORE_INCLUDE_DIR} ${NTCORE_INCLUDE_DIR} ${WPIUTIL_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})

    add_executable(localize_from_mm localize_from_mm.cpp)
    target_link_libraries(localize_from_mm ${phil_opencv_libs} aruco cscore phil_common)
    target_include_directories(localize_from_mm PRIVATE ${phil_include_dir} ${CSCORE_INCLUDE_DIR} ${NTCORE_INCLUDE_DIR} ${WPIUTIL_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})

    add_executable(lkdemo lkdemo.cpp)
    target_link_libraries(lkdemo ${phil_opencv_libs} opencv_video)
    target_include_directories(lkdemo PRIVATE ${OpenCV_INCLUDE_DIRS})

    add_executable(fisheye unwarp_fisheye.cpp)
    target_link_libraries(fisheye ${phil_opencv_libs})
    target_include_directories(fisheye PRIVATE ${OpenCV_INCLUDE_DIRS})


    if (NOT CMAKE_HOST_WIN32)
        add_executable(record_imu_calibration_data record_imu_calibration_data.cpp)
        target_include_directories(record_imu_calibration_data PRIVATE ${LINUX_NAVX_DRIVER_INCLUDE_DIR})
        target_link_libraries(record_imu_calibration_data linux_navx_driver)
    endif ()
endif ()

