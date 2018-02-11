project(phil)
cmake_minimum_required(VERSION 2.8.0)

option(RIO "cross-compile for RoboRIO" OFF)
option(TK1 "cross-compile for TK1" OFF)

#############################################################################
## Dependencies
#############################################################################
# flags and config
if (NOT ${RIO})
    find_package(OpenCV REQUIRED)
    find_package(aruco REQUIRED)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(orocos-bfl REQUIRED orocos-bfl)
endif ()

if (${TK1})
    set(phil_opencv_libs
            opencv_core
            opencv_imgproc
            opencv_highgui)
elseif (NOT ${RIO})
    set(phil_opencv_libs
            opencv_core
            opencv_imgproc
            opencv_videoio
            opencv_imgcodecs
            opencv_highgui)
endif ()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++11")

if (CMAKE_HOST_WIN32)
    set(HOME $ENV{HOMEPATH})
    STRING(REGEX REPLACE "\\\\" "/" HOME ${HOME})
else ()
    set(HOME $ENV{HOME})
endif ()

set(WPILIB_DIR ${HOME}/wpilib)
set(WPILIB_CPP_DIR ${WPILIB_DIR}/cpp/current)
set(WPILIB_COMMON_DIR ${WPILIB_DIR}/common/current)
set(WPIUTIL_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/libraries/wpiutil/src/main/native/include)
set(NTCORE_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/libraries/ntcore/src/main/native/include)
set(CSCORE_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/libraries/cscore/src/main/native/include)
set(NAVX_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/libraries/navxmxp/roborio/c++/navx_frc_cpp/include
        ${CMAKE_SOURCE_DIR}/libraries/navxmxp/roborio/c++/navx_frc_cpp/src)
set(LINUX_NAVX_DRIVER_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/libraries/linux_navx_driver)
if (CMAKE_HOST_WIN32)
    set(WPILIB_INCLUDE_DIR ${WPILIB_CPP_DIR}/include "C:/frc/usr/include")
else ()
    set(WPILIB_INCLUDE_DIR ${WPILIB_CPP_DIR}/include)
endif ()


# Allows us to build TestRobotProgram
if (${RIO})
    link_directories(${WPILIB_CPP_DIR}/reflib/linux/athena/shared ${WPILIB_COMMON_DIR}/lib/linux/athena/shared)
endif ()

add_subdirectory(libraries)
#############################################################################

set(phil_include_dir ${CMAKE_SOURCE_DIR}/include)
file(GLOB common_src src/common/*.cpp)
file(GLOB localization_src src/localization/*.cpp)
file(GLOB phil_rio_src src/phil_rio/*.cpp)
file(GLOB main_src src/main/*.cpp)

add_subdirectory(tools)


# Phil RIO library. Only when cross-compiling
if (${RIO})
    add_library(phil_rio ${phil_rio_src} ${common_src} ${localization_src})
    target_include_directories(phil_rio PRIVATE ${phil_include_dir} ${WPILIB_INCLUDE_DIR} ${NAVX_INCLUDE_DIR})

    # Test program
    add_executable(TestRobotProgram src/test/TestRobotProgram.cpp)
    target_include_directories(TestRobotProgram PRIVATE ${phil_include_dir} ${WPILIB_INCLUDE_DIR} ${NAVX_INCLUDE_DIR})
    target_link_libraries(TestRobotProgram phil_rio navx_frc_cpp wpi ntcore wpiutil)

    # install
    install(TARGETS phil_rio DESTINATION ${WPILIB_DIR}/user/cpp/lib)
    install(DIRECTORY include/phil DESTINATION ${WPILIB_DIR}/user/cpp/include FILES_MATCHING PATTERN "*.h")
    install(DIRECTORY ${CMAKE_SOURCE_DIR}/libraries/navxmxp/roborio/c++/navx_frc_cpp/include/
            DESTINATION ${WPILIB_DIR}/user/cpp/include FILES_MATCHING PATTERN "*.h")
endif ()

message(">> ${orocos-bfl_INCLUDE_DIRS}")

if (NOT ${RIO})
    add_executable(phil_main ${main_src} ${common_src} ${localization_src})
    target_include_directories(phil_main PRIVATE
        ${phil_include_dir}
        ${OpenCV_INCLUDE_DIRS}
        ${WPIUTIL_INCLUDE_DIR}
        ${NTCORE_INCLUDE_DIR}
        ${CSCORE_INCLUDE_DIR}
        ${orocos-bfl_INCLUDE_DIRS})
    target_link_libraries(phil_main ntcore cscore wpiutil ${phil_opencv_libs} aruco)

    add_executable(kf_test src/main/phil_kalman_filter.cpp ${localization_src})
    target_include_directories(kf_test PRIVATE
            ${phil_include_dir}
            ${OpenCV_INCLUDE_DIRS}
            ${WPIUTIL_INCLUDE_DIR}
            ${NTCORE_INCLUDE_DIR}
            ${CSCORE_INCLUDE_DIR}
            ${CMAKE_SOURCE_DIR}/include/eigen3
            ${orocos-bfl_INCLUDE_DIRS})
    target_link_libraries(kf_test ${orocos-bfl_LIBRARIES})

    add_executable(ekf_test src/main/test_nonlinear_kalman.cpp ${localization_src})
    target_include_directories(ekf_test PRIVATE
        ${phil_include_dir}
        ${OpenCV_INCLUDE_DIRS}
        ${WPIUTIL_INCLUDE_DIR}
        ${NTCORE_INCLUDE_DIR}
        ${CSCORE_INCLUDE_DIR}
        ${CMAKE_SOURCE_DIR}/include/eigen3
        ${orocos-bfl_INCLUDE_DIRS})
    target_link_libraries(ekf_test ${orocos-bfl_LIBRARIES})

    # custom target to scp the program to the TK1 and run it?
endif ()
