project(phil)
cmake_minimum_required(VERSION 2.8.0)

option(RIO "cross-compile for RoboRIO" OFF)
option(TK1 "cross-compile for TK1" OFF)
if (${RIO})
    set(CMAKE_CXX_COMPILER "arm-frc-linux-gnueabi-g++")
    set(CMAKE_AR "arm-frc-linux-gnueabi-ar" CACHE FILEPATH "Archiver" FORCE)
    set(CMAKE_RANLIB "arm-frc-linux-gnueabi-ranlib" CACHE FILEPATH "Ranlib" FORCE)

    if (${CMAKE_HOST_WIN32})
        include_directories("C:/frc/usr/include")
    endif ()
elseif (${TK1})
    set(CMAKE_CXX_COMPILER "arm-linux-gnueabihf-g++")
endif ()

#############################################################################
## Dependencies
#############################################################################
# flags and config
if (NOT ${RIO})
    find_package(OpenCV REQUIRED)
    find_package(aruco REQUIRED)
endif ()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++11")

if (${CMAKE_HOST_WIN32})
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
set(WPILIB_INCLUDE_DIR ${WPILIB_CPP_DIR}/include)


# Allows us to build TestRobotProgram
if (${RIO})
    link_directories(${WPILIB_CPP_DIR}/reflib/linux/athena/shared ${WPILIB_COMMON_DIR}/lib/linux/athena/shared)
endif ()

add_subdirectory(libraries)
#############################################################################

set(phil_include_dir ${CMAKE_SOURCE_DIR}/include)
file(GLOB phil_common_src src/phil_common/*.cpp)
file(GLOB phil_rio_src src/phil_rio/*.cpp)
file(GLOB tk1_src src/tk1/*.cpp)

add_subdirectory(tools)


# Phil RIO library. Only when cross-compiling
if (${RIO})
    add_library(phil_rio ${phil_rio_src} ${phil_common_src})
    target_include_directories(phil_rio PRIVATE ${phil_include_dir} ${WPILIB_INCLUDE_DIR} ${NAVX_INCLUDE_DIR})

    # Test program
    add_executable(TestRobotProgram src/test/TestRobotProgram.cpp)
    target_include_directories(TestRobotProgram PRIVATE ${phil_include_dir} ${WPILIB_INCLUDE_DIR} ${NAVX_INCLUDE_DIR})
    target_link_libraries(TestRobotProgram phil_rio navx_frc_cpp wpi ntcore wpiutil)

    # install
    install(TARGETS phil_rio DESTINATION ${WPILIB_DIR}/user/cpp/lib)
    install(DIRECTORY include/phil DESTINATION ${WPILIB_DIR}/user/cpp/include FILES_MATCHING PATTERN "*.h")
endif ()

if (${TK1})
    # build the executable that runs on the TK1
    add_executable(phil_tk1 ${tk1_src})
    target_include_directories(phil_tk1 PRIVATE ${WPIUTIL_INCLUDE_DIR} ${NTCORE_INCLUDE_DIR} ${CSCORE_INCLUDE_DIR})
    target_link_libraries(phil_tk1 ntcore cscore wpiutil)

    # custom target to scp the program to the TK1 and run it?
endif ()
