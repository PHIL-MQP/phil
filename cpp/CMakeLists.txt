project(phil)
cmake_minimum_required(VERSION 2.8.0)

option(RIO "cross-compile for RoboRIO" OFF)
option(TK1 "cross-compile for TK1" OFF)
if (${RIO})
    set(CMAKE_CXX_COMPILER "arm-frc-linux-gnueabi-g++")
endif ()

#############################################################################
## Dependencies
#############################################################################
# flags and config
find_package(OpenCV REQUIRED)
find_package(aruco REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++11")

set(WPILIB_DIR $ENV{HOME}/wpilib)
set(WPILIB_CPP_DIR $ENV{HOME}/wpilib/cpp/current)
set(WPIUTIL_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/libraries/wpiutil/src/main/native/include)
set(NTCORE_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/libraries/ntcore/src/main/native/include)
set(CSCORE_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/libraries/cscore/src/main/native/include)
set(WPILIB_INCLUDE_DIR ${WPILIB_CPP_DIR}/include)
set(WPILIB_USER_INCLUDE_DIR ${WPILIB_DIR}/user/cpp/include)
set(WPILIB_LIBRARY_DIR ${WPILIB_CPP_DIR}/lib)
set(WPILIB_LIBRARIES wpi)
set(NAVX_LIBRARIES navx_frc_cpp)

add_subdirectory(libraries)
#############################################################################

set(phil_include_dir ${CMAKE_SOURCE_DIR}/include)
file(GLOB common_src src/common/*.cpp)
file(GLOB phil_src src/phil/*.cpp)
file(GLOB tk1_src src/tk1/*.cpp)

add_subdirectory(tools)

# Phil RIO library. Only when cross-compiling
if (${RIO})
    add_library(phil ${phil_src})
    target_include_directories(phil PRIVATE ${PHIL_INCLUDE_DIR} ${WPILIB_INCLUDE_DIR} ${WPILIB_USER_INCLUDE_DIR})
    target_link_libraries(phil ${WPILIB_LIBRARIES} ${NAVX_LIBRARIES})

    # install
    install(TARGETS phil DESTINATION $ENV{HOME}/wpilib/user/cpp/lib)
    install(DIRECTORY include/phil DESTINATION ${WPILIB_DIR}/user/cpp/include FILES_MATCHING PATTERN "*.h")
endif ()

if (${TK1})
    # build the executable that runs on the TK1
    add_executable(phil_tk1 ${tk1_src})
    target_include_directories(phil_tk1 PRIVATE ${WPIUTIL_INCLUDE_DIR} ${NTCORE_INCLUDE_DIR} ${CSCORE_INCLUDE_DIR})
    target_link_libraries(phil_tk1 ntcore cscore wpiutil)

    # custom target to scp the program to the TK1 and run it?
endif ()
