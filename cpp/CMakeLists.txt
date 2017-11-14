project(tk1_wpiutil_test)
cmake_minimum_required(VERSION 2.8.0)

# flags and config
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++11")

set(WPIUTIL_INCLUDE_DIR libraries/wpiutil/src/main/native/include)
set(NTCORE_INCLUDE_DIR libraries/ntcore/src/main/native/include)
set(CSCORE_INCLUDE_DIR libraries/cscore/src/main/native/include)
include_directories(${WPIUTIL_INCLUDE_DIR} ${NTCORE_INCLUDE_DIR} ${CSCORE_INCLUDE_DIR})

add_subdirectory(libraries)
