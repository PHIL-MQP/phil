file(GLOB_RECURSE wpiutil_native_src wpiutil/src/main/native/cpp/*.cpp)
add_library(wpiutil ${wpiutil_native_src})
target_include_directories(wpiutil PRIVATE ${WPIUTIL_INCLUDE_DIR})

file(GLOB
    ntcore_native_src ntcore/src/main/native/cpp/*.cpp
    ntcore_native_src ntcore/src/main/native/cpp/networktables/*.cpp
    ntcore_native_src ntcore/src/main/native/cpp/tables/*.cpp)
add_library(ntcore ${ntcore_native_src})
target_link_libraries(ntcore wpiutil)
target_include_directories(ntcore PRIVATE ${WPIUTIL_INCLUDE_DIR} ${NTCORE_INCLUDE_DIR})

file(GLOB cscore_native_src cscore/src/main/native/cpp/*.cpp)
add_library(cscore ${cscore_native_src})
target_link_libraries(cscore ntcore wpiutil ${OpenCV_LIBS})
target_include_directories(cscore PRIVATE ${WPIUTIL_INCLUDE_DIR} ${NTCORE_INCLUDE_DIR} ${CSCORE_INCLUDE_DIR})
