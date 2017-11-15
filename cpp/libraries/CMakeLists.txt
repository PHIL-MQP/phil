file(GLOB_RECURSE wpiutil_native_src wpiutil/src/main/native/cpp/*.cpp)
add_library(wpiutil ${wpiutil_native_src})

file(GLOB
    ntcore_native_src ntcore/src/main/native/cpp/*.cpp
    ntcore_native_src ntcore/src/main/native/cpp/networktables/*.cpp
    ntcore_native_src ntcore/src/main/native/cpp/tables/*.cpp)
add_library(ntcore ${ntcore_native_src})
target_link_libraries(ntcore wpiutil)

file(GLOB cscore_native_src cscore/src/main/native/cpp/*.cpp)
add_library(cscore ${cscore_native_src})
target_link_libraries(cscore ntcore wpiutil ${OpenCV_LIBS})
