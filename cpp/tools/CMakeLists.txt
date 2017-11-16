add_executable(udp_server udp_server.cpp)
target_include_directories(udp_server PRIVATE ${PHIL_INCLUDE_DIR})

add_executable(udp_client udp_client.cpp)
target_include_directories(udp_client PRIVATE ${PHIL_INCLUDE_DIR})

add_executable(fisheye unwarp_fisheye.cpp)
target_link_libraries(fisheye ${OpenCV_LIBS})
