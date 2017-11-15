add_executable(udp_server udp_server.cpp)
add_executable(udp_client udp_client.cpp)

add_executable(fisheye unwarp_fisheye.cpp)
target_link_libraries(fisheye ${OpenCV_LIBS})
