#pragma once

#include <cstdint>
#include <cstdio>
#include <string>
#include <sstream>
#include <sys/time.h>
#include <string>
#include <netinet/in.h>

namespace phil {

constexpr uint16_t kPort = 6789;

struct data_t {
  double world_acc_x;
  double world_acc_y;
  double raw_acc_x;
  double raw_acc_y;
  double raw_acc_z;
  double yaw;
  double left_encoder_rate;
  double right_encoder_rate;
  double fpga_t;
  long navx_t;
  double rio_send_time_s;
  double received_time_s;

  std::string to_string() {
    std::stringstream ss;
    ss << raw_acc_x << ","
       << raw_acc_y << ","
       << raw_acc_z << ","
       << world_acc_x << ","
       << world_acc_y << ","
       << yaw << ","
       << left_encoder_rate << ","
       << right_encoder_rate << ","
       << fpga_t << ","
       << navx_t << ",";
    return ss.str();
  }

  static std::string header() {
    return std::string("raw_accel_x,raw_accel_y,raw_accel_z,"
                       "world_accel_x,world_accel_y,"
                       "yaw,"
                       "x,y,z,"
                       "left_encoder_rate,right_encoder_rate,"
                       "fpga time,navx time");
  }
};

inline void print_data_t(data_t d) {
  printf("rio:%f, main:%f\n", d.rio_send_time_s, d.received_time_s);
};

inline double timeval_to_sec(struct timeval tv) {
  return tv.tv_sec + (float) tv.tv_usec / 1e6;
}

constexpr size_t data_t_size = sizeof(data_t);
extern socklen_t sockaddr_size;

class UDPServer {
 public:
  explicit UDPServer(int16_t port_num = kPort);

  /**
   * Blocks until the next packet is received
   * @param result This functions fills the result pointer with data
   * @return pair of the number of bytes actually received and put in data and the address to respond to
   */
  std::pair<ssize_t, struct sockaddr_in> Read(phil::data_t *result);

  /**
   * Read but just return how much and don't return the data or socket of the client who sent it
   */
  ssize_t Read();

  ssize_t Reply(struct sockaddr_in client, phil::data_t reply);

  /**
   * Sets the timeout for future calls to sendto and recvfrom
   * @param timeout timeout
   */
  void SetTimeout(timeval timeout);

 private:
  int socket_fd;
};

class UDPClient {
 public:
  explicit UDPClient(const std::string &server_hostname, int port_num = kPort);

  /**
   * Sends data to TK1. This assumes data has been filled and stamped. This function may block for up to 1 second.
   * @return The data you send the TK1 but now with the TK1 time stamp in it
   */
  data_t Transaction(data_t data);

  /**
   * Blocks until the next packet is received
   * @param response the functions fills this pointer with data
   * @param response_size the amount of data you expect to receive in bytes
   * @return the number of bytes actuall received and put in data
   */
  ssize_t Read(uint8_t *response, size_t response_size);

  void RawTransaction(uint8_t *request, size_t request_size, uint8_t *response, size_t response_size);

  /**
   * Sets the timeout for future calls to sendto and recvfrom
   * @param timeout timeout
   */
  void SetTimeout(timeval timeout);

 private:
  int socket_fd;
  std::string server_hostname;
  struct sockaddr_in client_addr;
  struct sockaddr_in server_addr;

};

} // end namespace
