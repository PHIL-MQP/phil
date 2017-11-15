#pragma once

#include <cstdint>

constexpr uint16_t kPort = 6789;

struct data_t {
  double left_encoder_rate;
  double right_encoder_rate;
  double rio_send_time_s;
  double tk1_recv_time_s;
};

inline void print_data_t(data_t d) {
  printf("rio:%f, tk1:%f\n", d.rio_send_time_s, d.tk1_recv_time_s);
};

inline double timeval_to_sec(struct timeval tv) {
  return tv.tv_sec + (float) tv.tv_usec / 1e6;
}

constexpr size_t data_t_size = sizeof(data_t);
