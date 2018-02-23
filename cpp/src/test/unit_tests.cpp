#include<iostream>
#include <cstdlib>

#include <phil/common/common.h>

int main(int argc, const char **argv) {

  assert(phil::yaw_diff_rad(0.1, 0) == 0.1);
  assert(phil::yaw_diff_rad(0.5, 0) == 0.5);
  assert(phil::yaw_diff_rad(0, 0.1) == -0.1);
  assert(phil::yaw_diff_rad(0, 0.5) == -0.5);
  assert(phil::yaw_diff_rad(0, 2 * M_PI) == 0);
  assert(phil::yaw_diff_rad(2 * M_PI, 0) == 0);
  assert(fabs(phil::yaw_diff_rad(0.1,2*M_PI - 0.1) - (0.2)) < 1e-9);
  assert(fabs(phil::yaw_diff_rad(2*M_PI - 0.1, 0.1) - (-0.2)) < 1e-9);
  assert(phil::yaw_diff_deg(1, 359) == 2);
  assert(phil::yaw_diff_deg(359, 1) == -2);

  return EXIT_SUCCESS;
}
