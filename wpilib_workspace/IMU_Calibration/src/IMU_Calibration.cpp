#include <unistd.h>
#include <exception>
#include <iostream>
#include <iomanip>
#include <fstream>

#include <SPI.h>
#include <HAL/HAL.h>

#include <IMU_Calibration.h>
#include "csvReader.h"
/**
 * This programs logs navX MXP values to a file for use in IMU Calibration
 *
 * The output data values include:
 *
 * - Raw Gyro and Accelerometer Data
 *
 */
int main(int argc, char **argv) {
  int err_code = manditory_init();
  if (err_code) {
    return err_code;
  }

  std::cout << "starting capture" << std::endl;

  AHRS *ahrs;

  try {
    ahrs = new AHRS(SPI::Port::kMXP);
    err_code = collectSamples(7500, ahrs);
    if (err_code) {
      return err_code;
    }
  } catch (std::exception& ex) {
    printf("Error instantiating navX MXP: %s", ex.what());
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

int collectSamples(size_t number_of_samples, AHRS *ahrs) {
  const size_t microsec_per_sample = 10000;
  long long elapsed = 0;
  size_t c = 0;
  struct timeval t1;
  struct timeval t0;
  struct timezone tz;

  typedef struct ins {
    float accel[3];
    float gyro[3];
    struct timeval t0;
    struct timeval t1;
  } INS;

  gettimeofday(&t0, &tz);
  gettimeofday(&t1, &tz);
  INS temp;

  // open file for logging
  std::ofstream log;
  log.open("imu_calibration_data.csv");
  if (!log.good()) {
    return -1;
  }

  // write headers
  log << "accelx,accely,accelz,gyrox,gyroy,gyroz,time" << std::endl;
  while (c < number_of_samples) {
    // This buffer shouldn't be too big or you'll segfault everything!
    const size_t buff_size = 2000;
    INS buffer[buff_size];
    for (size_t i = 0; i < buff_size && c < number_of_samples; ) {
      gettimeofday(&t1, &tz);
      elapsed = (t1.tv_sec - t0.tv_sec) * 1e6 + t1.tv_usec - t0.tv_usec;


      //100 samples per second (Hz)
      if (elapsed > microsec_per_sample) {
        gettimeofday(&temp.t0, &tz);
        temp.accel[0] = ahrs->GetRawAccelX();
        temp.accel[1] = ahrs->GetRawAccelY();
        temp.accel[2] = ahrs->GetRawAccelZ();
        temp.gyro[0] = ahrs->GetRawGyroX();
        temp.gyro[1] = ahrs->GetRawGyroY();
        temp.gyro[2] = ahrs->GetRawGyroZ();
        gettimeofday(&temp.t1, &tz);
        memcpy(&buffer[i], &temp, sizeof temp);
        gettimeofday(&t0, &tz);
        c++;
        i++;
      }
    }

    std::cout << "saving some samples to file" << std::endl;
    for (size_t i = 0; i < buff_size; i++) {
      log << std::setw(9)
          << buffer[i].accel[0] << ","
          << buffer[i].accel[1] << ","
          << buffer[i].accel[2] << ","
          << buffer[i].gyro[0] << ","
          << buffer[i].gyro[1] << ","
          << buffer[i].gyro[2] << ","
          << time_to_sec(buffer[i].t1) << std::endl;
    }
  }

  log.close();
  printf("%d samples collected\n", c);

  return 0;
}

int manditory_init() {
  if (!HAL_Initialize(0)) {
    std::cout << "FATAL ERROR: HAL could not be initialized" << std::endl;
    return -1;
  }

  return 0;
}


double time_to_sec(struct timeval tv) {
  return tv.tv_sec + (float) tv.tv_usec / 1000000.0;
}

