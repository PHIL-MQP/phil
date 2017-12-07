#include <iostream>
#include <fstream>
#include <AHRS.h>
#include <thread>

void show_help() {
  std::cout << "USAGE: ./tools/record_imu_calibration_data device seconds_of_data"
            << std::endl
            << std::endl
            << "EXAMPLE: ./tools/record_imu_calibration_data /dev/ttyACM0 10"
            << std::endl;
}

int main(int argc, char *argv[]) {
  if (argc != 3) {
    show_help();
    return 1;
  }

  AHRS navx = AHRS(argv[1], AHRS::SerialDataType::kRawData, 60);
  double num_seconds = std::stof(argv[2]);
  unsigned int ms_per_sample = 10;
  auto total_samples = static_cast<unsigned int>(num_seconds * (1000.0 / ms_per_sample));

  std::ofstream log;
  log.open("imu_calibration_data.csv");

  if (!log.good()) {
    std::cout << "could not open file." << std::endl;
  }

  for (unsigned int i = 0; i < total_samples; i++) {
    // make a reading
    log << navx.GetRawAccelX() << ","
        << navx.GetRawAccelY() << ","
        << navx.GetRawAccelZ() << ","
        << navx.GetRawGyroX() << ","
        << navx.GetRawGyroY() << ","
        << navx.GetRawGyroZ() << ","
        << navx.GetLastSensorTimestamp()
        << std::endl;

    // sleep
    std::this_thread::sleep_for(std::chrono::milliseconds(ms_per_sample));
  }

  return 0;
}
