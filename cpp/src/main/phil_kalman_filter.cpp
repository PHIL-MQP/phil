#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>

#include <phil/common/args.h>
#include <phil/common/csv.h>
#include <phil/common/common.h>
#include <phil/localization/robot_model.h>
#include <fstream>
#include <phil/localization/ekf.h>

int main(int argc, const char **argv) {
  args::ArgumentParser
      parser("This program runs an EKF on the recorded data. You'll want to redirect the output of this to a file.\n"
                 "Currently, it expects the input CSV to have specific column names:\n"
                 " - world_accel_x (m/s^2)\n"
                 " - world_accel_y (m/s^2)\n"
                 " - yaw (degrees)\n"
                 " - left_encoder_rate (ticks/second)\n"
                 " - right_encoder_rate (ticks/second)\n");
  args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});
  args::Positional<std::string>
      infile_arg(parser, "infile", "input csv of data recorded on roborio", args::Options::Required);

  try {
    parser.ParseCLI(argc, argv);
  }
  catch (args::Help &e) {
    std::cout << parser;
    return EXIT_SUCCESS;
  }
  catch (args::RequiredError &e) {
    std::cout << parser;
    return EXIT_FAILURE;
  }

  std::string infile = args::get(infile_arg);
  io::CSVReader<5> reader(infile);
  reader.read_header(io::ignore_extra_column,
                     "world_accel_x",
                     "world_accel_y",
                     "yaw",
                     "left_encoder_rate",
                     "right_encoder_rate");

  // Third for the encoder measurements
  MatrixWrapper::ColumnVector encoder_measurement_mean(2);
  encoder_measurement_mean = 0;
  MatrixWrapper::SymmetricMatrix encoder_measurement_covariance(2);
  encoder_measurement_covariance = 0;
  encoder_measurement_covariance(1, 1) = 0.0001;
  encoder_measurement_covariance(2, 2) = 0.0001;
  BFL::Gaussian encoder_measurement_uncertainty(encoder_measurement_mean, encoder_measurement_covariance);
  AccMeasurementModel encoder_measurement_pdf(encoder_measurement_uncertainty);
  BFL::AnalyticMeasurementModelGaussianUncertainty encoder_measurement_model(&encoder_measurement_pdf);

  phil::EKF ekf;

  // output file and formatting
  const static Eigen::IOFormat csv_fmt(6, Eigen::DontAlignCols, ",", ",");

  // Prediction & Update Loop
  double ax, ay, yaw, encoder_l, encoder_r;
  size_t idx = 0;
  while (reader.read_row(ax, ay, yaw, encoder_l, encoder_r)) {
    // account for inverted yaw readings
    yaw = -yaw;

    static double accumulated_yaw_rad = yaw * M_PI / 180.0;
    static double last_yaw_rad = yaw * M_PI / 180.0;

    std::cout << ekf.filter->PostGet()->ExpectedValueGet().format(csv_fmt)
              << ","
              << ekf.filter->PostGet()->CovarianceGet().diagonal().format(csv_fmt)
              << ","
              << accumulated_yaw_rad
              << std::endl;

    // convert ticks per second to meters per second
    constexpr double meters_per_tick = 0.000357;
    double v_l = -encoder_l * meters_per_tick;
    double v_r = -encoder_r * meters_per_tick;

    // Create input vectors
    MatrixWrapper::ColumnVector acc_and_gyro_input(AccelerometerControlModel::M);
    acc_and_gyro_input(1) = ax;
    acc_and_gyro_input(2) = ay;
    acc_and_gyro_input(3) = 0; // FIXME: use gz? compute from sequential yaw readings?;

    MatrixWrapper::ColumnVector encoder_input(2);
    encoder_input(1) = v_l;
    encoder_input(2) = v_r;

    // The NavX gives us angles (-180/180), we want to unwrap this to (-\infty,\infty)
    double yaw_rad = yaw * M_PI / 180.0;
    auto d_yaw_rad = phil::yaw_diff_rad(yaw_rad, last_yaw_rad);
    last_yaw_rad = yaw_rad;
    accumulated_yaw_rad += d_yaw_rad;

    // Create RoboRIO measurement vectors
    MatrixWrapper::ColumnVector yaw_measurement(1);
    yaw_measurement << accumulated_yaw_rad;

    MatrixWrapper::ColumnVector acc_measurement(2);
    acc_measurement << ax, ay;

    MatrixWrapper::ColumnVector encoder_measurement(2);
    acc_measurement << v_l, v_r;

//    filter.Update(&acc_system_model, acc_input);
    ekf.filter->Update(ekf.encoder_system_model.get(), encoder_input);

    ekf.filter->Update(ekf.yaw_measurement_model.get(), yaw_measurement);
//    filter.Update(&encoder_measurement_model, encoder_measurement);
    ekf.filter->Update(ekf.acc_measurement_model.get(), acc_measurement);


    ++idx;
  }

  return EXIT_SUCCESS;
}
