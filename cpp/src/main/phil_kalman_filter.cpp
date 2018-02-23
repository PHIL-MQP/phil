#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/analyticconditionalgaussian.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>

#include <phil/common/args.h>
#include <phil/common/csv.h>
#include <phil/common/common.h>
#include <phil/localization/robot_model.h>
#include <fstream>

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

  // The next two matrices define process noise (Q)
  MatrixWrapper::ColumnVector process_noise_mean(RobotStateModel::N);
  process_noise_mean = 0.0;
  MatrixWrapper::SymmetricMatrix process_noise_covariance(RobotStateModel::N);
  process_noise_covariance = 0.0;

  BFL::Gaussian process_uncertainty(process_noise_mean, process_noise_covariance);
  RobotStateModel process_pdf(process_uncertainty);
  BFL::AnalyticSystemModelGaussianUncertainty system_model(&process_pdf);

  // Construct measurement models for each of our sensor packages, rio, camera, and beacon
  constexpr int rio_H_dim = 3;
  MatrixWrapper::Matrix rio_H(rio_H_dim, RobotStateModel::N);
  rio_H << 0, 0, 0, 0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 1, 0,
      0, 0, 1, 0, 0, 0, 0, 0, 0;
  MatrixWrapper::ColumnVector rio_measurement_mean(rio_H_dim);
  rio_measurement_mean = 0;
  MatrixWrapper::SymmetricMatrix rio_measurement_covariance(rio_H_dim);
  rio_measurement_covariance = 0;
  rio_measurement_covariance(1, 1) = 0.0001;
  rio_measurement_covariance(2, 2) = 0.0001;
  rio_measurement_covariance(3, 3) = 0.01;
  BFL::Gaussian rio_measurement_uncertainty(rio_measurement_mean, rio_measurement_covariance);
  RioModel encoder_rio_measurement_pdf(rio_measurement_uncertainty);
  BFL::AnalyticMeasurementModelGaussianUncertainty encoder_rio_measurement_model(&encoder_rio_measurement_pdf);
  BFL::LinearAnalyticConditionalGaussian acc_rio_measurement_pdf(rio_H, rio_measurement_uncertainty);
  BFL::LinearAnalyticMeasurementModelGaussianUncertainty acc_rio_measurement_model(&acc_rio_measurement_pdf);

  // Construct gaussian for initial estimate
  MatrixWrapper::ColumnVector prior_mean(RobotStateModel::N);
  prior_mean << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  MatrixWrapper::SymmetricMatrix prior_covariance(RobotStateModel::N);
  prior_covariance = 0;
  prior_covariance(1, 1) = 0.001;
  prior_covariance(2, 2) = 0.001;
  prior_covariance(3, 3) = 0.000001;
  prior_covariance(4, 4) = 0.00000001;
  prior_covariance(5, 5) = 0.00000001;
  prior_covariance(6, 6) = 0.00000001;
  prior_covariance(7, 7) = 0.0000001;
  prior_covariance(8, 8) = 0.0000001;
  prior_covariance(9, 9) = 0.00000001;
  BFL::Gaussian prior(prior_mean, prior_covariance);

  BFL::ExtendedKalmanFilter filter(&prior);

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

    std::cout << filter.PostGet()->ExpectedValueGet().format(csv_fmt)
              << ","
              << filter.PostGet()->CovarianceGet().diagonal().format(csv_fmt)
              << ","
              << accumulated_yaw_rad
              << std::endl;

    // convert ticks per second to meters per second
    constexpr double meters_per_tick = 0.000357;
    double v_l  = -encoder_l * meters_per_tick;
    double v_r  = -encoder_r * meters_per_tick;

    // Create input vectors (use only on of these)
    MatrixWrapper::ColumnVector acc_input(RobotStateModel::M);
    acc_input(1) = ax;
    acc_input(2) = ay;

    MatrixWrapper::ColumnVector encoder_input(RobotStateModel::M);
    encoder_input(1) = v_l;
    encoder_input(2) = v_r;

    // The NavX gives us angles (-180/180), we want to unwrap this to (-\infty,\infty)
    double yaw_rad = yaw * M_PI / 180.0;
    auto d_yaw_rad = phil::yaw_diff_rad(yaw_rad, last_yaw_rad);
    last_yaw_rad = yaw_rad;
    accumulated_yaw_rad += d_yaw_rad;

    // Create RoboRIO measurement vectors (use only on of these)
    MatrixWrapper::ColumnVector acc_rio_measurement(rio_H_dim);
    acc_rio_measurement << ax, ay, accumulated_yaw_rad;

    MatrixWrapper::ColumnVector encoder_rio_measurement(rio_H_dim);
    acc_rio_measurement << v_l, v_r, accumulated_yaw_rad;

//    filter.Update(&system_model, acc_input, &encoder_rio_measurement_model, encoder_rio_measurement);
    filter.Update(&system_model, encoder_input, &acc_rio_measurement_model, acc_rio_measurement);

    // If you want to run just the prediction update, you can run just not pass in measurements
//    filter.Update(&system_model, acc_input);
//    filter.Update(&system_model, encoder_input);

    ++idx;
  }

  return EXIT_SUCCESS;
}
