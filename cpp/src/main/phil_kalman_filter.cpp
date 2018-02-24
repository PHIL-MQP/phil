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
  MatrixWrapper::ColumnVector encoder_system_noise_mean(EncoderControlModel::N);
  encoder_system_noise_mean = 0.0;
  MatrixWrapper::SymmetricMatrix encoder_system_noise_covariance(EncoderControlModel::N);
  encoder_system_noise_covariance = 0.0;

  BFL::Gaussian encoder_system_uncertainty(encoder_system_noise_mean, encoder_system_noise_covariance);
  EncoderControlModel encoder_system_pdf(encoder_system_uncertainty);
  BFL::AnalyticSystemModelGaussianUncertainty encoder_system_model(&encoder_system_pdf);

  // Construct measurement models for each of our sensor packages
  // First for the yaw measurement which comes from the NavX on the RoboRIO
  MatrixWrapper::Matrix yaw_measurement_H(1, EncoderControlModel::N);
  yaw_measurement_H << 0, 0, 1, 0, 0, 0, 0, 0, 0;
  MatrixWrapper::ColumnVector yaw_measurement_mean(1);
  yaw_measurement_mean = 0;
  MatrixWrapper::SymmetricMatrix yaw_measurement_covariance(1);
  yaw_measurement_covariance = 0.01; // TODO: compute actual variance of this!
  BFL::Gaussian yaw_measurement_uncertainty(yaw_measurement_mean, yaw_measurement_covariance);
  BFL::LinearAnalyticConditionalGaussian yaw_measurement_pdf(yaw_measurement_H, yaw_measurement_uncertainty);
  BFL::LinearAnalyticMeasurementModelGaussianUncertainty yaw_measurement_model(&yaw_measurement_pdf);

  // Second for the world-frame accelerometer measurements which comes from the NavX on the RoboRIO
  MatrixWrapper::Matrix acc_measurement_H(2, EncoderControlModel::N);
  acc_measurement_H << 0, 0, 0, 0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 1, 0;
  MatrixWrapper::ColumnVector acc_measurement_mean(2);
  acc_measurement_mean = 0;
  MatrixWrapper::SymmetricMatrix acc_measurement_covariance(2);
  acc_measurement_covariance = 0;
  acc_measurement_covariance(1, 1) = 0.0001;
  acc_measurement_covariance(2, 2) = 0.0001;
  BFL::Gaussian acc_measurement_uncertainty(acc_measurement_mean, acc_measurement_covariance);
  BFL::LinearAnalyticConditionalGaussian acc_measurement_pdf(acc_measurement_H, acc_measurement_uncertainty);
  BFL::LinearAnalyticMeasurementModelGaussianUncertainty acc_measurement_model(&acc_measurement_pdf);

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

  // Construct gaussian for initial estimate
  MatrixWrapper::ColumnVector prior_mean(EncoderControlModel::N);
  prior_mean << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  MatrixWrapper::SymmetricMatrix prior_covariance(EncoderControlModel::N);
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
    filter.Update(&encoder_system_model, encoder_input);

    filter.Update(&yaw_measurement_model, yaw_measurement);
//    filter.Update(&encoder_measurement_model, encoder_measurement);
    filter.Update(&acc_measurement_model, acc_measurement);


    ++idx;
  }

  return EXIT_SUCCESS;
}
