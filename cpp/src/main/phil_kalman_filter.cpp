#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/analyticconditionalgaussian.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>

#include <phil/common/args.h>
#include <phil/common/csv.h>
#include <fstream>

// Params of the robot
constexpr float R = 0.9;
constexpr float W = 0.0762;

// Number of state variables
constexpr unsigned int N = 9;

// Number of control variables
constexpr unsigned int M = 2;

// This is assumed to be constant
constexpr float dt_s = 0.02;

MatrixWrapper::Matrix Bmatrix(float yaw) {
  MatrixWrapper::Matrix B(N, M);
  B << cos(yaw) * 0.25 * pow(dt_s, 2), cos(yaw) * 0.25 * pow(dt_s, 2),
      sin(yaw) * 0.25 * pow(dt_s, 2), sin(yaw) * 0.25 * pow(dt_s, 2),
      R * pow(dt_s, 2) / (2 * W), -R * pow(dt_s, 2) / (2 * W),
      cos(yaw) * 0.5 * dt_s, cos(yaw) * 0.5 * dt_s,
      sin(yaw) * 0.5 * dt_s, sin(yaw) * 0.5 * dt_s,
      R * dt_s / W, -R * dt_s / W,
      0, 0,
      0, 0,
      0, 0;

  return B;
}

int main(int argc, const char **argv) {
  args::ArgumentParser parser("This program runs an EKF on the recorded data");
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
  io::CSVReader<7> reader(infile);
  reader.read_header(io::ignore_extra_column,
                     "world_accel_x",
                     "world_accel_y",
                     "yaw",
                     "left_encoder_rate",
                     "right_encoder_rate",
                     "left_input",
                     "right_input");

  MatrixWrapper::Matrix A(N, N);
  A << 1, 0, 0, dt_s, 0, 0, 0.5 * dt_s, 0, 0,
      0, 1, 0, 0, dt_s, 0, 0, 0.5 * dt_s, 0,
      0, 0, 1, 0, 0, dt_s, 0, 0, 0.5 * dt_s,
      0, 0, 0, 1, 0, 0, dt_s, 0, 0,
      0, 0, 0, 0, 1, 0, 0, dt_s, 0,
      0, 0, 0, 0, 0, 1, 0, 0, dt_s,
      0, 0, 0, 0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 1;

  // represents dynamics of a robot
  // since the B matrix is a function of our state, we will update it every iteration
  // but for now it's just going to be calculated with some initial state values plugged in
  auto B = Bmatrix(0.f);

  std::vector<MatrixWrapper::Matrix> AB(2);
  AB[0] = A;
  AB[1] = B;

  // The next two matrices define process noise (Q)
  MatrixWrapper::ColumnVector process_noise_mean(N);
  process_noise_mean = 0.0;
  MatrixWrapper::SymmetricMatrix process_noise_covariance(N);
  process_noise_covariance = 0.0;

  BFL::Gaussian process_uncertainty(process_noise_mean, process_noise_covariance);
  BFL::LinearAnalyticConditionalGaussian process_pdf(AB, process_uncertainty);
  BFL::LinearAnalyticSystemModelGaussianUncertainty system_model(&process_pdf);

  // Construct measurement models for each of our sensor packages, rio, camera, and beacon
  MatrixWrapper::Matrix rio_H(5, N);
  rio_H << 0, 0, 0, 0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 1, 0,
      0, 0, 1, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0;
  MatrixWrapper::ColumnVector rio_measurement_mean(5);
  rio_measurement_mean = 0;
  MatrixWrapper::SymmetricMatrix rio_measurement_covariance(5);
  rio_measurement_covariance = 0;
  rio_measurement_covariance(1, 1) = 0.001;
  rio_measurement_covariance(2, 2) = 0.001;
  rio_measurement_covariance(3, 3) = 0.00001;
  rio_measurement_covariance(4, 4) = 0.00001;
  rio_measurement_covariance(5, 5) = 0.00001;
  BFL::Gaussian rio_measurement_uncertainty(rio_measurement_mean, rio_measurement_covariance);
  BFL::LinearAnalyticConditionalGaussian rio_measurement_pdf(rio_H, rio_measurement_uncertainty);
  BFL::LinearAnalyticMeasurementModelGaussianUncertainty rio_measurement_model(&rio_measurement_pdf);

  // Construct gaussian for initial estimate
  MatrixWrapper::ColumnVector prior_mean(N);
  prior_mean << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  MatrixWrapper::SymmetricMatrix prior_covariance(N);
  prior_covariance = 0;
  prior_covariance(1, 1) = 0.001;
  prior_covariance(2, 2) = 0.001;
  prior_covariance(3, 3) = 0.00001;
  prior_covariance(4, 4) = 0.001;
  prior_covariance(5, 5) = 0.001;
  prior_covariance(6, 6) = 0.00001;
  prior_covariance(7, 7) = 0.001;
  prior_covariance(8, 8) = 0.001;
  prior_covariance(9, 9) = 0.00001;
  BFL::Gaussian prior(prior_mean, prior_covariance);

  BFL::ExtendedKalmanFilter filter(&prior);

  // output file and formatting
  const static Eigen::IOFormat csv_fmt(6, Eigen::DontAlignCols, ",", ",");

  // Prediction & Update Loop
  float ax, ay, yaw, wl, wr, al, ar;
  while (reader.read_row(ax, ay, yaw, wl, wr, al, ar)) {
    BFL::Pdf<MatrixWrapper::ColumnVector> *posterior = filter.PostGet();
    std::cout << filter.PostGet()->ExpectedValueGet().format(csv_fmt)
              << ","
              << filter.PostGet()->CovarianceGet().format(csv_fmt)
              << std::endl;

    // update the B matrix given our current state estimate
    float estimated_yaw = static_cast<float>(posterior->ExpectedValueGet()(3));
    process_pdf.MatrixSet(1, Bmatrix(estimated_yaw));

    MatrixWrapper::ColumnVector input(M);
    input(1) = al;
    input(2) = ar;

    MatrixWrapper::ColumnVector rio_measurement(5);
    // FIXME: units are probably wong for wl/wr
    rio_measurement << 9.8 * ax, 9.8 * ay, yaw, wl*0.000357, wr*0.000357;

    filter.Update(&system_model, input, &rio_measurement_model, rio_measurement);

    // If you want to run just the prediction update, you can run just not pass in measurements
//     filter.Update(&system_model, input);
  }

  return EXIT_SUCCESS;
}
