#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/analyticconditionalgaussian.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>

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

int main(int argc, char **argv) {

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
  BFL::LinearAnalyticMeasurementModelGaussianUncertainty rio_measurement_model;
  {
    MatrixWrapper::Matrix H(5, N);
    H << 0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 1, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0;
    MatrixWrapper::ColumnVector measurement_noise_mean(5);
    measurement_noise_mean = 0;
    MatrixWrapper::SymmetricMatrix measurement_noise_covariance(5);
    measurement_noise_covariance = 0;
    measurement_noise_covariance(1, 1) = 0.0001;
    measurement_noise_covariance(2, 2) = 0.0001;
    measurement_noise_covariance(3, 3) = 0.0001;
    measurement_noise_covariance(4, 4) = 0.0001;
    measurement_noise_covariance(5, 5) = 0.0001;
    BFL::Gaussian measurement_uncertainty(measurement_noise_mean, measurement_noise_covariance);
    BFL::LinearAnalyticConditionalGaussian measurement_pdf(H, measurement_uncertainty);
    rio_measurement_model = BFL::LinearAnalyticMeasurementModelGaussianUncertainty(&measurement_pdf);
  }

  // Construct gaussian for initial estimate
  MatrixWrapper::ColumnVector prior_mean(N);
  prior_mean << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  MatrixWrapper::SymmetricMatrix prior_covariance(N);
  prior_covariance = 0;
  prior_covariance(1,1) = 0.001;
  prior_covariance(2,2) = 0.001;
  prior_covariance(3,3) = 0.001;
  prior_covariance(4,4) = 0.001;
  prior_covariance(5,5) = 0.001;
  prior_covariance(6,6) = 0.001;
  prior_covariance(7,7) = 0.001;
  prior_covariance(8,8) = 0.001;
  prior_covariance(9,9) = 0.001;
  BFL::Gaussian prior(prior_mean, prior_covariance);

  BFL::ExtendedKalmanFilter filter(&prior);

  // Prediction & Update Loop
  for (size_t  i = 0; i < 5; i++ ){
    BFL::Pdf<MatrixWrapper::ColumnVector> *posterior = filter.PostGet();
    // update the B matrix given our current state estimate
    float yaw = static_cast<float>(posterior->ExpectedValueGet()(3));
    process_pdf.MatrixSet(1, Bmatrix(yaw));
    MatrixWrapper::ColumnVector input(M);

  //  input(1) = left_input;
  //  input(2) = right_input;

    MatrixWrapper::ColumnVector rio_measurement(5);

//    filter.Update(&system_model, input, &rio_measurement_model, rio_measurement);
    filter.Update(&system_model, input);
  }

  return EXIT_SUCCESS;
}
