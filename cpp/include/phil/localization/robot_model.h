#pragma once

#include <bfl/pdf/analyticconditionalgaussian_additivenoise.h>

namespace phil {
namespace localization {

double dt_s = 0.02;

// Params of the robot
double W = 0.9;  // track width in meters

// kinematics model parameters
double alpha = 1.6;

// Number of state variables
static constexpr unsigned int N = 9;

// Number of control variables
static constexpr unsigned int M = 2;

class EncoderControlModel : public BFL::AnalyticConditionalGaussianAdditiveNoise {
 public:
  explicit EncoderControlModel(const BFL::Gaussian &additiveNoise, double W, double alpha, double dt_s);

  // redefine virtual functions
  MatrixWrapper::ColumnVector ExpectedValueGet() const override;

  MatrixWrapper::Matrix dfGet(unsigned int i) const override;
};

class AccMeasurementModel : public BFL::AnalyticConditionalGaussianAdditiveNoise {
 public:
  explicit AccMeasurementModel(const BFL::Gaussian &additiveNoise);

  // redefine virtual functions
  MatrixWrapper::ColumnVector ExpectedValueGet() const override;

  MatrixWrapper::Matrix dfGet(unsigned int i) const override;
};

}
}
