#pragma once

#include <bfl/pdf/analyticconditionalgaussian_additivenoise.h>

class RobotStateModel : public BFL::AnalyticConditionalGaussianAdditiveNoise {
 public:
  explicit RobotStateModel(const BFL::Gaussian &additiveNoise);

  // redefine virtual functions
  MatrixWrapper::ColumnVector ExpectedValueGet() const override;

  MatrixWrapper::Matrix dfGet(unsigned int i) const override;

  static constexpr double dt_s = 0.02;

// Params of the robot
  static constexpr double W = 0.9;  // track width in meters

// kinematics model parameters
  static constexpr double alpha = 1;

// Number of state variables
  static constexpr unsigned int N = 9;

// Number of control variables
  static constexpr unsigned int M = 2;
};

class RioModel : public BFL::AnalyticConditionalGaussianAdditiveNoise {
 public:
  explicit RioModel(const BFL::Gaussian &additiveNoise);

  // redefine virtual functions
  MatrixWrapper::ColumnVector ExpectedValueGet() const override;

  MatrixWrapper::Matrix dfGet(unsigned int i) const override;
};
