#pragma once

#include <bfl/pdf/analyticconditionalgaussian_additivenoise.h>

class EncoderControlModel : public BFL::AnalyticConditionalGaussianAdditiveNoise {
 public:
  explicit EncoderControlModel(const BFL::Gaussian &additiveNoise);

  // redefine virtual functions
  MatrixWrapper::ColumnVector ExpectedValueGet() const override;

  MatrixWrapper::Matrix dfGet(unsigned int i) const override;

  static constexpr double dt_s = 0.02;

  // Params of the robot
  static constexpr double W = 0.9;  // track width in meters

  // kinematics model parameters
  static constexpr double alpha = 0.75;

  // Number of state variables
  static constexpr unsigned int N = 9;

  // Number of control variables
  static constexpr unsigned int M = 2;
};

class AccelerometerControlModel : public BFL::AnalyticConditionalGaussianAdditiveNoise {
 public:
  explicit AccelerometerControlModel(const BFL::Gaussian &additiveNoise);

  // redefine virtual functions
  MatrixWrapper::ColumnVector ExpectedValueGet() const override;

  MatrixWrapper::Matrix dfGet(unsigned int i) const override;

  static constexpr double dt_s = 0.02;

  // Params of the robot
  static constexpr double W = 0.9;  // track width in meters

  // kinematics model parameters
  static constexpr double alpha = 0.75;

  // Number of state variables
  static constexpr unsigned int N = 9;

  // Number of control variables
  // - acc in x
  // - acc in y
  // - gyro rate in yaw
  static constexpr unsigned int M = 3;
};

class AccMeasurementModel : public BFL::AnalyticConditionalGaussianAdditiveNoise {
 public:
  explicit AccMeasurementModel(const BFL::Gaussian &additiveNoise);

  // redefine virtual functions
  MatrixWrapper::ColumnVector ExpectedValueGet() const override;

  MatrixWrapper::Matrix dfGet(unsigned int i) const override;
};
