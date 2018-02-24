#include <phil/localization/robot_model.h>

constexpr double EncoderControlModel::dt_s;

EncoderControlModel::EncoderControlModel(const BFL::Gaussian &additiveNoise)
    : AnalyticConditionalGaussianAdditiveNoise(additiveNoise, 2) {
}

MatrixWrapper::ColumnVector EncoderControlModel::ExpectedValueGet() const {
  MatrixWrapper::ColumnVector state = ConditionalArgumentGet(0);
  MatrixWrapper::ColumnVector control = ConditionalArgumentGet(1);
  const double v = (control(1) + control(2)) / 2.0;
  state(1) = state(1) + state(4) * dt_s + 0.5 * state(7) * dt_s * dt_s;
  state(2) = state(2) + state(5) * dt_s + 0.5 * state(8) * dt_s * dt_s;
  state(3) = state(3) + state(6) * dt_s;

  state(4) = v * cos(state(3));
  state(5) = v * sin(state(3));
  state(6) = (control(2) - control(1)) / (alpha * W);
  state(7) = state(7);
  state(8) = state(8);
  state(9) = 0;

  return state + AdditiveNoiseMuGet();
}

MatrixWrapper::Matrix EncoderControlModel::dfGet(unsigned int i) const {
  if (i == 0)//derivative to the first conditional argument (x)
  {
    MatrixWrapper::ColumnVector state = ConditionalArgumentGet(0);
    MatrixWrapper::ColumnVector control = ConditionalArgumentGet(1);
    const double v = (control(1) + control(2)) / 2.0;
    MatrixWrapper::Matrix df(N, N);
    df = 0;
    df(1, 1) = 1;
    df(1, 4) = dt_s;
    df(1, 7) = 0.5 * dt_s * dt_s;
    df(2, 2) = 1;
    df(2, 5) = dt_s;
    df(2, 8) = 0.5 * dt_s * dt_s;
    df(3, 3) = 1;
    df(3, 6) = dt_s;
    df(3, 9) = 0;
    df(4, 3) = -v * sin(state(3));
    df(5, 3) = v * cos(state(3));
    df(7, 7) = 1;
    df(8, 8) = 1;
    df(9, 9) = 0;
    return df;
  } else {
    if (i >= NumConditionalArgumentsGet()) {
      std::cerr << "This pdf Only has " << NumConditionalArgumentsGet() << " conditional arguments\n";
      exit(- BFL_ERRMISUSE);
    } else {
      std::cerr << "The df is not implemented for the" << i << "th conditional argument\n";
      exit(- BFL_ERRMISUSE);
    }
  }
}

AccMeasurementModel::AccMeasurementModel(const BFL::Gaussian &additiveNoise)
    : AnalyticConditionalGaussianAdditiveNoise(additiveNoise, 1) {}

MatrixWrapper::ColumnVector AccMeasurementModel::ExpectedValueGet() const {
  MatrixWrapper::ColumnVector measurement(EncoderControlModel::M);
  MatrixWrapper::ColumnVector state = ConditionalArgumentGet(0);
  measurement = 0;
  measurement(1) = state(4) / cos(state(3)) + EncoderControlModel::alpha * EncoderControlModel::W * state(6) / 2.0; // v_l
  measurement(2) = state(4) / cos(state(3)) - EncoderControlModel::alpha * EncoderControlModel::W * state(6) / 2.0; // v_r
  return measurement + AdditiveNoiseMuGet();
}

MatrixWrapper::Matrix AccMeasurementModel::dfGet(unsigned int i) const {
  if (i == 0)//derivative to the first conditional argument (x)
  {
    MatrixWrapper::ColumnVector state = ConditionalArgumentGet(0);
    MatrixWrapper::Matrix df(EncoderControlModel::M, EncoderControlModel::N);
    df = 0;
    df(1, 3) = 2 * sin(state(3)) / (cos(state(3)) + 1);
    df(1, 4) = 1 / cos(state(3));
    df(1, 5) = -EncoderControlModel::alpha * EncoderControlModel::W / 2;
    df(2, 3) = 2 * sin(state(3)) / (cos(state(3)) + 1);
    df(2, 4) = 1 / cos(state(3));
    df(2, 5) = EncoderControlModel::alpha * EncoderControlModel::W / 2;
    return df;
  } else {
    if (i >= NumConditionalArgumentsGet()) {
      std::cerr << "This pdf Only has " << NumConditionalArgumentsGet() << " conditional arguments\n";
      exit(- BFL_ERRMISUSE);
    } else {
      std::cerr << "The df is not implemented for the" << i << "th conditional argument\n";
      exit(- BFL_ERRMISUSE);
    }
  }
}
