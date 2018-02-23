#include <phil/localization/robot_model.h>

constexpr double RobotStateModel::dt_s;

RobotStateModel::RobotStateModel(const BFL::Gaussian &additiveNoise)
    : AnalyticConditionalGaussianAdditiveNoise(additiveNoise, 2) {
}

MatrixWrapper::ColumnVector RobotStateModel::ExpectedValueGet() const {
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

  // handle wrapping of theta into -pi,pi
  if (state(3) > M_PI) {
    state(3) -= 2 * M_PI;
  } else if (state(3) < -M_PI) {
    state(3) += 2 * M_PI;
  }

  return state + AdditiveNoiseMuGet();
}

MatrixWrapper::Matrix RobotStateModel::dfGet(unsigned int i) const {
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

RioModel::RioModel(const BFL::Gaussian &additiveNoise)
    : AnalyticConditionalGaussianAdditiveNoise(additiveNoise, 1) {}

MatrixWrapper::ColumnVector RioModel::ExpectedValueGet() const {
  MatrixWrapper::ColumnVector measurement = ConditionalArgumentGet(0);
  MatrixWrapper::ColumnVector state = ConditionalArgumentGet(1);
  measurement = 0;
  measurement(1) = state(4) / cos(state(3)) + RobotStateModel::alpha * RobotStateModel::W * state(6) / 2.0; // v_l
  measurement(2) = state(4) / cos(state(3)) - RobotStateModel::alpha * RobotStateModel::W * state(6) / 2.0; // v_r
  measurement(3) = state(4); // theta
  return measurement + AdditiveNoiseMuGet();
}

MatrixWrapper::Matrix RioModel::dfGet(unsigned int i) const {
  if (i == 0)//derivative to the first conditional argument (x)
  {
    MatrixWrapper::ColumnVector state = ConditionalArgumentGet(0);
    MatrixWrapper::ColumnVector control = ConditionalArgumentGet(1);
    MatrixWrapper::Matrix df(3, RobotStateModel::N);
    df = 0;
    df(1, 3) = 2 * sin(state(3)) / (cos(state(3)) + 1);
    df(1, 4) = 1 / cos(state(3));
    df(1, 5) = -RobotStateModel::alpha * RobotStateModel::W / 2;
    df(2, 3) = 2 * sin(state(3)) / (cos(state(3)) + 1);
    df(2, 4) = 1 / cos(state(3));
    df(2, 5) = RobotStateModel::alpha * RobotStateModel::W / 2;
    df(3, 3) = 1;
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
