#include <phil/localization/robot_model.h>
#include <phil/localization/point_mass_robot_model.h>

namespace phil {
namespace localization {

PointMassControlModel::PointMassControlModel(const BFL::Gaussian &additiveNoise)
    : AnalyticConditionalGaussianAdditiveNoise(additiveNoise, 2) {
}

MatrixWrapper::ColumnVector PointMassControlModel::ExpectedValueGet() const {
  MatrixWrapper::ColumnVector state = ConditionalArgumentGet(0);
  state(1) = state(1) + state(4) * dt_s + 0.5 * state(7) * dt_s * dt_s;
  state(2) = state(2) + state(5) * dt_s;
  state(3) = state(3) + state(6) * dt_s;

  state(4) = state(4) + state(7) * dt_s;
  state(5) = state(5) + state(8) * dt_s;
  state(6) = 0;
  state(7) = state(7);
  state(8) = state(8);
  state(9) = 0;

  return state + AdditiveNoiseMuGet();
}

MatrixWrapper::Matrix PointMassControlModel::dfGet(unsigned int i) const {
  if (i == 0)//derivative to the first conditional argument (x)
  {
    MatrixWrapper::ColumnVector state = ConditionalArgumentGet(0);
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
    df(4, 3) = 0;
    df(5, 3) = 0;
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

}
}
