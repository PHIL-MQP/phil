#pragma once

#include <bfl/pdf/analyticconditionalgaussian_additivenoise.h>

namespace phil {
namespace localization {

class PointMassControlModel : public BFL::AnalyticConditionalGaussianAdditiveNoise {
 public:
  explicit PointMassControlModel(const BFL::Gaussian &additiveNoise);

  // redefine virtual functions
  MatrixWrapper::ColumnVector ExpectedValueGet() const override;

  MatrixWrapper::Matrix dfGet(unsigned int i) const override;

};

}
}
