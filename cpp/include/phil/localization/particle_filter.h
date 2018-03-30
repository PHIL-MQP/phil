#pragma  once

#include <memory>

#include <bfl/filter/bootstrapfilter.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/model/analyticsystemmodel_gaussianuncertainty.h>

#include <phil/localization/robot_model.h>
#include <phil/localization/point_mass_robot_model.h>
#include <phil/localization/filter.h>

namespace phil {
 typedef BFL::BootstrapFilter<MatrixWrapper::ColumnVector, MatrixWrapper::ColumnVector> PF;

class ParticleFilter : public Filter<PF> {
 public:
  static constexpr unsigned int NUM_SAMPLES = 2000;

  ParticleFilter();

//  std::unique_ptr<PF> filter;
  std::unique_ptr<BFL::MCPdf<BFL::ColumnVector> > prior_discrete;
  std::unique_ptr<BFL::AnalyticSystemModelGaussianUncertainty> system_model;
  std::unique_ptr<BFL::LinearAnalyticMeasurementModelGaussianUncertainty> yaw_measurement_model;
  std::unique_ptr<BFL::LinearAnalyticMeasurementModelGaussianUncertainty> acc_measurement_model;
  std::unique_ptr<BFL::LinearAnalyticMeasurementModelGaussianUncertainty> camera_measurement_model;
  std::unique_ptr<BFL::LinearAnalyticMeasurementModelGaussianUncertainty> beacon_measurement_model;
  std::unique_ptr<localization::EncoderControlModel> system_pdf;
  std::unique_ptr<BFL::LinearAnalyticConditionalGaussian> yaw_measurement_pdf;
  std::unique_ptr<BFL::LinearAnalyticConditionalGaussian> acc_measurement_pdf;
  std::unique_ptr<BFL::LinearAnalyticConditionalGaussian> camera_measurement_pdf;
  std::unique_ptr<BFL::LinearAnalyticConditionalGaussian> beacon_measurement_pdf;

  void ZeroVelocityUpdate() override;
};

}
