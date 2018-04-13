#pragma  once

#include <memory>

#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>

#include <phil/localization/robot_model.h>
#include <phil/localization/ekf.h>
#include <phil/localization/filter.h>
#include <phil/localization/point_mass_robot_model.h>

namespace phil {

class EKF : public Filter<BFL::ExtendedKalmanFilter> {
 public:
  EKF(double W, double alpha, double dt_s);

  std::unique_ptr<BFL::AnalyticSystemModelGaussianUncertainty> system_model;
  std::unique_ptr<BFL::LinearAnalyticMeasurementModelGaussianUncertainty> yaw_measurement_model;
  std::unique_ptr<BFL::LinearAnalyticMeasurementModelGaussianUncertainty> acc_measurement_model;
  std::unique_ptr<BFL::LinearAnalyticMeasurementModelGaussianUncertainty> camera_measurement_model;
  std::unique_ptr<BFL::LinearAnalyticMeasurementModelGaussianUncertainty> beacon_measurement_model;
  std::unique_ptr<localization::EncoderControlModel> system_pdf;
  std::unique_ptr<BFL::Gaussian> prior;
  std::unique_ptr<BFL::LinearAnalyticConditionalGaussian> yaw_measurement_pdf;
  std::unique_ptr<BFL::LinearAnalyticConditionalGaussian> acc_measurement_pdf;
  std::unique_ptr<BFL::LinearAnalyticConditionalGaussian> camera_measurement_pdf;
  std::unique_ptr<BFL::LinearAnalyticConditionalGaussian> beacon_measurement_pdf;

  void ZeroVelocityUpdate() override;
};

}
