#pragma  once

#include <memory>

#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>

#include <phil/localization/robot_model.h>

namespace phil {

class EKF {
 public:
  EKF();

  std::unique_ptr<BFL::ExtendedKalmanFilter> filter;
  std::unique_ptr<BFL::AnalyticSystemModelGaussianUncertainty> encoder_system_model;
  std::unique_ptr<BFL::LinearAnalyticMeasurementModelGaussianUncertainty> yaw_measurement_model;
  std::unique_ptr<BFL::LinearAnalyticMeasurementModelGaussianUncertainty> acc_measurement_model;
  std::unique_ptr<BFL::LinearAnalyticMeasurementModelGaussianUncertainty> camera_measurement_model;
  std::unique_ptr<BFL::LinearAnalyticMeasurementModelGaussianUncertainty> beacon_measurement_model;
  std::unique_ptr<EncoderControlModel> encoder_system_pdf;
  std::unique_ptr<BFL::Gaussian> prior;
  std::unique_ptr<BFL::LinearAnalyticConditionalGaussian> yaw_measurement_pdf;
  std::unique_ptr<BFL::LinearAnalyticConditionalGaussian> acc_measurement_pdf;
  std::unique_ptr<BFL::LinearAnalyticConditionalGaussian> camera_measurement_pdf;
  std::unique_ptr<BFL::LinearAnalyticConditionalGaussian> beacon_measurement_pdf;
};

}
