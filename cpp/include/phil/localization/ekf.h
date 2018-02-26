#pragma  once

#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <memory>

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
};

}
