#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/analyticconditionalgaussian.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>

int main(int argc, char **argv) {

  MatrixWrapper::Matrix A(2, 2);
  // identity matrix
  A(1, 1) = 1.0;
  A(1, 2) = 0.0;
  A(2, 1) = 0.0;
  A(2, 2) = 1.0;

  MatrixWrapper::Matrix B(2, 2);
  // represents dynamics of a robot driving straight forward at an angle of 0.8 radians
  B(1, 1) = cos(0.8);
  B(1, 2) = 0;
  B(2, 1) = sin(0.8);
  B(2, 2) = 0;

  std::vector<MatrixWrapper::Matrix> AB(2);
  AB[0] = A;
  AB[1] = B;

  // The next two matrices define process noise (Q)
  MatrixWrapper::ColumnVector sysNoise_Mu(2);
  sysNoise_Mu(1) = 0.0;
  sysNoise_Mu(2) = 0.0;
  MatrixWrapper::SymmetricMatrix sysNoise_Cov(2);
  sysNoise_Cov = 0.0;
  sysNoise_Cov(1,1) = pow(0.01, 2);
  sysNoise_Cov(1,2) = 0.0;
  sysNoise_Cov(2,1) = 0.0;
  sysNoise_Cov(2,2) = pow(0.01, 2);

  return EXIT_SUCCESS;
}
