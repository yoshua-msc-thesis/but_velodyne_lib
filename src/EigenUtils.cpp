#include <but_velodyne/EigenUtils.h>

#include <Eigen/Eigenvalues>

namespace but_velodyne {

void getEigenvalues(const Eigen::Matrix3f &covariance, std::vector<float> &eigenvalues) {
  Eigen::Vector3cf ev = covariance.eigenvalues();
  eigenvalues.resize(3);
  eigenvalues[0] = std::real(ev(0));
  eigenvalues[1] = std::real(ev(1));
  eigenvalues[2] = std::real(ev(2));
  sort(eigenvalues.begin(), eigenvalues.end());
}

}
