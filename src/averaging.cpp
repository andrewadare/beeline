#include "averaging.h"

#include <Eigen/Dense>
#include <algorithm>

using manif::SE3d;
using std::cout, std::endl;
using namespace Eigen;

SE3d mean(const std::vector<SE3d>& transforms) {
  Matrix3d R = Matrix3d::Zero();
  Vector3d t(0, 0, 0);
  for (const auto& transform : transforms) {
    R += transform.rotation();
    t += transform.translation();
  }
  t /= transforms.size();

  JacobiSVD<Matrix3d> svd(R, ComputeFullU | ComputeFullV);
  const Matrix3d U = svd.matrixU();
  const Matrix3d V = svd.matrixV();
  const DiagonalMatrix<double, 3> D(1, 1, (U * V.transpose()).determinant());

  R = U * D * V.transpose();

  return {t, Quaterniond(R)};
}

// Compute deviations as 3D position distances from mean.
// Find the inlier/outlier boundary based on a quantile choice (here, median).
// Select the inliers and re-average.
SE3d inlier_mean(const std::vector<SE3d>& transforms) {
  SE3d avg = mean(transforms);

  std::vector<double> devs;
  for (const auto& transform : transforms) {
    devs.push_back((transform.translation() - avg.translation()).norm());
  }
  std::sort(devs.begin(), devs.end());
  const double cutoff = devs[devs.size() / 2];

  std::vector<SE3d> inliers;
  for (const auto& transform : transforms) {
    if ((transform.translation() - avg.translation()).norm() <= cutoff) {
      inliers.push_back(transform);
    }
  }

  return mean(inliers);
}
