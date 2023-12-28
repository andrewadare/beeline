#include <kalmanif/kalmanif.h>

#include <iostream>
#include <toml.hpp>  // https://github.com/marzer/tomlplusplus

#include "state_measurement_model.h"
#include "tag_data.h"

using namespace std;
using manif::SE3d;

using State = SE3d;
using EKF = kalmanif::ExtendedKalmanFilter<State>;
using StateCovariance = kalmanif::Covariance<State>;
using SystemModel = kalmanif::LieSystemModel<State>;
using Control = SystemModel::Control;
// using MeasurementModel = kalmanif::Landmark3DMeasurementModel<State>;
// using Landmark = MeasurementModel::Landmark;
// using MeasurementModel = kalmanif::StateMeasurementModel<State>;
// using Measurement = MeasurementModel::Measurement;

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Array6d = Eigen::Array<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

int main(int argc, const char** argv) {
  if (argc != 2) {
    cout << "Usage: " << argv[0] << "your-config.toml" << endl;
    return 1;
  }

  const auto config = toml::parse_file(argv[1]);
  const int tag_to_track = config["tag_to_track"].value_or(0);
  const int first = config["first_frame"].value_or(0);
  const int last = config["last_frame"].value_or(0);
  const string_view infile = config["detections_file"].value_or(""sv);

  ObsMap observations;
  IntMap poses_by_tag;
  read_tag_observations(infile.data(), &observations, &poses_by_tag);

  // EKF and initial filter state covariance
  EKF ekf;
  StateCovariance state_cov_init = StateCovariance::Identity();
  state_cov_init.topLeftCorner<3, 3>() *= 100;
  state_cov_init.bottomRightCorner<3, 3>() *= MANIF_PI_4;
  ekf.setCovariance(state_cov_init);

  // Motion model f(X, u=0)
  constexpr double dt = 1. / 30;          // Video sample period [s]
  constexpr double sigma_velocity = 1e3;  // Lin. vel. uncertainty [m/s]
  constexpr double sigma_omega = 1e3;     // Ang. vel. uncertainty [rad/s]
  Control u;                              // Twist (u, v)
  Array6d u_sigmas;                       // Twist uncertainties
  Matrix6d Q;                             // Process noise cov. matrix
  u.setZero();
  u_sigmas.head<3>().setConstant(sigma_velocity);
  u_sigmas.tail<3>().setConstant(sigma_omega);
  Q = (u_sigmas * u_sigmas * 1. / dt).matrix().asDiagonal();
  SystemModel system_model(Q);

  // Observation model: h(X) = X
  Vector6d y;        // Measurement vector = log(X).coeffs()
  Array6d y_sigmas;  // Measurement uncertainties
  Matrix6d R_tag;    // Measurement noise cov. matrix
  y_sigmas << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;
  R_tag = (y_sigmas * y_sigmas).matrix().asDiagonal();
  auto measurement_model = kalmanif::StateMeasurementModel<State>(R_tag);

  bool filter_initialized = false;
  for (int i = first; i < last; ++i) {
    // Predict
    if (filter_initialized) {
      ekf.propagate(system_model, u);
    }

    if (poses_by_tag[tag_to_track].count(i) == 1) {
      const auto tag = observations.at(i).at(tag_to_track);
      // cout << tag.frame_id << " " << tag.transform << endl;
      cout << tag.frame_id << " "
           << (tag.transform.translation() - ekf.getState().translation())
                  .transpose()
           << endl;

      if (!filter_initialized) {
        ekf.setState(tag.transform);
        filter_initialized = true;
      }

      // Update
      y = measurement_model(tag.transform);
      ekf.update(measurement_model, y);

    } else {
      cout << i << endl;
    }
  }

  return 0;
}
