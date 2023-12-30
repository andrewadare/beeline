#include <kalmanif/kalmanif.h>

#include <iostream>
#include <toml.hpp>  // https://github.com/marzer/tomlplusplus

#include "pose_graph.h"
#include "state_measurement_model.h"
#include "tag_data.h"

using namespace std;
using manif::SE3d;

using State = SE3d;
// using KF = kalmanif::ExtendedKalmanFilter<State>;
using KF = kalmanif::UnscentedKalmanFilterManifolds<State>;
using StateCovariance = kalmanif::Covariance<State>;
using SystemModel = kalmanif::LieSystemModel<State>;
using Control = SystemModel::Control;

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
  const int max_num_coasts = config["max_num_coasts"].value_or(1);
  const string_view infile = config["detections_file"].value_or(""sv);
  const double dt = config["dt"].value_or(0.03);
  const double sigma_velocity = config["sigma_velocity"].value_or(1);
  const double sigma_omega = config["sigma_omega"].value_or(1);

  // Measurement uncertainties
  const double dx = config["dx"].value_or(0.1);
  const double dy = config["dy"].value_or(0.1);
  const double dz = config["dz"].value_or(0.1);
  const double drx = config["drx"].value_or(0.1);
  const double dry = config["dry"].value_or(0.1);
  const double drz = config["drz"].value_or(0.1);

  const double initial_position_variance =
      config["initial_position_variance"].value_or(500);
  const double initial_rotation_variance =
      config["initial_rotation_variance"].value_or(500);

  ObsMap observations;
  IntMap poses_by_tag;
  read_tag_observations(infile.data(), &observations, &poses_by_tag);

  // Kalman filter and state covariance
  KF kf;
  StateCovariance state_cov_init = StateCovariance::Identity();
  state_cov_init.topLeftCorner<3, 3>() *= initial_position_variance;
  state_cov_init.bottomRightCorner<3, 3>() *= initial_rotation_variance;

  // Motion model f(X, u=0)
  Control u;         // Twist (u, v)
  Array6d u_sigmas;  // Twist uncertainties
  Matrix6d Q;        // Process noise cov. matrix
  u.setZero();
  u_sigmas.head<3>().setConstant(sigma_velocity);
  u_sigmas.tail<3>().setConstant(sigma_omega);
  Q = (u_sigmas * u_sigmas * 1. / dt).matrix().asDiagonal();
  SystemModel system_model(Q);

  // Observation model: h(X) = X
  Vector6d y;        // Measurement vector = log(X)
  Array6d y_sigmas;  // Measurement uncertainties
  Matrix6d R_tag;    // Measurement noise cov. matrix
  y_sigmas << dx, dy, dz, drx, dry, drz;
  R_tag = (y_sigmas * y_sigmas).matrix().asDiagonal();
  auto measurement_model = kalmanif::StateMeasurementModel<State>(R_tag);

  // Save PoseGraph objects for analysis
  PoseGraph g_meas, g_filt;

  bool filter_initialized = false;
  int num_coasts = 0;  // Consecutive frames with missing detection
  for (int i = first; i < last; ++i) {
    // Predict
    if (filter_initialized) {
      kf.propagate(system_model, u);
    }

    // Check for a measurement
    if (poses_by_tag[tag_to_track].count(i) == 1) {
      const auto tag = observations.at(i).at(tag_to_track);
      // cout << tag.frame_id << " " << tag.transform << endl;
      cout << tag.frame_id << " "
           << (tag.transform.translation() - kf.getState().translation())
                  .transpose()
           << endl;

      // Update or (re)initialize
      if (filter_initialized) {
        y = measurement_model(tag.transform);
        kf.update(measurement_model, y);
        g_filt.add({i, NodeType::POSE, kf.getState()});
      } else {
        kf.setState(tag.transform);
        kf.setCovariance(state_cov_init);
        filter_initialized = true;
      }

      g_meas.add({i, NodeType::POSE, tag.transform});

    } else {  // No measurement
      cout << i << endl;
      num_coasts++;
      if (num_coasts > max_num_coasts) {
        filter_initialized = false;
      }
    }
  }

  save_g2o(g_meas, "measured.g2o");
  save_g2o(g_filt, "filtered.g2o");

  return 0;
}
