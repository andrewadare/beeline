#ifndef _STATE_MEASUREMENT_MODEL_H_
#define _STATE_MEASUREMENT_MODEL_H_

#include <kalmanif/kalmanif.h>

namespace kalmanif {

// Measurement model h(X) for an "identity" observation h(X) = X,
// where the estimated state is directly observed.
template <typename _State>
struct StateMeasurementModel
    : MeasurementModelBase<StateMeasurementModel<_State>>,
      Linearized<MeasurementModelBase<StateMeasurementModel<_State>>>,
      LinearizedInvariant<MeasurementModelBase<StateMeasurementModel<_State>>> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  using Base = MeasurementModelBase<StateMeasurementModel<_State>>;
  using Base::getCovariance;
  using Base::getCovarianceSquareRoot;
  using Base::setCovariance;
  using Base::operator();

  using State = _State;
  using Scalar = typename State::Scalar;
  // static constexpr auto Dim = State::Dim;
  // using Measurement = Eigen::Matrix<Scalar, Dim, 1>;
  using Measurement = Eigen::Matrix<Scalar, State::DoF, 1>;

  StateMeasurementModel(const Eigen::Ref<Covariance<Measurement>>& R) {
    setCovariance(R);
  }

  Measurement run(const State& x) const { return x.log().coeffs(); }

  // Returns Jacobians H and V as identities, since dh(X)/dX = dX/dX
  Measurement run_linearized(
      const State& x, Eigen::Ref<Jacobian<Measurement, State>> H,
      Eigen::Ref<Jacobian<Measurement, Measurement>> V) const {
    H.setIdentity();
    V.setIdentity();
    return x.log().coeffs();
  }

  Measurement run_linearized_invariant(
      const State& x, Eigen::Ref<Jacobian<Measurement, State>> H,
      Eigen::Ref<Jacobian<Measurement, Measurement>> V) const {
    H.setIdentity();
    V.setIdentity();
    return x.log().coeffs();
  }
};

namespace internal {

template <class StateType>
struct traits<StateMeasurementModel<StateType>> {
  using State = StateType;
  using Scalar = typename State::Scalar;
  using Measurement = Eigen::Matrix<Scalar, State::DoF, 1>;
  static constexpr Invariance invariance = Invariance::Left;
};

}  // namespace internal
}  // namespace kalmanif

#endif  // _STATE_MEASUREMENT_MODEL_H_
