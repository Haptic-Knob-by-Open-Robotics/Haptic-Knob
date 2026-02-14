// control/KalmanAngleVel.cpp
#include "control/KalmanAngleVel.h"

void KalmanAngleVel::init(float theta0_rad, float omega0_rad_s, const Params& p) {
  p_ = p;
  theta_rad_ = theta0_rad;
  omega_rad_s_ = omega0_rad_s;
  inited_ = true;

  // TODO: Initialize 2x2 covariance matrix P (P00, P01, P10, P11)
  // Suggested idea: P00 small-ish, P11 larger (velocity starts more uncertain)
}

void KalmanAngleVel::update(float theta_meas_rad, float dt_s) {
  if (!inited_) {
    init(theta_meas_rad, 0.0f, p_);
    return;
  }

  // TODO: Predict step
  //  - x_pred = A x, where A = [[1, dt],[0,1]]
  //  - P_pred = A P A^T + Q, with Q from (q_theta, q_omega)

  // TODO: Update step
  //  - Innovation: y = z - H x_pred, where H = [1, 0]
  //  - S = H P_pred H^T + R, where R = r_theta
  //  - K = P_pred H^T * inv(S)
  //  - x = x_pred + K y
  //  - P = (I - K H) P_pred

  // TODO: Store results back into theta_rad_ and omega_rad_s_

  (void)theta_meas_rad;
  (void)dt_s;
}
