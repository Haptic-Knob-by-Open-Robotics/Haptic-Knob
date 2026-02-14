#pragma once

/*
  Kalman Filter Overview

  A Kalman filter is a lightweight estimator that combines:
    (1) a simple physics/motion model (what we *expect* will happen),
    (2) noisy sensor measurements (what we *observe*),
  to produce a best-guess estimate of the system state.

  Key idea:
    - The model predicts the next state (prediction step).
    - The measurement corrects that prediction (update step).
    - How much we trust each one is controlled by "noise" parameters.

  Two repeating steps each control tick:

  1) PREDICT
     Use the motion model to predict where the state should be after dt:

        x_pred = A x
        P_pred = A P A^T + Q

     Where:
       x = state vector (what we want to estimate)
       P = covariance (how uncertain we are about x)
       A = state transition (model)
       Q = process noise (model uncertainty)

  2) UPDATE (CORRECT)
     Compare prediction to the new sensor measurement and correct:

        y = z - H x_pred                 (innovation / residual)
        S = H P_pred H^T + R             (innovation covariance)
        K = P_pred H^T S^-1              (Kalman gain, trust weighting)

        x = x_pred + K y                 (corrected state)
        P = (I - K H) P_pred             (reduced uncertainty)

     Where:
       z = measurement
       H = measurement model (maps state -> measurement)
       R = measurement noise (sensor uncertainty)

  Intuition (tuning):
    - Larger R  => trust sensor less  => smoother, more laggy estimates
    - Larger Q  => trust model less   => responds faster, but noisier estimates
    - The filter automatically chooses K (Kalman gain) to balance these.

  =========================
  Why we use it in Haptic Knob
  =========================

  Our haptic models depend on clean motion terms:
    - Resistor: uses ω (velocity)
    - Inductor: uses α = dω/dt (acceleration)
    - RLC: uses θ, ω, α together

  Encoder gives θ_meas, but differentiating θ to get ω amplifies noise, and
  differentiating again for α amplifies even more.

  This module uses a 2-state Kalman filter to estimate:
      x = [ θ ; ω ]
  from only encoder angle measurements, producing a much cleaner ω (and α),
  which makes the haptic feedback feel smooth instead of “grainy”.

  =========================
  Model used here (constant velocity)
  =========================

    x_k+1 = A x_k + w
    z_k   = H x_k + v

    A = [ 1  dt ]
        [ 0  1  ]
    H = [ 1  0 ]     (we measure only θ)

  Units:
    θ in radians, ω in rad/s, dt in seconds
*/

class KalmanAngleVel {
public:
  struct Params {
    // Process noise (model uncertainty). Increase if you want faster response,
    // decrease for smoother but potentially laggier behavior.
    float q_theta = 1e-4f;
    float q_omega = 1e-2f;

    // Measurement noise (encoder angle noise). Increase to smooth more.
    float r_theta = 1e-3f;
  };

  // Initialize filter state. Call once at startup (or on reset).
  void init(float theta0_rad, float omega0_rad_s, const Params& p);

  // Update filter with the latest angle measurement and time step.
  // Call at fixed rate from ControlTask (e.g., dt=0.001 for 1kHz).
  void update(float theta_meas_rad, float dt_s);

  // Estimated state outputs
  float theta() const { return theta_rad_; }
  float omega() const { return omega_rad_s_; }

private:
  Params p_{};
  bool inited_ = false;

  // Estimated state
  float theta_rad_ = 0.0f;
  float omega_rad_s_ = 0.0f;

  // TODO (implementation):
  // Store and update the 2x2 covariance matrix P:
  //   P = [P00 P01
  //        P10 P11]
  // plus predict/update steps.
};
