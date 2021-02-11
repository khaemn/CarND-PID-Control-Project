#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_, double t_) {
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  t = t_;
}

void PID::UpdateError(double error) {
  p_error = Kp * error;
  d_error = Kd * (error - prev_error) / t;
  integral_error_sum += error * t;
  prev_error = error;
  i_error = Ki * integral_error_sum;
}

double PID::TotalError() {
  return -1*(p_error + i_error + d_error);
}
