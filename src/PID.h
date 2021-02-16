#ifndef PID_H
#define PID_H

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_, double t_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double error);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

 private:
  /**
   * PID Errors
   */
  double p_error{0.};
  double i_error{0.};
  double d_error{0.};

  /**
   * PID Coefficients
   */ 
  double Kp{0.};
  double Ki{0.};
  double Kd{0.};

  // Discretization time, sec
  double t{0.02};

  double integral_error_sum{0.};
  double prev_error{0.};
};

#endif  // PID_H
