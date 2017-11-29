#include <iostream>
#include <vector>

#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */
  double Kp;
  double Ki;
  double Kd;

  std::vector<double> dp;
  int step, param_index;
  // number of steps to allow changes to settle, then to evaluate error
  int n_settle_steps, n_eval_steps;
  double total_error, best_error;
  bool tried_adding, tried_subtracting, use_twiddle;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  void AddToParameterAtIndex(int index, double amount);

  bool ReachMaxSteps();
};

#endif /* PID_H */
