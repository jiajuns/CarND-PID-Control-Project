#include "PID.h"
#include <iostream>
#include <cmath>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    PID::Kp = Kp;
    PID::Ki = Ki;
    PID::Kd = Kd;

    p_error = 0;
    i_error = 0;
    d_error = 0;

    use_twiddle = false;
    dp = {1, 1, 1};
    step = 0;
    param_index = 2;
    settle_steps = 200;
    eval_steps = 2000;
    total_error = 0;
    best_error = std::numeric_limits<double>::max();
    tried_adding = false;
    tried_subtracting = false;
}

void PID::UpdateError(double cte) {
    if (step > settle_steps){
        total_error += pow(cte, 2);
    }

    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;

    // twiddle section
    if (use_twiddle && step == eval_steps - 1){
        cout << "total error: " << total_error << endl;
        cout << "best error: " << best_error << endl;
        if (total_error < best_error) {
            cout << "improvement!" << endl;
            best_error = total_error;
            if (step !=  eval_steps) {
                dp[param_index] *= 1.1;
            }
            param_index = (param_index + 1) % 3;
            tried_adding = false;
            tried_subtracting = false;
        }

        if (!tried_adding && !tried_subtracting) {
            AddToParameterAtIndex(param_index, dp[param_index]);
            tried_adding = true;
        }
        else if (tried_adding && !tried_subtracting) {
            AddToParameterAtIndex(param_index, -2 * dp[param_index]);
            tried_subtracting = true;
        }
        else {
            AddToParameterAtIndex(param_index, dp[param_index]);
            dp[param_index] *= 0.9;
            param_index = (param_index + 1) % 3;
            tried_adding = false;
            tried_subtracting = false;
        }
        cout << "P: " << Kp << endl;
        cout << "I: " << Ki << endl;
        cout << "D: " << Kd << endl;
        total_error = 0;
    }
    step++;
}

double PID::TotalError() {
    return 0.0;
}

bool PID::ReachMaxSteps(){
    if (eval_steps == step){
        step = 0;
        p_error = 0;
        i_error = 0;
        d_error = 0;
        return true;
    } else {
        return false;
    }
}

void PID::AddToParameterAtIndex(int index, double amount) {
    if (index == 0) {
        Kp += amount;
    }
    else if (index == 1) {
        Kd += amount;
    }
    else if (index == 2) {
        Ki += amount;
    }
}
