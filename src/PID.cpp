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
    dp = {0.1*Kp, 0.1*Kd, 0.1*Ki};
    step = 0;
    param_index = 2;
    n_settle_steps = 200;
    n_eval_steps = 2000;
    total_error = 0;
    best_error = std::numeric_limits<double>::max();
    tried_adding = false;
    tried_subtracting = false;
}

void PID::UpdateError(double cte) {
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;

    if (step > n_settle_steps){
        total_error += pow(cte, 2);
    }

    if (use_twiddle && step == n_settle_steps + n_eval_steps - 1){
        cout << "total error: " << total_error << endl;
        cout << "best error: " << best_error << endl;
        if (total_error < best_error) {
            cout << "improvement!" << endl;
            best_error = total_error;
            if (step !=  n_settle_steps + n_eval_steps) {
                dp[param_index] *= 1.1;
            }
            // next parameter
            param_index = (param_index + 1) % 3;
            tried_adding = tried_subtracting = false;
        }

        if (!tried_adding && !tried_subtracting) {
            // try adding dp[i] to params[i]
            AddToParameterAtIndex(param_index, dp[param_index]);
            tried_adding = true;
        }
        else if (tried_adding && !tried_subtracting) {
            // try subtracting dp[i] from params[i]
            AddToParameterAtIndex(param_index, -2 * dp[param_index]);
            tried_subtracting = true;
        }
        else {
            // set it back, reduce dp[i], move on to next parameter
            AddToParameterAtIndex(param_index, dp[param_index]);
            dp[param_index] *= 0.9;
            // next parameter
            param_index = (param_index + 1) % 3;
            tried_adding = tried_subtracting = false;
        }
        total_error = 0;
        cout << "new parameters" << endl;
        cout << "P: " << Kp << ", I: " << Ki << ", D: " << Kd << endl;
    }
    step++;
}

double PID::TotalError() {
    return 0.0;
}

bool PID::ReachMaxSteps(){
    if (n_settle_steps + n_eval_steps == step){
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
    else {
        std::cout << "AddToParameterAtIndex: index out of bounds";
    }
}
