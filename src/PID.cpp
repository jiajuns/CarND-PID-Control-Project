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
    param_index = 0;
    settle_steps = 100;
    eval_steps = 500;

    total_error = 0.0;
    // best_error = numeric_limits<double>::max();
    best_error = 99999.9;

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
            cout << "improved" << endl;
            best_error = total_error;
            dp[param_index] *= 1.1;
            param_index = (param_index + 1) % 3;
            tried_adding = false;
            tried_subtracting = false;
        }
        // when found a better solution
        if (!tried_adding && !tried_subtracting) {
            ModifyParameter(param_index, dp[param_index]);
            tried_adding = true;
        }
        // when added but solution isnt good
        else if (tried_adding && !tried_subtracting) {
            ModifyParameter(param_index, -2 * dp[param_index]);
            tried_subtracting = true;
        }
        // when after finished adding and substracting but solution isnt as good
        else {
            ModifyParameter(param_index, dp[param_index]);
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

void PID::ModifyParameter(int index, double amount) {
    switch (index)
    {
    case 0:
        Kp += amount;
        break;
    case 1:
        Kd += amount;
        break;
    case 2:
        Ki += amount;
        break;
    }
}
