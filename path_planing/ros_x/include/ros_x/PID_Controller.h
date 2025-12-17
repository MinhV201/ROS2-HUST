#if !defined(PID_Controller)
#define PID_Controller

#include <cmath>
#include <algorithm>

class PIDController{
    public:
    PIDController(double kp, double ki, double kd, double min_out=-1.0, double max_out=1.0 )
        : kp_(kp), ki_(ki), kd_(kd), 
          min_output_(min_out), max_output_(max_out),
          prev_error_(0.0), integral_(0.0)
          {

          }
    ~PIDController() = default;

    double calculate(double setpoint, double measured, double dt){
        double err = setpoint - measured;
        
        integral_ += err * dt;
        integral_ = std::clamp(integral_, -10.0, 10.0);

        double derivative = 0.0;
        if (dt > 1e-6) { // Tránh chia cho 0
            derivative = (err - prev_error_) / dt;
        }

        double output = (kp_ * err) + (ki_ * integral_) + (kd_ * derivative);
        prev_error_ = err;

        return std::clamp(output, min_output_, max_output_);
    }

    void reset() {
        prev_error_ = 0.0;
        integral_ = 0.0;
    }

    void setGains(double kp, double ki, double kd) {
        kp_ = kp; ki_ = ki; kd_ = kd;
    }

    private:
    double kp_, ki_, kd_;
    double min_output_, max_output_;
    double prev_error_;
    double integral_;

};

#endif // PID_Controller
