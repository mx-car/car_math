/**
 * @author Markus Bader <markus.bader@mx-robotics.com>
 */

#ifndef CAR_MATH_PID_H
#define CAR_MATH_PID_H

#include <exception>

namespace car
{
    namespace math
    {
        /**
         * PID Controller
         **/
        class PID
        {
        public:
            /**
             * constructor
             * @param dt
             * @param min
             * @param max
             * @param Kp
             * @param Ki
             * @param Kd
             **/
            PID(float dt, float min, float max, float Kp, float Ki, float Kd)
                : dt_(dt), min_(min), max_(max), Kp_(Kp), Ki_(Ki), Kd_(Kd), integral_(0), error_(0)
            {
            }
            /**
             * update to compute a new command
             * @param target setpoint / SP / r(t)
             * @param measured measured process value (PV) / y(t)
             * @return control variable u(t) 
             **/
            float update(float target, float measured)
            {
                float error = target - measured;
                float Pout = Kp_ * error;
                integral_ += error * dt_;
                float Iout = Ki_ * integral_;
                float derivative = (error - error_) / dt_;
                float Dout = Kd_ * derivative;
                error_ = error;
                out_ = Pout + Iout + Dout;
                if (out_ > max_)
                    out_ = max_;
                if (out_ < min_)
                    out_ = min_;
                return out_;
            }

            /**
             * last computed control variable u(t) 
             * @return last computed control variable u(t) 
             **/
            float out() const{
                return out_;
            }
            /**
             * last computed error e(t) 
             * @return error e(t) 
             **/
            float error() const{
                return error_;
            }

        private:
            float dt_;
            float min_;
            float max_;
            float Kp_;
            float Ki_;
            float Kd_;
            float integral_;
            float error_;
            float out_;
        };

    } // namespace math
} // namespace car
#endif // CAR_BLDC_PID_H
