/**
 * @author Markus Bader <markus.bader@mx-robotics.com>
 */

#ifndef CAR_MATH_ACKERMANN_H
#define CAR_MATH_ACKERMANN_H

#include <car/math/motion.h>

namespace car
{
    namespace math
    {

        /**
         * Ackermann Steering
         **/
        class AckermannModel
        {
        public:
            /**
             * constructor
             * @param wheel_displacement distance l between left and right wheel 
             * @param wheel_axle_displacement distance d between frond and back wheel axle
             * @param wheel_diameter wheel diameter
             **/
            AckermannModel(float wheel_displacement, float wheel_axle_displacement, float wheel_diameter)
                : l_(wheel_displacement), d_(wheel_axle_displacement), wheel_diameter_(wheel_diameter)
            {
            }

            /**
             * compute motion command
             * @param target
             * @return command 
             **/
            const Ackermann &update(const Twist &target)
            {
                return command_;
            }

            /**
             * last computed control variable u(t) 
             * @return last computed control variable u(t) 
             **/
            const Ackermann &out() const
            {
                return command_;
            }
        private:
            float l_;              /// distance l between left and right wheel
            float d_;              /// distance d between frond and back wheel axle
            float wheel_diameter_; /// wheel diameter
            Ackermann command_;
        };

    } // namespace math
} // namespace car
#endif // CAR_MATH_ACKERMANN_H
