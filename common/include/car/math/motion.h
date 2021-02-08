/**
 * @author Markus Bader <markus.bader@mx-robotics.com>
 */

#ifndef CAR_MATH_MOTION_H
#define CAR_MATH_MOTION_H

#include <array>

namespace car
{
    namespace math
    {
        class Twist
        {
        public:
            Twist();
            Twist(const Twist &o);
            Twist(float v, float w);
            float v;
            float w;
            /**
             * returns the current radius or infinit if w == 0
             * @return v/w
             **/
            float radius() const;
            /**
             * returns the current 1/radius 
             * @return v*w 
             **/
            float radius_inv() const;
        };

        class AckermannConfig
        {
        public:
            AckermannConfig();
            AckermannConfig(const AckermannConfig &o);
            AckermannConfig(float wheel_diameter, float wheel_displacement, float wheel_axle_displacement);
            float wheel_diameter;          /// wheel diameter
            float wheel_displacement;      /// distance l between left and right wheel
            float wheel_axle_displacement; /// distance d between frond and back wheel axle
        };

        class AckermannState
        {
        public:
            static const int LEFT = 0;
            static const int RIGHT = 1;
            AckermannState();
            AckermannState(const AckermannState &o);
            AckermannState(const std::array<float, 3> &v);
            AckermannState(const std::array<float, 2> &v, float streering);
            AckermannState(float v_left, float v_right, float streering);
            void set(const Twist &twist, const AckermannConfig &config);
            std::array<float, 2> v;
            float steering;
        };


    } // namespace math
} // namespace car
#endif // CAR_MATH_MOTION_H
