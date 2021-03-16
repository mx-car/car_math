/**
 * @author Markus Bader <markus.bader@mx-robotics.com>
 */

#ifndef CAR_MATH_MOTION_H
#define CAR_MATH_MOTION_H

#include <array>
#include <car/math/value.h>

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
        typedef Value<Twist> TwistStamped;

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
        typedef Value<AckermannConfig> AckermannConfigStamped;

        class AckermannState
        {
        public:
            static const int LEFT = 0;
            static const int RIGHT = 1;
            static const int MODE_NA = 0;
            static const int MODE_PWM = 1;
            static const int MODE_VELOCITY = 2;
            AckermannState();
            AckermannState(const AckermannState &o);
            AckermannState(const std::array<float, 3> &v, uint16_t mode, bool coubled = true);
            AckermannState(const std::array<float, 2> &v, float streering, uint16_t mode, bool coubled = true);
            AckermannState(float v_left, float v_right, float streering, uint16_t mode, bool coubled = true);

            void set(std::array<float, 2> v, float steering, uint16_t mode, bool coubled);
            void set(const Twist &twist, const AckermannConfig &config, bool coubled);
            void set(const Twist &twist, const AckermannConfig &config);
            void couble(bool on);
            std::array<bool, 2> coubled;
            std::array<float, 2> v;
            float steering;
            uint16_t mode;
        };
        typedef Value<AckermannState> AckermannStateStamped;

    } // namespace math
} // namespace car
#endif // CAR_MATH_MOTION_H
