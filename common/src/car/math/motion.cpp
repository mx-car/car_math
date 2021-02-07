/**
 * @author Markus Bader <markus.bader@mx-robotics.com>
 */

#include <Arduino.h>
#include <car/math/motion.h>
#include <limits>

using namespace car::math;

AckermannConfig::AckermannConfig()
    : wheel_diameter(0), wheel_displacement(0), wheel_axle_displacement(0) {}
AckermannConfig::AckermannConfig(const AckermannConfig &o)
    : wheel_diameter(o.wheel_diameter), wheel_displacement(o.wheel_displacement), wheel_axle_displacement(o.wheel_axle_displacement) {}
AckermannConfig::AckermannConfig(float wheel_diameter, float wheel_displacement, float wheel_axle_displacement) 
: wheel_diameter(wheel_diameter), wheel_displacement(wheel_displacement), wheel_axle_displacement(wheel_axle_displacement) {}

AckermannRearWheelDrive::AckermannRearWheelDrive()
    : v({0, 0}), steering(0) {}
AckermannRearWheelDrive::AckermannRearWheelDrive(const AckermannRearWheelDrive &o)
    : v(o.v), steering(o.steering) {}
AckermannRearWheelDrive::AckermannRearWheelDrive(const std::array<float, 3> &v)
    : v({v[0], v[1]}), steering(v[2]) {}
AckermannRearWheelDrive::AckermannRearWheelDrive(const std::array<float, 2> &v, float steering)
    : v(v), steering(steering) {}
AckermannRearWheelDrive::AckermannRearWheelDrive(float v_left, float v_right, float steering)
    : v({v_left, v_right}), steering(steering) {}

void AckermannRearWheelDrive::set(const Twist &twist, const AckermannConfig &config)
{
    if (twist.w == 0)
    {
        v[LEFT] = twist.v;
        v[RIGHT] = twist.v;
        steering = 0;
    }
    else
    {
        float R = twist.radius();
        v[LEFT] = twist.w * (R - config.wheel_displacement / 2.);
        v[RIGHT] = twist.w * (R + config.wheel_displacement / 2.);
        steering = atan2(config.wheel_axle_displacement, R);
    }
}
Twist::Twist() : v(0), w(0) {}
Twist::Twist(const Twist &o) : v(o.v), w(o.w) {}
Twist::Twist(float v, float w) : v(v), w(w) {}

float Twist::radius() const 
{
    if (w == 0)
        return std::numeric_limits<float>::infinity();
    return v / w;
}
float Twist::radius_inv() const
{
    return v * w;
}