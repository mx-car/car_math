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

AckermannState::AckermannState()
    : coubled({0, 0}), v({0, 0}), steering(0), mode(MODE_NA) {}
AckermannState::AckermannState(const AckermannState &o)
    : coubled(o.coubled), v(o.v), steering(o.steering), mode(o.mode) {}
AckermannState::AckermannState(const std::array<float, 3> &v, uint16_t mode, bool coubled)
    : coubled({coubled, coubled}), v({v[0], v[1]}), steering(v[2]), mode(mode) {}
AckermannState::AckermannState(const std::array<float, 2> &v, float steering, uint16_t mode, bool coubled)
    : coubled({coubled, coubled}), v(v), steering(steering), mode(mode) {}
AckermannState::AckermannState(float v_left, float v_right, float steering, uint16_t mode, bool coubled)
    : coubled({coubled, coubled}), v({v_left, v_right}), steering(steering), mode(mode) {}

void AckermannState::set(const Twist &twist, const AckermannConfig &config, bool coubled)
{
    set(twist, config);
    couble(coubled);
}
void AckermannState::couble(bool on)
{
    coubled[LEFT] = on;
    coubled[RIGHT] = on;
}

void AckermannState::set(std::array<float, 2> v, float steering, uint16_t mode, bool couble)
{
    this->couble(couble);
    this->v[LEFT] = v[LEFT], this->v[RIGHT] = v[RIGHT], this->steering = steering, this->mode = mode;
}

void AckermannState::set(const Twist &twist, const AckermannConfig &config)
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
    mode = MODE_VELOCITY;
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