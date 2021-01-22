/**
 * @author Markus Bader <markus.bader@mx-robotics.com>
 * @license Simplified BSD License
 */

#include "car/math/angle.h"
#include <math.h>

using namespace car::math;

float cos_lockup[360];

Angle::Angle() : degree(0) {}
Angle::Angle(const Angle &o) : degree(o.degree) {}
Angle::Angle(const int16_t &deg) : degree(deg) {}

const int16_t& Angle::operator()() const{
    return degree;
}
void Angle::init()
{
    for (int16_t deg = 0; deg < 360; deg++)
    {
        cos_lockup[deg] = cos(Angle::deg2rad(deg));
    }
}
void Angle::set_rad(float rad)
{
    degree = rad2deg(rad);
}
void Angle::set_deg(float deg)
{
    degree = deg;
}
float Angle::get_cos() const
{
    return cos_lockup[degree];
}
float Angle::get_rad() const
{
    return deg2rad(degree);
}
float Angle::deg2rad(int16_t deg)
{
    return M_PI / 180.0 * (float) deg;
}
int16_t Angle::rad2deg(float rad)
{
    return round(rad * 180.0 / M_PI);
}

Angle &Angle::operator+=(const Angle &rhs)
{
    degree += rhs.degree;
    degree = degree % 360;
    return *this;
}
Angle &Angle::operator-=(const Angle &rhs)
{
    degree -= rhs.degree;
    degree = degree % 360;
    return *this;
}
