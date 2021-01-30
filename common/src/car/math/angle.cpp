/**
 * @author Markus Bader <markus.bader@mx-robotics.com>
 * @license Simplified BSD License
 */

#include <Arduino.h>
#include <car/math/angle.h>

template<> float* car::math::AngleDeg::cos_lockup = NULL;
template<> float* car::math::Angle14Bit::cos_lockup = NULL;