/**
 * @author Markus Bader <markus.bader@mx-robotics.com>
 * @license Simplified BSD License
 */

#ifndef CAR_MATH_MEASURMENT_H
#define CAR_MATH_MEASURMENT_H

#include <Arduino.h>

namespace car
{
    namespace math
    {
        template <typename T>
        class Measurement
        {
        public:
            uint32_t stamp; /// microsec
            T value;
            Measurement()
            : stamp(0)
            , value() {}
            Measurement(const Measurement &m)
            : stamp(m.stamp)
            , value(m.value) {}
            Measurement(uint32_t stamp, const T &value)
            : stamp(stamp)
            , value(value) {}
            float stamp_as_sec(){
                return stamp / 1000. / 1000.;
            }
            Measurement &operator+=(const Measurement &rhs)
            {
                stamp += rhs.stamp;
                value += rhs.value;
                return *this;
            }
            Measurement &operator-=(const Measurement &rhs)
            {
                stamp -= rhs.stamp;
                value -= rhs.value;
                return *this;
            }
            friend Measurement operator+(Measurement lhs, const Measurement &rhs)
            {
                lhs += rhs;
                return lhs;
            }
            friend Measurement operator-(Measurement lhs, const Measurement &rhs)
            {
                lhs -= rhs;
                return lhs;
            }
        };
    }; // namespace math
};     // namespace car
#endif //CAR_MATH_MEASURMENT_H
