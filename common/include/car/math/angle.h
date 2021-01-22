/**
 * @author Markus Bader <markus.bader@mx-robotics.com>
 * @license Simplified BSD License
 */

#ifndef CAR_MATH_ANGLE_H
#define CAR_MATH_ANGLE_H

#include <Arduino.h>

namespace car
{
    namespace math
    {
        class Angle
        {

        public:
            int16_t degree;
            Angle();
            Angle(const Angle &o);
            Angle(const int16_t &deg);
            const int16_t& operator()() const;

            static void init();
            void set_rad(float rad);
            void set_deg(float deg);
            float get_cos() const;
            float get_rad() const;
            static float deg2rad(int16_t deg);
            static int16_t rad2deg(float rad);
            Angle& operator+=(const Angle& rhs);
            Angle& operator-=(const Angle &rhs);
            friend Angle operator+(Angle lhs, const Angle& rhs){
                lhs += rhs;
                 return lhs;
            }
            friend Angle operator-(Angle lhs, const Angle& rhs){
                lhs -= rhs;
                 return lhs;
            }
        };
    }; // namespace math
};     // namespace car
#endif //CAR_MATH_ANGLE_H
