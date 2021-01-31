/**
 * @author Markus Bader <markus.bader@mx-robotics.com>
 * @license Simplified BSD License
 */

#ifndef CAR_MATH_ANGLE_H
#define CAR_MATH_ANGLE_H

#include <Arduino.h>
#include <math.h>

namespace car
{
    namespace math
    {

        enum Direction{
            CLOCKWISE,
            COUNTERCLOCKWISE
        };
        
        /** class to manage discretized angles
         * @param MAX maximum angle discretization e.g. 360
         * @param RESOLUTION lookup table resolution to perform fast cosinus operations  
         **/
        template <short MAX, short RESOLUTION>
        class Angle
        {

            static float *cos_lockup;
            int16_t value_;

        public:
            /**
             * constructor
            **/
            Angle() : value_(0) {}
            /**
             * constructor
             * @param o 
            **/
            Angle(const Angle &o) : value_(o.value_) {}
            /**
             * constructor
             * @param val value will be normalized
            **/
            Angle(const int16_t &val) : value_(val)
            {
                normalize();
            }
            /**
             * max value for a full rotation
            **/
            int16_t max(){
                return MAX;
            }
            /**
             * Init lookup tables
             * it will alocated a lookup table to perform fast cosinus oprations
            **/
            static void init()
            {
                if (cos_lockup == NULL)
                {
                    cos_lockup = new float[MAX / RESOLUTION];
                    for (int16_t val = 0; val < MAX; val += RESOLUTION)
                    {
                        Angle::cos_lockup[val / RESOLUTION] = cos(val2rad(val));
                    }
                }
            }
            /**
             * @return discretized angle value
            **/
            const int16_t &operator()() const
            {
                return value_;
            }
            /**
             * @return discretized angle value
            **/
            int16_t &operator()()
            {
                return value_;
            }
            /**
             * normalizes the discretized angle between 0 and MAX-1
             * @return this
            **/
            Angle &normalize()
            {
                while (value_ < 0)
                    value_ += MAX;
                while (value_ >= MAX)
                    value_ -= MAX;
                return *this;
            }
            /**
             * set angle from radians
             * @param rad value to set in rad
             * @return this
            **/
            Angle &setRad(float rad)
            {
                value_ = rad2val(rad);
                return *this;
            }
            /**
             * set angle from degrees
             * @param deg value to set in rad
             * @return this
            **/
            Angle &setDeg(float deg)
            {
                value_ = deg2val(deg);
                return *this;
            }
            /**
             * set angle from discretized angle between 0 and MAX-1
             * @param val discretized angle between 0 and MAX-1
             * @return this
            **/
            Angle &set(float val)
            {
                value_ = val;
                return *this;
            }
            /**
             * returns cosinus
             * @pre init
             * @return cosinus
            **/
            float get_cos() const
            {
                return Angle::cos_lockup[value_ / RESOLUTION];
            }
            /**
             * angle in rad
             * @return angle in rad
            **/
            float get_rad() const
            {
                return val2rad(value_);
            }
            /**
             * computes angle in rad
             * @param val discretized angle between 0 and MAX-1
             * @return value between 0 and 1
            **/
            static float val2rad(int16_t val)
            {
                return M_TWOPI / MAX * (float)val;
            }
            /**
             * angle in as value between 0 and 1
             * @return value between 0 and 1
            **/
            float get_norm() const
            {
                return val2norm(value_);
            }
            /**
             * computes angle in as value between 0 and 1
             * @param val discretized angle between 0 and MAX-1
             * @return value between 0 and 1
            **/
            static float val2norm(int16_t val)
            {
                return 1.0 / MAX * (float)val;
            }
            /**
             * converts rad value to discretized angle value
             * @param rad rad
             * @return discretized angle between 0 and MAX-1
            **/
            static int16_t rad2val(float rad)
            {
                return round(rad * (float)MAX / M_TWOPI);
            }
            /**
             * converts degree value to discretized angle value
             * @param deg degree 0 - 359
             * @return discretized angle between 0 and MAX-1
            **/
            static int16_t deg2val(float deg)
            {
                return round(deg * (float)MAX / 360.);
            }
            /**
             * converts norm value to discretized angle value
             * @param rad norm value between 0 an 1
             * @return discretized angle between 0 and MAX-1
            **/
            static int16_t norm2val(float rad)
            {
                return round(rad * (float)MAX);
            }
            Angle &operator+=(int16_t rhs)
            {
                value_ += rhs;
                return normalize();
            }
            Angle &operator-=(int16_t rhs)
            {
                value_ -= rhs;
                return normalize();
            }

            Angle &operator+=(const Angle &rhs)
            {
                value_ += rhs.value_;
                return normalize();
            }
            Angle &operator-=(const Angle &rhs)
            {
                value_ -= rhs.value_;
                return normalize();
            }
            friend Angle operator+(Angle lhs, int16_t rhs)
            {
                lhs += rhs;
                return lhs;
            }
            friend Angle operator-(Angle lhs, int16_t rhs)
            {
                lhs -= rhs;
                return lhs;
            }
            friend Angle operator+(Angle lhs, const Angle &rhs)
            {
                lhs += rhs;
                return lhs;
            }
            friend Angle operator-(Angle lhs, const Angle &rhs)
            {
                lhs -= rhs;
                return lhs;
            }
        };
        typedef Angle<360, 2> AngleDeg;
        typedef Angle<0x3FFF, 0x3F> Angle14Bit;
    }; // namespace math
};     // namespace car
#endif //CAR_MATH_ANGLE_H
