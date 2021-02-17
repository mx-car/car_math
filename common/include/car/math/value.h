/**
 * @author Markus Bader <markus.bader@mx-robotics.com>
 */

#ifndef CAR_MATH_VALUE_H
#define CAR_MATH_VALUE_H

namespace car
{
    namespace math
    {
        template <typename T>
        class Value
        {
        public:
            uint32_t stamp; /// microsec
            T value;
            Value()
            : stamp(0)
            , value() {}
            Value(const T &value)
            : stamp(0)
            , value(value) {}
            Value(const Value &m)
            : stamp(m.stamp)
            , value(m.value) {}
            Value(uint32_t stamp, const T &value)
            : stamp(stamp)
            , value(value) {}
            float stamp_as_sec(){
                return stamp / 1000. / 1000.;
            }
            T &operator()()
            {
                return value;
            }
            const T &operator()() const
            {
                return value;
            }
            Value &operator+=(const Value &rhs)
            {
                stamp += rhs.stamp;
                value += rhs.value;
                return *this;
            }
            Value &operator-=(const Value &rhs)
            {
                stamp -= rhs.stamp;
                value -= rhs.value;
                return *this;
            }
            friend Value operator+(Value lhs, const Value &rhs)
            {
                lhs += rhs;
                return lhs;
            }
            friend Value operator-(Value lhs, const Value &rhs)
            {
                lhs -= rhs;
                return lhs;
            }
        };
    }; // namespace math
};     // namespace car
#endif //CAR_MATH_VALUE_H
