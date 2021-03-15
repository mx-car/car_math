/**
 * @author Markus Bader <markus.bader@mx-robotics.com>
 */

#ifndef CAR_MATH_ODOMETRY_H
#define CAR_MATH_ODOMETRY_H

#include <array>

namespace car
{
    namespace math
    {
        struct Pose {
            std::array<float,3> v;
        };

        class Odometry
        {
        public:
            float wheel_diameter;
            float wheel_displacement;
            float wheel_axle_displacement;
        };
    }; // namespace math
};     // namespace car
#endif //CAR_MATH_ODOMETRY_H
