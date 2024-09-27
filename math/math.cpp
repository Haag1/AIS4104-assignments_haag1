#include <iostream>
#include <Eigen/Dense>
#include "include/math/math.h"

namespace math {
    // ============== From assignment 1 ==============
    // -------------- 2.1 --------------
    Eigen::Matrix3d skew_symmetric(Eigen::Vector3d v) {
        Eigen::Matrix3d skew_symetric_matrix;

        skew_symetric_matrix <<
            0, -v(2), v(1),
        v(2), 0, -v(0),
        -v(1), v(0), 0;

        return skew_symetric_matrix;
    }

    // -------------- Task 2.2 --------------
    Eigen::Matrix3d rotation_matrix_from_frame_axes(const Eigen::Vector3d &x, const Eigen::Vector3d &y,
    const Eigen::Vector3d &z)
    {
        Eigen::Matrix3d matrix;
        // implement the necessary equations and functionality.

        matrix <<
            x, y, z;

        return matrix;
    }

    Eigen::Matrix3d rotate_x(double degrees)
    {
        Eigen::Matrix3d matrix;
        const double radians = degrees* math::deg_to_rad_const;
        // implement the necessary equations and functionality.

        matrix <<
            1, 0, 0,
        0, std::cos(radians), -std::sin(radians),
        0, std::sin(radians), std::cos(radians);

        return matrix;
    }
    Eigen::Matrix3d rotate_y(double degrees)
    {
        Eigen::Matrix3d matrix;
        const double radians = degrees*math::deg_to_rad_const;
        // implement the necessary equations and functionality.

        matrix <<
            std::cos(radians), 0, std::sin(radians),
        0, 1, 0,
        -std::sin(radians), 0, std::cos(radians);

        return matrix;
    }
    Eigen::Matrix3d rotate_z(double degrees)
    {
        Eigen::Matrix3d matrix;
        const double radians = degrees*math::deg_to_rad_const;
        // implement the necessary equations and functionality.

        matrix <<
            std::cos(radians), -std::sin(radians), 0,
        std::sin(radians), std::cos(radians), 0,
        0, 0, 1;

        return matrix;
    }

    Eigen::Matrix3d rotation_matrix_from_axis_angle(const Eigen::Vector3d &axis, double degrees)
    {
        Eigen::Matrix3d matrix;
        const double radians = degrees*math::deg_to_rad_const;
        const double c_0 = std::cos(radians);
        const double s_0 = std::sin(radians);
        const double w_1 = axis(0);
        const double w_2 = axis(1);
        const double w_3 = axis(2);
        // implement the necessary equations and functionality.

        matrix <<
            c_0 + w_1*w_1*(1 - c_0), w_1*w_2*(1 - c_0) - w_3*s_0, w_1*w_3*(1 - c_0) + w_2*s_0,
        w_1*w_2*(1 - c_0) + w_3*s_0, c_0 + w_2*w_2*(1 - c_0), w_2*w_3*(1 - c_0) - w_1*s_0,
        w_1*w_3*(1 - c_0) - w_2*s_0, w_2*w_3*(1 - c_0) + w_1*s_0, c_0 + w_3*w_3*(1 - c_0);


        return matrix;
    }

    Eigen::Matrix3d rotation_matrix_from_euler_zyx(const Eigen::Vector3d &e)
    {
        Eigen::Matrix3d R_euler;
        Eigen::Matrix3d I;

        I <<
            1, 0, 0,
        0, 1, 0,
        0, 0, 1;

        R_euler = I * rotate_z(e(0)) * rotate_y(e(1)) * rotate_x(e(2));

        return R_euler;
    }

    // -------------- Task 2.3 --------------
    Eigen::Matrix4d transformation_matrix(const Eigen::Matrix3d &r, const Eigen::Vector3d &p)
    {
        Eigen::Matrix4d matrix;
        // implement the necessary equations and functionality.

        matrix <<
            r(0,0), r(0,1), r(0,2), p(0),
        r(1,0), r(1,1), r(1,2), p(1),
        r(2,0), r(2,1), r(2,2), p(2),
        0, 0, 0, 1;

        return matrix;
    }
}