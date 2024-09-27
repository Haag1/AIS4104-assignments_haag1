#include <iostream>
#include <Eigen/Dense>

#include "../math/include/math/math.h"

/*
Eigen::Matrix3d rotate_x(double degrees)
{
    Eigen::Matrix3d matrix;
    double radians = deg_to_rad(degrees);

    matrix <<
        1.0, 0.0, 0.0,
    0.0, std::cos(radians), -std::sin(radians),
    0.0, std::sin(radians), std::cos(radians);

    return matrix;
}

void example(double constant)
{
    Eigen::Matrix3d identity;
    identity <<
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0;
    std::cout << "I: " << std::endl << identity << std::endl << std::endl;
    std::cout << constant <<"*I: " << std::endl << constant * identity << std::endl << std::endl;
}
*/


// Task 2.1
// ==============================================================================================
Eigen::Matrix3d skew_symmetric(Eigen::Vector3d v) {
    Eigen::Matrix3d skew_symetric_matrix;

    skew_symetric_matrix <<
        0, -v(2), v(1),
    v(2), 0, -v(0),
    -v(1), v(0), 0;

    return skew_symetric_matrix;

}

void skew_symmetric_test()
{
    Eigen::Matrix3d skew_matrix = skew_symmetric(Eigen::Vector3d{0.5, 0.5, 0.707107});
    std::cout << "Skew-symmetric matrix: " << std::endl;
    std::cout << skew_matrix << std::endl;
    std::cout << "Skew-symmetric matrix transposition: " << std::endl;
    std::cout << -skew_matrix.transpose() << std::endl;
}
// ==============================================================================================


// Task 2.2
// ==============================================================================================
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

void rotation_matrix_test()
{
    Eigen::Matrix3d rot =
    rotation_matrix_from_euler_zyx(Eigen::Vector3d{45.0, -45.0, 90.0});

    Eigen::Matrix3d rot_aa =
    rotation_matrix_from_axis_angle(Eigen::Vector3d{0.8164966, 0.0, 0.5773503}, 120.0);

    Eigen::Matrix3d rot_fa = rotation_matrix_from_frame_axes(Eigen::Vector3d{0.5, 0.5, 0.707107},
                                                            Eigen::Vector3d{-0.5, -0.5, 0.707107},
                                                            Eigen::Vector3d{0.707107, -0.707107, 0.0});

    std::cout << "Rotation matrix from Euler: " << std::endl;
    std::cout << rot << std::endl << std::endl;
    std::cout << "Rotation matrix from axis-angle pair: " << std::endl;
    std::cout << rot_aa << std::endl << std::endl;
    std::cout << "Rotation matrix from frame axes: " << std::endl;
    std::cout << rot_fa << std::endl << std::endl;
}
// ==============================================================================================


// Task 2.3
// ==============================================================================================
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

void transformation_matrix_test()
{
    Eigen::Matrix3d r = rotation_matrix_from_euler_zyx(Eigen::Vector3d{45, -45.0, 90.0});
    Eigen::Vector3d v{1.0, -2.0, 3.0};
    std::cout << "transformation_matrix: " << std::endl;
    std::cout << transformation_matrix(r, v) << std::endl;
}

void transform_vector() {
    Eigen::Vector3d e;
    Eigen::Vector3d v_a;
    Eigen::Vector3d p;
    Eigen::Vector3d v_w;
    Eigen::Vector4d v_a_and_1{0,0,0,1};


    // from task
    // ------------
    e <<
        60, 45, 0;

    e = e.transpose();

    v_a <<
        2.5,
    3.0,
    -10;
    // ------------

    // 1. Vector defining frame {a}'s origin, with a vector in frame {w}
    p <<
        0,
    0,
    10;

    // 2. Puts vector in to a 4 dimentional setting
    v_a_and_1 <<
        v_a(0),
    v_a(1),
    v_a(2),
    1;

    // 3. Transforms the vector in {a} frame to be expressed by {w} frame
    Eigen::Vector4d v_w_and_1 = transformation_matrix(rotation_matrix_from_euler_zyx(e), p) * v_a_and_1;

    // 4. Extracts the 3 dimentional vector from the 4 dimentional calculation
    v_w <<
        v_w_and_1(0),
    v_w(1),
    v_w(2);

    // 5. Prints the answer
    std::cout << "v_w: \n" << v_w_and_1 << std::endl;

}
// ==============================================================================================



int main()
{
    // example(2.0);
    /*
    Eigen::Matrix3d x_0 = rotate_x(0.0);
    std::cout << x_0 << std::endl;

    Eigen::Matrix3d x_45 = rotate_x(45);
    std::cout << x_45 << std::endl;
    */

    // Task 2.1
    skew_symmetric_test();

    // Task 2.2
    rotation_matrix_test();

    // Task 2.3

    // b)
    transformation_matrix_test();

    //c)
    math::print_hello();


    return 0;
}

