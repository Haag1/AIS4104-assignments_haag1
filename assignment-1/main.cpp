#include <iostream>
#include <Eigen/Dense>
#include "../math/include/math/math.h"


// Task 2.1 b)
// ==============================================================================================
void skew_symmetric_test()
{
    Eigen::Matrix3d skew_matrix = math::skew_symmetric(Eigen::Vector3d{0.5, 0.5, 0.707107});
    std::cout << "Skew-symmetric matrix: " << std::endl;
    std::cout << skew_matrix << std::endl;
    std::cout << "Skew-symmetric matrix transposition: " << std::endl;
    std::cout << -skew_matrix.transpose() << std::endl;
}
// ==============================================================================================


// Task 2.2
// ==============================================================================================
void rotation_matrix_test()
{
    Eigen::Matrix3d rot =
    math::rotation_matrix_from_euler_zyx(Eigen::Vector3d{45.0, -45.0, 90.0});

    Eigen::Matrix3d rot_aa =
    math::rotation_matrix_from_axis_angle(Eigen::Vector3d{0.8164966, 0.0, 0.5773503}, 120.0);

    Eigen::Matrix3d rot_fa = math::rotation_matrix_from_frame_axes(Eigen::Vector3d{0.5, 0.5, 0.707107},
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
void transformation_matrix_test()
{
    Eigen::Matrix3d r = math::rotation_matrix_from_euler_zyx(Eigen::Vector3d{45, -45.0, 90.0});
    Eigen::Vector3d v{1.0, -2.0, 3.0};
    std::cout << "transformation_matrix: " << std::endl;
    std::cout << math::transformation_matrix(r, v) << std::endl;
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
    Eigen::Vector4d v_w_and_1 = math::transformation_matrix(math::rotation_matrix_from_euler_zyx(e), p) * v_a_and_1;

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
    // Task 2.1
    skew_symmetric_test();

    // Task 2.2
    rotation_matrix_test();

    // Task 2.3

    // b)
    transformation_matrix_test();

    //c)

    return 0;
}

