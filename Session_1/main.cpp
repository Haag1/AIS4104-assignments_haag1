#include <iostream>
#include <Eigen/Dense>

const double deg_to_rad_const = 0.0174532925;
const double rad_to_deg_const = 57.29578;

// helper functions
// =================================================================
bool floatEquals(double a, double b) {
    return std::abs(a - b) < 1e-6;
}

Eigen::Matrix3d rotate_x(double radians)
{
    Eigen::Matrix3d matrix;
    // implement the necessary equations and functionality.

    matrix <<
        1, 0, 0,
    0, std::cos(radians), -std::sin(radians),
    0, std::sin(radians), std::cos(radians);

    return matrix;
}

Eigen::Matrix3d rotate_y(double radians)
{
    Eigen::Matrix3d matrix;
    // implement the necessary equations and functionality.

    matrix <<
        std::cos(radians), 0, std::sin(radians),
    0, 1, 0,
    -std::sin(radians), 0, std::cos(radians);

    return matrix;
}

Eigen::Matrix3d rotate_z(double radians)
{
    Eigen::Matrix3d matrix;
    // implement the necessary equations and functionality.

    matrix <<
        std::cos(radians), -std::sin(radians), 0,
    std::sin(radians), std::cos(radians), 0,
    0, 0, 1;

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
// =================================================================


// Task 1 a)
// =================================================================
Eigen::Vector3d euler_zyx_from_rotation(const Eigen::Matrix3d &r) {

    double a;
    double b;
    double c;
    if(floatEquals(r(2, 0), -1.0)){
        b = EIGEN_PI / 2.0;
        a = 0;
        c = std::atan2(r(0, 1), r(1, 1));
    }
    else if(floatEquals(r(2, 0), 1.0)) {
        b = -EIGEN_PI / 2.0;
        a = 0.0;
        c = -std::atan2(r(0, 1), r(1, 1));
    }
    else{
        b = std::atan2(-r(2, 0), std::sqrt(r(0, 0)*r(0, 0) + r(1, 0) * r(1, 0)));
        a = std::atan2(r(1, 0), r(0, 0));
        c = std::atan2(r(2, 1), r(2, 2));
    }

    return Eigen::Vector3d{a, b, c};
}
// =================================================================

// Task 1 b)
// =================================================================
Eigen::VectorXd twist(const Eigen::Vector3d &w, const Eigen::Vector3d &v){
    Eigen::VectorXd twist(6);

    twist << w(0), w(1), w(2), v(0), v(1), v(2);

    return twist.transpose();

}
// =================================================================

int main() {
    // 1 a)
    // ==============================================================
    Eigen::Vector3d e = Eigen::Vector3d(60.0, 45.0, 30.0);
    Eigen::Matrix3d r = rotation_matrix_from_euler_zyx(e * deg_to_rad_const);
    Eigen::Vector3d ea = euler_zyx_from_rotation(r) * rad_to_deg_const;
    std::cout << e.transpose() << std::endl;
    std::cout << ea.transpose() << std::endl;
    // ==============================================================

    // 1 b)
    // ==============================================================
    Eigen::Vector3d v_a{1, 2, 3};
    Eigen::Vector3d v_b{4, 5, 6};
    v_a = v_a.transpose();
    v_b = v_b.transpose();

    std::cout << twist(v_a, v_b).transpose() << std::endl;
    // ==============================================================

    // 1 c)
    // ==============================================================

    // ==============================================================

    return 0;
}