#ifndef MATH_H
#define MATH_H


// From assignment 1
namespace math {
    // ============== From assignment 1 ==============
    const double deg_to_rad_const = 0.0174532925;
    const double rad_to_deg_const = 57.29578;

    // -------------- 2.1 --------------
    Eigen::Matrix3d skew_symmetric(Eigen::Vector3d v);

    // -------------- 2.2 --------------
    Eigen::Matrix3d rotation_matrix_from_frame_axes(const Eigen::Vector3d &x, const Eigen::Vector3d &y,
        const Eigen::Vector3d &z);

    Eigen::Matrix3d rotate_x(double degrees);
    Eigen::Matrix3d rotate_y(double degrees);
    Eigen::Matrix3d rotate_z(double degrees);

    Eigen::Matrix3d rotation_matrix_from_axis_angle(const Eigen::Vector3d &axis, double degrees);

    Eigen::Matrix3d rotation_matrix_from_euler_zyx(const Eigen::Vector3d &e);

    // -------------- 2.3 --------------
    Eigen::Matrix4d transformation_matrix(const Eigen::Matrix3d &r, const Eigen::Vector3d &p);
}

#endif //MATH_H
