#include <iostream>
#include <Eigen/Dense>
#include "../math/include/math/math.h"


int main() {
    // 1 a)
    // ==============================================================
    /*
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

    std::cout << twist(v_a, v_b).transpose() << std::endl;
    // ==============================================================


    // 1 c)
    // ==============================================================
    std::cout << screw_axis(v_a, v_b, 1).transpose() << std::endl;
    // ==============================================================

    // Task 1 d)
    // ==============================================================
    Eigen::Matrix4d T;

    T <<
        1, 2, 3, 4,
    5, 6, 7, 8,
    9, 10, 11, 12,
    13, 14, 15, 16;

    std::cout << adjoint_matrix(T) << std::endl;
    // ==============================================================

    // Task 1 e)
    // ==============================================================
    std::cout << cot(30*deg_to_rad_const) << std::endl;
    // ==============================================================
    */


    // Task 2 a)
    // ==============================================================
    Eigen::Vector3d ews{60, -60, 0};
    Eigen::Vector3d fw{-30, 0, 0};
    Eigen::Vector3d ms{0, 0, 2};

    math::wrench(fw, ms, ews);
    // ==============================================================


    // Task 2 b)
    // ==============================================================
    Eigen::VectorXd Fh(6);
    Eigen::VectorXd Fa(6);
    Eigen::Vector3d wh{0, 0, 0};
    Eigen::Vector3d vh{0, -5, 0};
    Eigen::Vector3d wa{0, 0, 0};
    Eigen::Vector3d va{0, 0, 1};
    Eigen::VectorXd Ff(6);
    Eigen::MatrixXd Thf(4, 4);
    Eigen::MatrixXd Taf(4, 4);

    Fh << math::twist(wh, vh);
    Fa << math::twist(wa, va);

    Thf <<
        1, 0, 0, -0.1,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1;

    Taf <<
        1, 0, 0, -0.25,
    0, 0, 1, 0,
    0, -1, 0, 0,
    0, 0, 0, 1;

    Ff = math::adjoint_matrix(Thf).transpose()*Fh + math::adjoint_matrix(Taf).transpose()*Fa;
    std::cout << " " << std::endl;
    std::cout << "Task 2b)" << std::endl;
    std::cout << "Ff" << std::endl;
    std::cout << Ff.transpose() << std::endl;
    // ==============================================================


    // Task 4b)
    // ==============================================================
    std::vector<Eigen::Vector3d> joint_positions
    {{0.0, 0.0, 0.0}, {90.0, 0.0, 0.0}, {0.0, 90.0, 0.0}, {0.0, 0.0, 90.0}, {10.0, -15.0, 2.75}};

    std::cout << " " << std::endl;
    std::cout << "Task 4b)" << std::endl;
    Eigen::Matrix4d test = math::planar_3r_fk_transform(joint_positions);
    // ==============================================================

    // Task 5a)
    // ==============================================================
    Eigen::VectorXd j1(6);
    Eigen::VectorXd j2(6);
    Eigen::VectorXd j3(6);

    j1 << 0, 0, 0, -90, 0, 0;
    j2 << 0, -180, 0, 0, 0, 0;
    j3 << 0, -90, 0, 0, 0, 0;

    const std::vector<Eigen::VectorXd> joint_positions_5{j1, j2, j3};

    std::cout << "Task 5a)" << std::endl;
    test = math::ur3e_fk_screw(joint_positions_5);
    // ==============================================================

    // Task 5b)
    // ==============================================================
    std::cout << "Task 5b)" << std::endl;
    test = math::ur3_fk_transform(j1*math::deg_to_rad_const);
    math::print_pose("transformation ur3", test);

    test = math::ur3_fk_transform(j2*math::deg_to_rad_const);
    math::print_pose("transformation ur3", test);

    test = math::ur3_fk_transform(j3*math::deg_to_rad_const);
    math::print_pose("transformation ur3", test);
    // ==============================================================
    return 0;
}