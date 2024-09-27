#include <iostream>
#include <Eigen/Dense>
#include "../math/include/math/math.h"

// Task 4 a)
// ==============================================================

// ==============================================================

// Task 4 b)
// ==============================================================
Eigen::Matrix4d planar_3r_fk_transform(const std::vector<Eigen::Vector3d> &joint_positions) {
    const unsigned long positionVectorLength = joint_positions.size();

    Eigen::Vector3d p{10, 0, 0};

    Eigen::Matrix4d T04;

    for(int i = 0; i < positionVectorLength; i++) {
        Eigen::Matrix3d R1 = rotate_z(joint_positions[i][0]*deg_to_rad_const);
        Eigen::Matrix3d R2 = rotate_z(joint_positions[i][1]*deg_to_rad_const);
        Eigen::Matrix3d R3 = rotate_z(joint_positions[i][2]*deg_to_rad_const);
        Eigen::Matrix4d T01 = transformation_matrix(R1, {0, 0, 0});
        Eigen::Matrix4d T12 = transformation_matrix(R2, p);
        Eigen::Matrix4d T23 = transformation_matrix(R3, p);
        Eigen::Matrix4d T34 = transformation_matrix(Eigen::Matrix3d::Identity(), p);

        Eigen::Matrix4d T04_temp = T01*T12*T23*T34;

        std::cout << "j" << i + 1 << ":" << std::endl;
        print_pose("Angle and pos", T04_temp);
    }
    return T04;
}
// ==============================================================

// Task 4c)
// ==============================================================
Eigen::Matrix4d planar_3r_fk_screw(const std::vector<Eigen::Vector3d> &joint_positions) {
    const unsigned long positionVectorLength = joint_positions.size();
    const int L = 10;
    Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
    M(0, 3) = 3*L;

    for(int i = 0; i < positionVectorLength; i++) {
        Eigen::Vector3d w1{0, 0, 1};
        Eigen::Vector3d w2{0, 0, 1};
        Eigen::Vector3d w3{0, 0, 1};

        Eigen::Vector3d v1{0, 0, 0};
        Eigen::Vector3d v2{0, -L, 0};
        Eigen::Vector3d v3{0, -2*L, 0};


        Eigen::Matrix4d e1 = matrix_exponential(w1, v1, joint_positions[i][0]*deg_to_rad_const);
        Eigen::Matrix4d e2 = matrix_exponential(w2, v2, joint_positions[i][1]*deg_to_rad_const);
        Eigen::Matrix4d e3 = matrix_exponential(w3, v3, joint_positions[i][2]*deg_to_rad_const);

        Eigen::Matrix4d f4_temp = e1*e2*e3*M;

        std::cout << "Hello" << std::endl;
        std::cout << e1 << std::endl;
        std::cout << e2 << std::endl;
        std::cout << e3 << std::endl;


        print_pose("Angle and pos", f4_temp);
    }
    return M;
}
// ==============================================================


// Task 5a)
// ==============================================================
Eigen::Matrix4d ur3e_fk_screw(const std::vector<Eigen::VectorXd> &joint_positions) {
    double L1 = 0.2435;
    double L2 = 0.2132;
    double W1 = 0.1315;
    double W2 = 0.0921;
    double H1 = 0.1518;
    double H2 = -0.08535;

    std::vector<Eigen::Vector3d> all_omega{
        {0, 0, 1}, {0, 1, 0}, {0, 1, 0}, {0, 1, 0}, {0, 0, -1}, {0, 1, 0}};

    std::vector<Eigen::Vector3d> all_v{
        {0, 0, 0}, {-H1, 0, 0}, {-H1, 0, L1}, {-H1, 0, L1+L2}, {-W1, L1 + L2, 0}, {H2-H1, 0, L1+L2}};

    Eigen::Matrix4d M;

    M << -1, 0, 0, L1+L2,
    0, 0, 1, W1+W2,
    0, 1, 0, H1+H2,
    0, 0, 0, 1;

    for(int i = 0; i < joint_positions.size(); i++) {
        Eigen::Matrix4d T1 = matrix_exponential(all_omega[0], all_v[0], joint_positions[i](0, 0)*deg_to_rad_const);
        Eigen::Matrix4d T2 = matrix_exponential(all_omega[1], all_v[1], joint_positions[i](1, 0)*deg_to_rad_const);
        Eigen::Matrix4d T3 = matrix_exponential(all_omega[2], all_v[2], joint_positions[i](2, 0)*deg_to_rad_const);
        Eigen::Matrix4d T4 = matrix_exponential(all_omega[3], all_v[3], joint_positions[i](3, 0)*deg_to_rad_const);
        Eigen::Matrix4d T5 = matrix_exponential(all_omega[4], all_v[4], joint_positions[i](4, 0)*deg_to_rad_const);
        Eigen::Matrix4d T6 = matrix_exponential(all_omega[5], all_v[5], joint_positions[i](5, 0)*deg_to_rad_const);

        Eigen::Matrix4d T = T1*T2*T3*T4*T5*T6*M;

        std::cout << "j" << i+1 << ":" << std::endl;
        print_pose("exponential Ur3", T);
    }
    return M;
}
// ==============================================================

// Task 5b)
// ==============================================================
Eigen::Matrix4d ur3_fk_transform(const Eigen::VectorXd &joint_positions) {
    double L1 = 0.2435;
    double L2 = 0.2132;
    double W1 = 0.1315;
    double W2 = 0.0921;
    double H1 = 0.1518;
    double H2 = -0.08535;

    Eigen::Matrix3d R01 = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R12 = rotate_y(joint_positions[1]);
    Eigen::Matrix3d R23 = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R34 = rotate_y(joint_positions[3]);
    Eigen::Matrix3d R45 = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R56 = rotation_matrix_from_euler_zyx(Eigen::Vector3d{0, -M_PI, -M_PI/2});

    Eigen::Vector3d p01{0, 0, H1};
    Eigen::Vector3d p12{0, W1, 0};
    Eigen::Vector3d p23{L1, 0, 0};
    Eigen::Vector3d p34{L2, 0, 0};
    Eigen::Vector3d p45{0, 0, H2};
    Eigen::Vector3d p56{0, W2, 0};

    Eigen::Matrix4d T01 = transformation_matrix(R01, p01);
    Eigen::Matrix4d T12 = transformation_matrix(R12, p12);
    Eigen::Matrix4d T23 = transformation_matrix(R23, p23);
    Eigen::Matrix4d T34 = transformation_matrix(R34, p34);
    Eigen::Matrix4d T45 = transformation_matrix(R45, p45);
    Eigen::Matrix4d T56 = transformation_matrix(R56, p56);

    Eigen::Matrix4d T06 = T01*T12*T23*T34*T45*T56;

    return T06;
}
// ==============================================================

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

    wrench(fw, ms, ews);
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

    Fh << twist(wh, vh);
    Fa << twist(wa, va);

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

    Ff = adjoint_matrix(Thf).transpose()*Fh + adjoint_matrix(Taf).transpose()*Fa;
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
    std::cout << "Task 4a)" << std::endl;
    Eigen::Matrix4d test = planar_3r_fk_transform(joint_positions);
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
    test = ur3e_fk_screw(joint_positions_5);
    // ==============================================================

    // Task 5b)
    // ==============================================================
    std::cout << "Task 5b)" << std::endl;
    test = ur3_fk_transform(j1*deg_to_rad_const);
    print_pose("transformation ur3", test);

    test = ur3_fk_transform(j2*deg_to_rad_const);
    print_pose("transformation ur3", test);

    test = ur3_fk_transform(j3*deg_to_rad_const);
    print_pose("transformation ur3", test);
    // ==============================================================
    return 0;
}