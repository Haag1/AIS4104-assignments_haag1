#include <iostream>
#include <Eigen/Dense>
#include "include/math/math.h"

namespace math {
    // ============== From assignment 1 ==============
    // -------------- 2.1 --------------
    Eigen::Matrix3d skew_symmetric(Eigen::Vector3d v) {
        Eigen::Matrix3d skew_symetric_matrix;

        skew_symetric_matrix <<
            0.0, -v(2), v(1),
        v(2), 0.0, -v(0),
        -v(1), v(0), 0.0;

        return skew_symetric_matrix;
    }

    // -------------- 2.2 --------------
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
        const double radians = degrees*deg_to_rad_const;
        // implement the necessary equations and functionality.

        matrix <<
            1.0, 0.0, 0.0,
        0.0, std::cos(radians), -std::sin(radians),
        0.0, std::sin(radians), std::cos(radians);

        return matrix;
    }
    Eigen::Matrix3d rotate_y(double degrees)
    {
        Eigen::Matrix3d matrix;
        const double radians = degrees*deg_to_rad_const;
        // implement the necessary equations and functionality.

        matrix <<
            std::cos(radians), 0.0, std::sin(radians),
        0.0, 1.0, 0.0,
        -std::sin(radians), 0.0, std::cos(radians);

        return matrix;
    }
    Eigen::Matrix3d rotate_z(double degrees)
    {
        Eigen::Matrix3d matrix;
        const double radians = degrees*deg_to_rad_const;
        // implement the necessary equations and functionality.

        matrix <<
            std::cos(radians), -std::sin(radians), 0.0,
        std::sin(radians), std::cos(radians), 0.0,
        0.0, 0.0, 1.0;

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
            1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0;

        R_euler = I * rotate_z(e(0)) * rotate_y(e(1)) * rotate_x(e(2));

        return R_euler;
    }

    // -------------- 2.3 --------------
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

    // ============== From assignment 2 ==============
    bool floatEquals(double a, double b) {
        return std::abs(a - b) < 1e-6;
    }

    // -------------- 1. --------------
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

    Eigen::VectorXd twist(const Eigen::Vector3d &w, const Eigen::Vector3d &v){
        Eigen::VectorXd twist(6);

        twist << w, v;

        return twist;
    }

    Eigen::VectorXd screw_axis(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h) {
        // Antar rotasjonshastighet på 1
        Eigen::VectorXd screwAxis(6);

        Eigen::Vector3d w = s;
        Eigen::Vector3d v = -s.cross(q) + h*s;

        screwAxis << w, v;;

        return screwAxis;
    }

    Eigen::MatrixXd adjoint_matrix(const Eigen::Matrix4d &tf){
        Eigen::MatrixXd adjoint(6, 6);
        Eigen::Matrix3d pR;
        Eigen::Matrix3d skewSymetric;
        Eigen::Matrix3d R;
        Eigen::Vector3d p;

        p = tf.block<3, 1>(0, 3);


        skewSymetric = math::skew_symmetric(p);

        R = tf.block<3, 3>(0, 0);

        pR = skewSymetric*R;

        adjoint.block<3, 3>(0, 0) = R;
        adjoint.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();
        adjoint.block<3, 3>(3, 0) = pR;
        adjoint.block<3, 3>(3, 3) = R;

        /*
            R(0, 0), R(0, 1), R(0, 2), 0, 0, 0,
        R(1, 0), R(1, 1), R(1, 2), 0, 0, 0,
        R(2, 0), R(2, 1), R(2, 2), 0, 0, 0,
        pR(0, 0), pR(0, 1), pR(0, 2), R(0, 0), R(0, 1), R(0, 2),
        pR(1, 0), pR(1, 1), pR(1, 2), R(1, 0), R(1, 1), R(1, 2),
        pR(2, 0), pR(2, 1), pR(2, 2), R(2, 0), R(2, 1), R(2, 2);
        */

        return adjoint;
    }

    double cot(double x) {
        return 1.0/std::tan(x);
    }

    // -------------- 2. --------------
    void wrench(Eigen::Vector3d fw, Eigen::Vector3d ms, Eigen::Vector3d ews) {
        // print variables
        Eigen::VectorXd wrench_w(6);
        Eigen::VectorXd wrench_s(6);

        // Equation variables
        Eigen::Vector3d fs;
        Eigen::Vector3d mw;
        Eigen::Matrix3d Rws;

        // fw = rws * fs
        // fs = rws^-1 * fw
        Rws = rotation_matrix_from_euler_zyx(ews * deg_to_rad_const);
        fs = Rws * fw;
        wrench_s << ms, fs;

        mw = Rws.transpose() * ms;
        // endre
        wrench_w << mw, fw;

        std::cout << "Task 2a)" << std::endl;
        std::cout << "s:" << std::endl;
        std::cout << wrench_s.transpose() << std::endl;

        std::cout << "w:" << std::endl;
        std::cout << wrench_w.transpose() << std::endl;
    }

    // -------------- 3. --------------

    Eigen::Matrix3d matrix_exponential(const Eigen::Vector3d &w, double theta) {
        return Eigen::Matrix3d::Identity() + std::sin(theta)*skew_symmetric(w) +
            (1.0 - std::cos(theta))*skew_symmetric(w)*skew_symmetric(w);
    }

    std::pair<Eigen::Vector3d, double> matrix_logarithm(const Eigen::Matrix3d &r) {
        double theta = 0.0;
        Eigen::Vector3d w = Eigen::Vector3d::Zero();

        // Bruker trace for å sjekke diagonalen til r
        double tr_r = r.trace();

        if (std::abs(tr_r + 1) < 1e-6) {
            theta = M_PI;

            // Bruk float equals og double literal i tall
            if (!floatEquals(r(2, 2), -1)){
                w = 1.0/(std::sqrt(2.0*(1.0 + r(2, 2)))) * Eigen::Vector3d(r(0, 2),
                    r(1, 2), 1.0 + r(2, 2));
            } else if (floatEquals(r(1, 1), 0)) {
                w = 1.0/(std::sqrt(2.0*(1.0 + r(1, 1)))) * Eigen::Vector3d(r(0, 1), 1.0 +
                    r(1, 1), r(2, 1));
            } else {
                w = 1.0/(std::sqrt(2.0*(1.0 + r(0, 0)))) * Eigen::Vector3d(1.0 + r(0, 0), r(1, 0),
                    r(2, 0));
            }
        } else {
            theta = std::acos(0.5 * (tr_r - 1));
            Eigen::Matrix3d w_skew = 1.0/(2.0*std::sin(theta))*(r - r.transpose());
            w << w_skew(2, 1), w_skew(0, 2), w_skew(1, 0);
        }
        return {w, theta};
    }

    Eigen::Matrix4d matrix_exponential(const Eigen::Vector3d &w, const Eigen::Vector3d &v, double theta) {
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Eigen::Matrix4d T;

        if(w.norm() == 1) {
            Eigen::Matrix3d R = matrix_exponential(w, theta);
            Eigen::Vector3d p = (I*theta + (1.0 - std::cos(theta))*skew_symmetric(w) +
                (theta - std::sin(theta))*skew_symmetric(w)*skew_symmetric(w))*v;
            T = transformation_matrix(R, p);
        } else {
            T = transformation_matrix(I, theta*v);
        }

        return T;
    }

    std::pair<Eigen::VectorXd, double> matrix_logarithm(const Eigen::Matrix4d &t) {
        Eigen::Vector3d w = Eigen::Vector3d::Zero();
        Eigen::Vector3d p = t.block<3, 1> (0, 3);
        Eigen::Matrix3d R = t.block<3, 3> (0, 0);
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

        Eigen::Vector3d v;
        Eigen::VectorXd s(6);
        double theta = 0.0;

        if (R.isApprox(Eigen::Matrix3d::Identity())) {
            w = Eigen::Vector3d::Zero();
            v = p.normalized();
            theta = p.norm();

        } else {
            Eigen::Matrix3d w_skewsymetric = skew_symmetric(w);
            std::tie(w,theta) = matrix_logarithm(R);
            double theta_1 = 1.0/theta;

            Eigen::Matrix3d G = theta_1*I - 0.5*w_skewsymetric + (theta_1 - 0.5*cot(theta/2.0))*
                w_skewsymetric*w_skewsymetric;
            v = G.transpose() * p;
        }

        s << w, v;

        return {s, theta};
    }

    // -------------- 4. --------------
    void print_pose(const std::string &label, const Eigen::Matrix4d &tf) {
        std::cout << label << std::endl;
        std::cout << "angle: " << euler_zyx_from_rotation(tf.block<3, 3>(0, 0)).transpose()*
            rad_to_deg_const << std::endl;
        std::cout << "Pos: " << tf.block<3, 1>(0, 3).transpose() << std::endl;
        std::cout << "\n" << std::endl;
    }

    Eigen::Matrix4d planar_3r_fk_transform(const std::vector<Eigen::Vector3d> &joint_positions) {
        const unsigned long positionVectorLength = joint_positions.size();

        Eigen::Vector3d p{10.0, 0.0, 0.0};

        Eigen::Matrix4d T04;

        for(int i = 0; i < positionVectorLength; i++) {
            Eigen::Matrix3d R1 = rotate_z(joint_positions[i][0]*deg_to_rad_const);
            Eigen::Matrix3d R2 = rotate_z(joint_positions[i][1]*deg_to_rad_const);
            Eigen::Matrix3d R3 = rotate_z(joint_positions[i][2]*deg_to_rad_const);
            Eigen::Matrix4d T01 = transformation_matrix(R1, {0.0, 0.0, 0.0});
            Eigen::Matrix4d T12 = transformation_matrix(R2, p);
            Eigen::Matrix4d T23 = transformation_matrix(R3, p);
            Eigen::Matrix4d T34 = transformation_matrix(Eigen::Matrix3d::Identity(), p);

            Eigen::Matrix4d T04_temp = T01*T12*T23*T34;

            std::cout << "j" << i + 1 << ":" << std::endl;
            print_pose("Angle and pos", T04_temp);
        }
        return T04;
    }

    Eigen::Matrix4d planar_3r_fk_screw(const std::vector<Eigen::Vector3d> &joint_positions) {
        const unsigned long positionVectorLength = joint_positions.size();
        const double L = 10.0;
        Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
        M(0, 3) = 3*L;

        for(int i = 0; i < positionVectorLength; i++) {
            Eigen::Vector3d w1{0.0, 0.0, 1.0};
            Eigen::Vector3d w2{0.0, 0.0, 1.0};
            Eigen::Vector3d w3{0.0, 0.0, 1.0};

            Eigen::Vector3d v1{0.0, 0.0, 0.0};
            Eigen::Vector3d v2{0.0, -L, 0.0};
            Eigen::Vector3d v3{0.0, -2*L, 0.0};

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

    // -------------- 5. --------------
    Eigen::Matrix4d ur3e_fk_screw(const std::vector<Eigen::VectorXd> &joint_positions) {
        double L1 = 0.2435;
        double L2 = 0.2132;
        double W1 = 0.1315;
        double W2 = 0.0921;
        double H1 = 0.1518;
        double H2 = -0.08535;

        // Kann bruke screw axis_axis(s, q, h) for å finne v
        std::vector<Eigen::Vector3d> all_omega{
            {0.0, 0.0, 1.0}, {0.0, 1.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, -1.0},
            {0.0, 1.0, 0.0}};

        std::vector<Eigen::Vector3d> all_v{
            {0.0, 0.0, 0.0}, {-H1, 0.0, 0.0}, {-H1, 0.0, L1}, {-H1, 0.0, L1+L2}, {-W1, L1 + L2, 0.0},
            {H2-H1, 0.0, L1+L2}};

        Eigen::Matrix4d M;

        M << -1.0, 0.0, 0.0, L1+L2,
        0.0, 0.0, 1.0, W1+W2,
        0.0, 1.0, 0.0, H1+H2,
        0.0, 0.0, 0.0, 1.0;

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

        Eigen::Vector3d p01{0.0, 0.0, H1};
        Eigen::Vector3d p12{0.0, W1, 0.0};
        Eigen::Vector3d p23{L1, 0.0, 0.0};
        Eigen::Vector3d p34{L2, 0.0, 0.0};
        Eigen::Vector3d p45{0.0, 0.0, H2};
        Eigen::Vector3d p56{0.0, W2, 0.0};

        Eigen::Matrix4d T01 = transformation_matrix(R01, p01);
        Eigen::Matrix4d T12 = transformation_matrix(R12, p12);
        Eigen::Matrix4d T23 = transformation_matrix(R23, p23);
        Eigen::Matrix4d T34 = transformation_matrix(R34, p34);
        Eigen::Matrix4d T45 = transformation_matrix(R45, p45);
        Eigen::Matrix4d T56 = transformation_matrix(R56, p56);

        Eigen::Matrix4d T06 = T01*T12*T23*T34*T45*T56;

        return T06;
    }
}