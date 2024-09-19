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

Eigen::Matrix3d skew_symmetric(Eigen::Vector3d v) {
    Eigen::Matrix3d skew_symetric_matrix;

    skew_symetric_matrix <<
        0, -v(2), v(1),
    v(2), 0, -v(0),
    -v(1), v(0), 0;

    return skew_symetric_matrix;
}

Eigen::Matrix3d rotation_matrix_from_axis_angle(const Eigen::Vector3d &axis, double radians)
{
    Eigen::Matrix3d matrix;
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

    return twist;

}
// =================================================================

// Task 1 c)
// =================================================================
Eigen::VectorXd screw_axis(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h) {
    // Antar rotasjonshastighet på 1
    Eigen::VectorXd screwAxis(6);

    Eigen::Vector3d w = s;
    Eigen::Vector3d v = -s.cross(q) + h*s;

    screwAxis << w(0), w(1), w(2), v(0), v(1), v(2);
    screwAxis = screwAxis.transpose();

    return screwAxis;
}
// =================================================================

// Task 1 d)
// ==============================================================
Eigen::MatrixXd adjoint_matrix(const Eigen::Matrix4d &tf){
    Eigen::MatrixXd adjoint(6, 6);
    Eigen::Matrix3d pR;
    Eigen::Matrix3d skewSymetric;
    Eigen::Matrix3d R;
    Eigen::Vector3d p;

    p <<
        tf(0, 3),
    tf(1, 3),
    tf(2, 3);


    skewSymetric <<
        0, -p(2), p(1),
    p(2), 0, -p(0),
    -p(1), p(0), 0;

    R <<
        tf(0, 0), tf(0, 1), tf(0, 2),
    tf(1, 0), tf(1, 1), tf(1, 2),
    tf(2, 0), tf(2, 1), tf(2, 2);

    pR = skewSymetric*R;

    adjoint <<
        R(0, 0), R(0, 1), R(0, 2), 0, 0, 0,
    R(1, 0), R(1, 1), R(1, 2), 0, 0, 0,
    R(2, 0), R(2, 1), R(2, 2), 0, 0, 0,
    pR(0, 0), pR(0, 1), pR(0, 2), R(0, 0), R(0, 1), R(0, 2),
    pR(1, 0), pR(1, 1), pR(1, 2), R(1, 0), R(1, 1), R(1, 2),
    pR(2, 0), pR(2, 1), pR(2, 2), R(2, 0), R(2, 1), R(2, 2);

    return adjoint;
}
// ==============================================================

// Task 1 e)
// ==============================================================
double cot(double x) {
    return std::cos(x)/std::sin(x);
}
// ==============================================================

// Task 2 a)
// ==============================================================
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
    wrench_s << ms(0), ms(1), ms(2), fs(0), fs(1), fs(2);

    mw = Rws.transpose() * ms;
    wrench_w << mw(0), mw(1), mw(2), fw(0), fw(1), fw(2);

    std::cout << wrench_s.transpose() << std::endl;
    std::cout << wrench_w.transpose() << std::endl;
}
// ==============================================================


// Task 3 a)
// ==============================================================
Eigen::Matrix3d matrix_exponential(const Eigen::Vector3d &w, double theta) {
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    return I + std::sin(theta)*skew_symmetric(w) + (1 - std::cos(theta))*skew_symmetric(w)*skew_symmetric(w);
}
// ==============================================================


// Task 3 b)
// ==============================================================
std::pair<Eigen::Vector3d, double> matrix_logarithm(const Eigen::Matrix3d &r) {
    double theta = 0;
    Eigen::Vector3d w = Eigen::Vector3d::Zero();

    // Bruker trace for å sjekke diagonalen til r
    double tr_r = r.trace();

    if (std::abs(tr_r + 1) < 1e-6) {
        theta = M_PI;

        if (r(2, 2) > 0){
            w = 1/(std::sqrt(2*(1 + r(2, 2)))) * Eigen::Vector3d(r(0, 2), r(1, 2), 1 + r(2, 2));
        } else if (r(1, 1) > 0) {
            w = 1/(std::sqrt(2*(1 + r(1, 1)))) * Eigen::Vector3d(r(0, 1), 1 + r(1, 1), r(2, 1));
        } else {
            w = 1/(std::sqrt(2*(1 + r(0, 0)))) * Eigen::Vector3d(1 + r(0, 0), r(1, 0), r(2, 0));
        }
    } else {
        theta = std::acos(1.0/2 * (tr_r - 1));
        Eigen::Matrix3d w_skew = 1.0/(2*std::sin(theta))*(r - r.transpose());
        w << w_skew(2, 1), w_skew(0, 2), w_skew(1, 0);
    }
    return {w, theta};
}
// ==============================================================


// Task 3 c)
// ==============================================================
Eigen::Matrix4d matrix_exponential(const Eigen::Vector3d &w, const Eigen::Vector3d &v, double theta) {
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();


    Eigen::Matrix3d R = matrix_exponential(w, theta);
    Eigen::Vector3d p = (I*theta + (1 - std::cos(theta))*skew_symmetric(w) + (theta - std::sin(theta))*skew_symmetric(w)*skew_symmetric(w))*v;


    Eigen::Matrix4d T = transformation_matrix(R, p);

    return T;
}
// ==============================================================


// Task 3 d)
// ==============================================================
std::pair<Eigen::VectorXd, double> matrix_logarithm(const Eigen::Matrix4d &t) {
    Eigen::Vector3d w = Eigen::Vector3d::Zero();
    Eigen::Vector3d p = t.block<3, 1> (0, 3);
    Eigen::Matrix3d R = t.block<3, 3> (0, 0);
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    Eigen::Vector3d v;
    Eigen::VectorXd s(6);
    double theta = 0;

    if (R.isApprox(Eigen::Matrix3d::Identity())) {
        w = Eigen::Vector3d::Zero();
        v = p/p.norm();
        theta = p.norm();
    } else {
        std::tie(w,theta) = matrix_logarithm(R);
        Eigen::Matrix3d G = 1/theta*I - 1/2*skew_symmetric(w) + (1/theta - 1/2*cot(theta/2))*skew_symmetric(w)*skew_symmetric(w);
        v = G.transpose() * p;
    }

    s << w(0), w(1), w(2), v(0), v(1), v(2);

    return {s, theta};
}
// ==============================================================

// Task 4 a)
// ==============================================================
void print_pose(const std::string &label, const Eigen::Matrix4d &tf) {
    std::cout << label << std::endl;
    std::cout << euler_zyx_from_rotation(tf.block<3, 3>(0, 0)).transpose()*rad_to_deg_const << std::endl;
    std::cout << tf.block<3, 1>(0, 3).transpose() << std::endl;
    std::cout << "\n" << std::endl;
}
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

// ==============================================================
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

    std::cout << Ff.transpose() << std::endl;
    // ==============================================================
    std::vector<Eigen::Vector3d> joint_positions
    {{0.0, 0.0, 0.0}, {90.0, 0.0, 0.0}, {0.0, 90.0, 0.0}, {0.0, 0.0, 90.0}, {10.0, -15.0, 2.75}};


    Eigen::Matrix4d test = planar_3r_fk_transform(joint_positions);

    std::cout << test << std::endl;

    // ==============================================================
    test = planar_3r_fk_screw(joint_positions);

    return 0;
}