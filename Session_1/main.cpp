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
    x = x * deg_to_rad_const;

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
    std::cout << cot(30) << std::endl;
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
    float N = 0;

    Eigen::VectorXd Fh(6);
    Eigen::VectorXd Fa(6);
    Eigen::Vector3d wa{0, 0, 0};
    Eigen::Vector3d va{-5, N, 0};
    Eigen::Vector3d wh{0, 0, 0};
    Eigen::Vector3d vh{0, -1, N};
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


    return 0;
}