#include <iostream>
#include <numeric>
#include <Eigen/Dense>

const double deg_to_rad_const = 0.0174532925;
const double rad_to_deg_const = 57.29578;

Eigen::Matrix3d skew_symmetric(Eigen::Vector3d v) {
    Eigen::Matrix3d skew_symetric_matrix;

    skew_symetric_matrix <<
        0, -v(2), v(1),
    v(2), 0, -v(0),
    -v(1), v(0), 0;

    return skew_symetric_matrix;
}

void print_pose(const std::string &label, const Eigen::Matrix4d &tf) {
    std::cout << label << std::endl;
    std::cout << euler_zyx_from_rotation(tf.block<3, 3>(0, 0)).transpose()*rad_to_deg_const << std::endl;
    std::cout << tf.block<3, 1>(0, 3).transpose() << std::endl;
    std::cout << "\n" << std::endl;
}

Eigen::VectorXd screw_axis(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h) {
    // Antar rotasjonshastighet på 1
    Eigen::VectorXd screwAxis(6);

    Eigen::Vector3d w = s;
    Eigen::Vector3d v = -s.cross(q) + h*s;

    screwAxis << w(0), w(1), w(2), v(0), v(1), v(2);
    screwAxis = screwAxis.transpose();

    return screwAxis;
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

Eigen::VectorXd screw_axis(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h) {
    // Antar rotasjonshastighet på 1
    Eigen::VectorXd screwAxis(6);

    Eigen::Vector3d w = s;
    Eigen::Vector3d v = -s.cross(q) + h*s;

    screwAxis << w(0), w(1), w(2), v(0), v(1), v(2);
    screwAxis = screwAxis.transpose();

    return screwAxis;
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

Eigen::Matrix4d matrix_exponential(const Eigen::Vector3d &w, const Eigen::Vector3d &v, double theta) {
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();


    Eigen::Matrix3d R = matrix_exponential(w, theta);
    Eigen::Vector3d p = (I*theta + (1 - std::cos(theta))*skew_symmetric(w) + (theta - std::sin(theta))*skew_symmetric(w)*skew_symmetric(w))*v;


    Eigen::Matrix4d T = transformation_matrix(R, p);

    return T;
}

Eigen::VectorXd std_vector_xd(const std::vector<doube> &v) {

    Eigen::VectorXd r(v.size());

    for(int i = 0; i < v.size(); i++) {
        r[i] = v[i];
    }

    return r;
}

bool is_average_below_eps(const std::vector<double> &values, double eps = 10e-7, uint8_t n_values = 5u) {

    if(values.size() < n_values) {
        return false;
    }

    const double sum = std::accumulate(values.end()-n_values, values.end(), 0.0);
    const bool check = std::abs(sum/n_values) < eps;

    return check;
}

std::pair<Eigen::Vector4d, std::vector<Eigen::VectorXd>> ur3e_space_chain() {
    double L1 = 0.2435;
    double L2 = 0.2132;
    double W1 = 0.1315;
    double W2 = 0.0921;
    double H1 = 0.1518;
    double H2 = -0.08535;

    Eigen::Matrix3d mr = rotate_y(-90.0 * deg_to_rad_const)
        * rotate_x(-90.0 * deg_to_rad_const)
        * rotate_z(-90 * deg_to_rad_const);

    Eigen::Matrix4d m = transformation_matrix(mr, Eigen::Vector3d{L1 + L2, W1 + W2, H1 - H2});

    std::vector<Eigen::VectorXd> screws{
        screw_axis({0, 0, 0}, {0, 0, 1}, 0),
        screw_axis({0, 0, H1}, {0, 1, 0}, 0),
        screw_axis({L1, 0, H1}, {0, 1, 0}, 0),
        screw_axis({L1 + L2, 0, H1}, {0, 1, 0}, 0),
        screw_axis({L1 + L2, W1, 0}, {0, 0, -1}, 0),
        screw_axis({L1 + L2, 0, H1 - H2}, {0, 1, 0}, 0)
    };
    return std::make_pair(m, screws);
}

Eigen::Matrix4d matirx_exponential(const Eigen::VectorXd &screw, double theta) {
    return matrix_exponential(screw.head<3>(), screw.tail<3>(), theta);
}

Eigen::Matrix4d ur3_space_fk(const Eigen::VectorXd &joint_positions) {
    auto [m, space_screws] = ur3e_space_chain();
    Eigen::Matrix4d t06 = Eigen::Matrix4d::Identity();

    for(int i = 0; i < joint_positions.size(); i++) {
        t06 *= matrix_exponential(space_screws[i], joint_positions[i]);
    }
    return t06*m;
}



int main(){



  return 0;
 }
