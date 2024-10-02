#include <iostream>
#include <functional>
#include <numeric>
#include <Eigen/Dense>
#include "../math/include/math/math.h"


// ===================================== Task 1 b) =====================================
Eigen::VectorXd std_vector_to_eigen(const std::vector<double> &v) {

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

std::pair<Eigen::Matrix4d, std::vector<Eigen::VectorXd>> ur3e_space_chain() {
    double L1 = 0.2435;
    double L2 = 0.2132;
    double W1 = 0.1315;
    double W2 = 0.0921;
    double H1 = 0.1518;
    double H2 = 0.08535;

    Eigen::Matrix3d mr = math::rotate_y(-90.0)
        * math::rotate_x(-90.0)
        * math::rotate_z(-90.0);

    Eigen::Matrix4d m = math::transformation_matrix(mr, Eigen::Vector3d{L1 + L2, W1 + W2, H1 - H2});

    std::vector<Eigen::VectorXd> screws{
        math::screw_axis({0, 0, 0}, {0, 0, 1}, 0),
        math::screw_axis({0, 0, H1}, {0, 1, 0}, 0),
        math::screw_axis({L1, 0, H1}, {0, 1, 0}, 0),
        math::screw_axis({L1 + L2, 0, H1}, {0, 1, 0}, 0),
        math::screw_axis({L1 + L2, W1, 0}, {0, 0, -1}, 0),
        math::screw_axis({L1 + L2, 0, H1 - H2}, {0, 1, 0}, 0)
    };

    return std::make_pair(m, screws);
}

Eigen::Matrix4d matrix_exponential(const Eigen::VectorXd &screw, double theta) {
    return math::matrix_exponential(screw.head<3>(), screw.tail<3>(), theta);
}

Eigen::Matrix4d ur3_space_fk(const Eigen::VectorXd &joint_positions) {
    auto [m, space_screws] = ur3e_space_chain();
    Eigen::Matrix4d t06 = Eigen::Matrix4d::Identity();

    for(int i = 0; i < joint_positions.size(); i++) {
        t06 *= matrix_exponential(space_screws[i], joint_positions[i]);
    }

    return t06*m;
}

std::pair<Eigen::Matrix4d, std::vector<Eigen::VectorXd>> ur3e_body_chain() {
    double L1 = 0.2435;
    double L2 = 0.2132;
    double W1 = 0.1315;
    double W2 = 0.0921;
    double H1 = 0.1518;
    double H2 = 0.08535;

    Eigen::Matrix3d mr = math::rotate_y(-90.0)
        * math::rotate_x(-90.0)
        * math::rotate_z(-90.0);

    Eigen::Matrix4d m = math::transformation_matrix(mr, Eigen::Vector3d{L1 + L2, W1 + W2, H1 - H2});

    std::vector<Eigen::VectorXd> screws{
        math::screw_axis({0, 0, 0}, {0, 0, 1}, 0),
        math::screw_axis({0, 0, H1}, {0, 1, 0}, 0),
        math::screw_axis({L1, 0, H1}, {0, 1, 0}, 0),
        math::screw_axis({L1 + L2, 0, H1}, {0, 1, 0}, 0),
        math::screw_axis({L1 + L2, W1, 0}, {0, 0, -1}, 0),
        math::screw_axis({L1 + L2, 0, H1 - H2}, {0, 1, 0}, 0)
    };

    std::vector<Eigen::VectorXd> beta{
        math::adjoint_matrix(m.inverse())*screws[0],
        math::adjoint_matrix(m.inverse())*screws[1],
        math::adjoint_matrix(m.inverse())*screws[2],
        math::adjoint_matrix(m.inverse())*screws[3],
        math::adjoint_matrix(m.inverse())*screws[4],
        math::adjoint_matrix(m.inverse())*screws[5]
    };

    return std::make_pair(m, beta);
}

Eigen::Matrix4d ur3_body_fk(const Eigen::VectorXd &joint_positions) {
    auto [m, beta] = ur3e_body_chain();
    Eigen::Matrix4d t06 = Eigen::Matrix4d::Identity();

    for(int i = 0; i < joint_positions.size(); i++) {
        t06 *= matrix_exponential(beta[i], joint_positions[i]);
    }

    return m*t06;
}

void ur3e_test_fk()
{
    std::cout << "Forward kinematics tests" << std::endl;
    math::print_pose("space transformation one: ",
        ur3_space_fk(std_vector_to_eigen(
            std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0})*math::deg_to_rad_const));
    math::print_pose("Body transformation one: ",
        ur3_body_fk(std_vector_to_eigen(
            std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0})*math::deg_to_rad_const));

    math::print_pose("space transformation two: ",
        ur3_space_fk(std_vector_to_eigen(
            std::vector<double>{0.0, 0.0, 0.0, -90.0, 0.0, 0.0})*math::deg_to_rad_const));
    math::print_pose("Body transformation two: ",
        ur3_body_fk(std_vector_to_eigen(
            std::vector<double>{0.0, 0.0, 0.0, -90.0, 0.0, 0.0})*math::deg_to_rad_const));

    math::print_pose("space transformation three: ",
        ur3_space_fk(std_vector_to_eigen(
            std::vector<double>{0.0, 0.0, -180.0, 0.0, 0.0, 0.0})*math::deg_to_rad_const));
    math::print_pose("Body transformation three: ",
        ur3_body_fk(std_vector_to_eigen(
            std::vector<double>{0.0, 0.0, -180.0, 0.0, 0.0, 0.0})*math::deg_to_rad_const));

    math::print_pose("space transformation four: ",
        ur3_space_fk(std_vector_to_eigen(
            std::vector<double>{0.0, 0.0, -90.0, 0.0, 0.0, 0.0})*math::deg_to_rad_const));
    math::print_pose("Body transformation four: ",
        ur3_body_fk(std_vector_to_eigen(
            std::vector<double>{0.0, 0.0, -90.0, 0.0, 0.0, 0.0})*math::deg_to_rad_const));
}

// ===================================== Task 2 b) =====================================
std::pair<uint32_t, double> newton_raphson_root_find(const std::function<double(double)> &tf,
    double x_0, double gamma = 0.1, doublle dx_0 = 0.5, double eps = 10e-7) {
    skew_v = std::log(tf.)
}


int main(){
    ur3e_test_fk();
    return 0;
 }
