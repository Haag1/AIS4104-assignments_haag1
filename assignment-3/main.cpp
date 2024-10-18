#include <iostream>
#include <functional>
#include <numeric>
#include <Eigen/Dense>
#include "../math/include/math/math.h"


// ===================================== Task 1) =====================================
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

    math::print_pose("space transformation five: ",
        ur3_space_fk(std_vector_to_eigen(
            std::vector<double>{45.0, -20.0, 10.0, 2.5, 30.0, -50.0})*math::deg_to_rad_const));
    math::print_pose("Body transformation five: ",
        ur3_body_fk(std_vector_to_eigen(
            std::vector<double>{45.0, -20.0, 10.0, 2.5, 30.0, -50.0})*math::deg_to_rad_const));
}

// ===================================== Task 2) =====================================
std::pair<uint32_t, double> newton_raphson_root_find(const std::function<double(double)> &f,
    double x_0, double dx_0 = 0.5, double eps = 10e-7) {
    int max_iterations = 1000;
    int iteration_count = 0;
    double a_n = 0;
    double x_n = x_0;

    while(iteration_count < max_iterations) {
        a_n = (f(x_n + dx_0) - f(x_n))/dx_0;

        x_n = x_n - f(x_n)/a_n;

        if(abs(f(x_n)) < eps) {
            break;
        }

        // std::cout << iteration_count << " - " << x_n << std::endl;
        iteration_count++;
    }

    return std::pair(iteration_count, x_n);
}

std::pair<uint32_t, double> gradient_descent_root_find(const std::function<double(double)> &f,
    double x_0, double gamma = 0.1, double dx_0 = 0.5, double eps = 10e-7) {
    int max_iterations = 1000;
    int iteration_count = 0;
    double a_n = 0;
    double x_n = x_0;
    double error = 0;
    double solution_candidate = 0;

    while(iteration_count < max_iterations) {
        a_n = dx_0/(f(x_n + dx_0) - f(x_n));
        error = f(x_n);

        x_n = x_n - gamma*a_n*error;
        if(abs(error) < eps) {
            break;
        }

        //std::cout << iteration_count << " - " << f(x_n) << std::endl;
        iteration_count++;
    }

    return std::pair(iteration_count, x_n);

}


void test_newton_raphson_root_find(const std::function<double(double)> &f, double x0)
{
    auto [iterations, x_hat] = newton_raphson_root_find(f, x0);
    std::cout << "NR root f, x0=" << x0 << " -> it=" << iterations << " x=" << x_hat << " f(x)=" <<
        f(x_hat) << std::endl;
}

void test_gradient_descent_root_find(const std::function<double(double)> &f, double x0)
{
    auto [iterations, x_hat] = gradient_descent_root_find(f, x0);
    std::cout << "GD root f, x0=" << x0 << " -> it=" << iterations << " x=" << x_hat << " f(x)=" <<
        f(x_hat) << std::endl;
}

void test_root_find()
{
    std::cout << "Root finding tests" << std::endl;

    auto f1 = [](double x)
    {
        return (x - 3.0) * (x - 3.0) - 1.0;
    };
    test_newton_raphson_root_find(f1, -20);
    test_gradient_descent_root_find(f1, -20);
}


// ===================================== Task 3) =====================================
Eigen::MatrixXd ur3e_space_jacobian(const Eigen::VectorXd &current_joint_positions) {
    auto[M, screws] = ur3e_space_chain();

    Eigen::Matrix4d matrix_exponential_products = Eigen::Matrix4d::Identity();
    Eigen::MatrixXd Js(6, 6);

    Js.col(0) = screws[0];

    for (int i = 1; i < 6; i++) {
        Eigen::Vector3d w = screws[i-1].block<3, 1>(0, 0);
        Eigen::Vector3d v = screws[i-1].block<3, 1>(3, 0);
        matrix_exponential_products *= math::matrix_exponential(w, v, current_joint_positions[i-1]);

        Js.col(i) = math::adjoint_matrix(matrix_exponential_products)*screws[i];
    }

    return Js;
}

Eigen::MatrixXd ur3e_body_jacobian(const Eigen::VectorXd &current_joint_positions) {
    auto[M, screws] = ur3e_body_chain();

    Eigen::Matrix4d matrix_exponential_products = Eigen::Matrix4d::Identity();
    Eigen::MatrixXd Js(6, 6);

    Js.col(5) =screws[5];

    for (int i = 4; i >= 0; i--) {
        Eigen::Vector3d w = screws[i+1].block<3, 1>(0, 0);
        Eigen::Vector3d v = screws[i+1].block<3, 1>(3, 0);
        matrix_exponential_products *= math::matrix_exponential(w, v, current_joint_positions[i+1]).inverse();

        Js.col(i) = math::adjoint_matrix(matrix_exponential_products)*screws[i];
    }

    return Js;
}

void ur3e_test_jacobian(const Eigen::VectorXd &joint_positions){
    Eigen::Matrix4d tsb = ur3_body_fk(joint_positions);
    //auto [m, space_screws] = ur3e_space_chain();
    Eigen::MatrixXd jb = ur3e_body_jacobian(joint_positions);
    Eigen::MatrixXd js = ur3e_space_jacobian(joint_positions);
    Eigen::MatrixXd ad_tsb = math::adjoint_matrix(tsb);
    Eigen::MatrixXd ad_tbs = math::adjoint_matrix(tsb.inverse());

    std::cout << "Jb: " << std::endl << jb << std::endl << "Ad_tbs*Js:" <<
        std::endl << ad_tbs * js << std::endl << std::endl;
    std::cout << "Js: " << std::endl << js << std::endl << "Ad_tsb*Jb:" <<
        std::endl << ad_tsb * jb << std::endl << std::endl;
    std::cout << "d Jb: " << std::endl << jb - ad_tbs * js << std::endl << std::endl;
    std::cout << "d Js: " << std::endl << js - ad_tsb * jb << std::endl << std::endl;
}
void ur3e_test_jacobian(){
    std::cout << "Jacobian matrix tests" << std::endl;
    ur3e_test_jacobian(std_vector_to_eigen(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0})
        * math::deg_to_rad_const);
    ur3e_test_jacobian(std_vector_to_eigen(std::vector<double>{45.0, -20.0, 10.0, 2.5, 30.0, -50.0})
        * math::deg_to_rad_const);
}

// ===================================== Task 4) =====================================
std::pair<size_t, Eigen::VectorXd> ur3e_ik_body(const Eigen::Matrix4d &t_sd,
    const Eigen::VectorXd &current_joint_positions, double gamma = 6e-2, double v_e = 4e-3, double w_e = 4e-3) {

    // Initialize parameters
    auto J = ur3e_body_jacobian(current_joint_positions);
    auto new_pos = current_joint_positions;
    const int max_loops = 1000;
    int actual_loops = max_loops;
    bool angle_converged = false;
    bool position_converged = false;
    bool converged = false;
    Eigen::VectorXd screw_velocity(6);
    Eigen::VectorXd screw_error(6);

    for(int l = 0; l < max_loops; l++) {
        // New transformation matrix for current position
        Eigen::Matrix4d t_sb = ur3_body_fk(new_pos);

        // Calculate error position, and transform it in to a screw describing the error
        Eigen::Matrix4d t_error = t_sd - t_sb;
        std::pair<Eigen::VectorXd, double> error_logarithm_pair = math::matrix_logarithm(t_error);
        screw_error = error_logarithm_pair.first;

        // Check if converged before doing more gradient decent
        position_converged = screw_error.tail<3>().norm() < v_e;
        angle_converged = screw_error.head<3>().norm() < w_e;

        // converged if both position and twist pass the test
        converged = position_converged and angle_converged;
        if(converged) {
            actual_loops = l;
            break;
        }

        // Calculate end-effector velocity from error
        Eigen::Matrix4d velocity = t_sd.inverse() * t_sb;
        std::pair<Eigen::VectorXd, double> velocity_logarithm_pair = math::matrix_logarithm(velocity);
        screw_velocity = velocity_logarithm_pair.first;

        // Create a Jacobien in body frame
        J = ur3e_body_jacobian(new_pos);

        // calculate new joint angles from screw velocity and transpose of Jacobien
        new_pos = new_pos - gamma*J.transpose()*screw_velocity;
    }
    std::cout << "Error at end:" << " " << screw_error.tail<3>().norm() << std::endl;
    std::cout << "Error at end:" << " " << screw_error.head<3>().norm() << std::endl;
    return std::make_pair(actual_loops, new_pos);
}

void ur3e_ik_test_pose(const Eigen::Vector3d &pos, const Eigen::Vector3d &zyx, const Eigen::VectorXd &j0)
{
    std::cout << "Test from pose" << std::endl;
    Eigen::Matrix4d t_sd = math::transformation_matrix(math::rotation_matrix_from_euler_zyx(zyx), pos);
    auto [iterations, j_ik] = ur3e_ik_body(t_sd, j0);
    Eigen::Matrix4d t_ik = ur3_body_fk(j_ik);
    math::print_pose(" IK pose", t_ik);
    math::print_pose("Desired pose", t_sd);
    std::cout << "Converged after " << iterations << " iterations" << std::endl;
    std::cout << "J_0: " << j0.transpose() * math::rad_to_deg_const << std::endl;
    std::cout << "J_ik: " << j_ik.transpose() * math::rad_to_deg_const << std::endl << std::endl;
}
void ur3e_ik_test_configuration(const Eigen::VectorXd &joint_positions, const Eigen::VectorXd &j0)
{
    std::cout << "Test from configuration" << std::endl;
    Eigen::Matrix4d t_sd = ur3_space_fk(joint_positions);
    auto [iterations, j_ik] = ur3e_ik_body(t_sd, j0);
    Eigen::Matrix4d t_ik = ur3_body_fk(j_ik);
    math::print_pose(" IK pose", t_ik);
    math::print_pose("Desired pose", t_sd);
    std::cout << "Converged after " << iterations << " iterations" << std::endl;
    std::cout << "J_0: " << j0.transpose() * math::rad_to_deg_const << std::endl;
    std::cout << "J_d: " << joint_positions.transpose() * math::rad_to_deg_const << std::endl;
    std::cout << "J_ik: " << j_ik.transpose() * math::rad_to_deg_const << std::endl << std::endl;
}

void ur3e_ik_test()
{
    Eigen::VectorXd j_t0 = std_vector_to_eigen(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0})
        * math::deg_to_rad_const;
    Eigen::VectorXd j_t1 = std_vector_to_eigen(std::vector<double>{0.0, 0.0, -89.0, 0.0, 0.0, 0.0})
        * math::deg_to_rad_const;
    ur3e_ik_test_pose(Eigen::Vector3d{0.3289, 0.22315, 0.36505},
        Eigen::Vector3d{0.0, 90.0, -90.0} * math::deg_to_rad_const, j_t0);
    ur3e_ik_test_pose(Eigen::Vector3d{0.3289, 0.22315, 0.36505},
        Eigen::Vector3d{0.0, 90.0, -90.0} * math::deg_to_rad_const, j_t1);

    Eigen::VectorXd j_t2 = std_vector_to_eigen(std::vector<double>{50.0, -30.0, 20, 0.0, -30.0, 50.0})
        * math::deg_to_rad_const;
    Eigen::VectorXd j_d1 = std_vector_to_eigen(std::vector<double>{45.0, -20.0, 10.0, 2.5, 30.0, -50.0})
        * math::deg_to_rad_const;
    ur3e_ik_test_configuration(j_d1, j_t0);
    ur3e_ik_test_configuration(j_d1, j_t2);
}


int main(){
    ur3e_test_fk();
    test_root_find();
    //ur3e_test_jacobian();
    //ur3e_ik_test();
    return 0;
}
