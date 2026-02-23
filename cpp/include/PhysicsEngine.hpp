
#ifndef PHYSICS_ENGINE_H
#define PHYSICS_ENGINE_H

#include <Eigen/Dense>
#include <vector>

class PhysicsEngine {
public:
    // Python의 energy_numba와 동일
    static double calculate_energy(
        const Eigen::VectorXd& move_point_flat, 
        const Eigen::MatrixXd& bound_point, 
        const Eigen::MatrixXd& e_matrix, 
        double C_val
    );

    // Python의 gradient_numba와 동일
    static Eigen::VectorXd calculate_gradient(
        const Eigen::VectorXd& move_point_flat, 
        const Eigen::MatrixXd& bound_point, 
        const Eigen::MatrixXd& e_matrix, 
        double C_val
    );
};

#endif
