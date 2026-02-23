
#include "PhysicsEngine.hpp"
#include <cmath>

double PhysicsEngine::calculate_energy(
    const Eigen::VectorXd& move_point_flat, 
    const Eigen::MatrixXd& bound_point, 
    const Eigen::MatrixXd& e_matrix, 
    double C_val) 
{
    int num_movable = move_point_flat.size() / 2;
    // 1D flat vector를 2D view처럼 사용
    auto move_point = move_point_flat.reshaped<Eigen::RowMajor>(num_movable, 2);

    double total_energy = 0.0;
    double compensation = 0.0; // Kahan Summation

    for (int i = 0; i < e_matrix.rows(); ++i) {
        Eigen::Vector2d p1, p2;

        // 시작점 설정 (Python: e_matrix[i, 0] > -0.9)
        if (e_matrix(i, 0) > -0.9) {
            p1 = bound_point.row(static_cast<int>(e_matrix(i, 0)));
        } else {
            p1 = move_point.row(static_cast<int>(e_matrix(i, 1)));
        }

        // 끝점 설정 (Python: e_matrix[i, 2] > -0.9)
        if (e_matrix(i, 2) > -0.9) {
            p2 = bound_point.row(static_cast<int>(e_matrix(i, 2)));
        } else {
            p2 = move_point.row(static_cast<int>(e_matrix(i, 3)));
        }

        double dx = p2.x() - p1.x();
        double dy = p2.y() - p1.y();
        double length = std::sqrt(dx*dx + dy*dy);
        double rest_length = e_matrix(i, 5);
        double strain = (length / rest_length) - 1.0;

        // Kahan summation 로직 이식
        double increment = 0.5 * rest_length * std::pow(strain, 2);
        double y_val = increment - compensation;
        double t = total_energy + y_val;
        compensation = (t - total_energy) - y_val;
        total_energy = t;
    }

    return total_energy * C_val;
}

Eigen::VectorXd PhysicsEngine::calculate_gradient(
    const Eigen::VectorXd& move_point_flat, 
    const Eigen::MatrixXd& bound_point, 
    const Eigen::MatrixXd& e_matrix, 
    double C_val) 
{
    int num_movable = move_point_flat.size() / 2;
    auto move_point = move_point_flat.reshaped<Eigen::RowMajor>(num_movable, 2);
    
    Eigen::MatrixXd grad = Eigen::MatrixXd::Zero(num_movable, 2);

    for (int i = 0; i < e_matrix.rows(); ++i) {
        bool is_start_m = e_matrix(i, 0) < -0.9;
        bool is_end_m = e_matrix(i, 2) < -0.9;

        Eigen::Vector2d p1 = is_start_m ? move_point.row(static_cast<int>(e_matrix(i, 1))) 
                                        : bound_point.row(static_cast<int>(e_matrix(i, 0)));
        Eigen::Vector2d p2 = is_end_m ? move_point.row(static_cast<int>(e_matrix(i, 3))) 
                                      : bound_point.row(static_cast<int>(e_matrix(i, 2)));

        double dx = p2.x() - p1.x();
        double dy = p2.y() - p1.y();
        double L = std::sqrt(dx*dx + dy*dy);
        
        if (L < 1e-14) continue;

        double L0 = e_matrix(i, 5);
        double coeff = C_val * ((L / L0) - 1.0) / L;

        if (is_start_m) {
            int idx1 = static_cast<int>(e_matrix(i, 1));
            grad(idx1, 0) -= coeff * dx;
            grad(idx1, 1) -= coeff * dy;
        }
        if (is_end_m) {
            int idx2 = static_cast<int>(e_matrix(i, 3));
            grad(idx2, 0) += coeff * dx;
            grad(idx2, 1) += coeff * dy;
        }
    }

    // 2D Matrix를 다시 Flat한 1D Vector로 변환하여 반환
    return grad.reshaped<Eigen::RowMajor>();
}
