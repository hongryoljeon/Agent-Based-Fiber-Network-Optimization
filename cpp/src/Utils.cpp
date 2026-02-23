#include "Utils.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <filesystem> // C++17 이상

namespace fs = std::filesystem;

void Utils::write_matrix_to_csv(const std::string& filename, const Eigen::MatrixXd& matrix) {
    std::ofstream file(filename);
    if (file.is_open()) {
        for (int i = 0; i < matrix.rows(); ++i) {
            for (int j = 0; j < matrix.cols(); ++j) {
                file << std::setprecision(15) << matrix(i, j);
                if (j < matrix.cols() - 1) file << ",";
            }
            file << "\n";
        }
    }
}

void Utils::save_simulation_data(
    int en, 
    int sim_num, 
    const std::string& folder_name, 
    const Eigen::MatrixXd& node, 
    const Eigen::MatrixXd& node_bound, 
    const Eigen::MatrixXd& e_matrix) 
{
    // 폴더 생성
    fs::create_directories(folder_name);

    char buffer[20];
    std::snprintf(buffer, sizeof(buffer), "%03d", en);
    std::string prefix = std::string(buffer) + "_";

    // 파일 저장
    write_matrix_to_csv(folder_name + "/" + prefix + "e_matrix.csv", e_matrix);
    write_matrix_to_csv(folder_name + "/" + prefix + "node.csv", node);
    write_matrix_to_csv(folder_name + "/" + prefix + "node_bound.csv", node_bound);
    
    std::cout << "✅ Data saved to: " << folder_name << " (Ensemble " << en << ")" << std::endl;
}

double Utils::check_convergence(
    bool success,
    const std::string& message,
    double final_energy,
    const Eigen::VectorXd& final_grad,
    int iterations,
    double threshold) 
{
    // L-infinity norm (Max absolute value)
    double max_force = final_grad.array().abs().maxCoeff();
    double mean_force = final_grad.array().abs().mean();

    std::cout << std::string(30, '-') << "\n";
    std::cout << "Optimization Status: " << (success ? "SUCCESS" : "FAILED") << "\n";
    std::cout << "Message: " << message << "\n";
    std::cout << "Final Energy: " << std::scientific << final_energy << "\n";
    std::cout << "Maximum Force (inf-norm): " << std::scientific << max_force << "\n";
    std::cout << "Average Force: " << mean_force << "\n";
    std::cout << "Number of Iterations: " << iterations << "\n";

    if (max_force > threshold) {
        std::cout << "⚠️ WARNING: Convergence threshold (" << threshold << ") not met!\n";
    } else {
        std::cout << "✅ Success: System reached mechanical equilibrium.\n";
    }
    std::cout << std::string(30, '-') << std::endl;

    return max_force;
}
