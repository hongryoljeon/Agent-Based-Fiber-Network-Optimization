#ifndef UTILS_HPP
#define UTILS_HPP

#include <Eigen/Dense>
#include <string>
#include <vector>

class Utils {
public:
    // 디렉토리 생성 및 CSV 저장
    static void save_simulation_data(
        int en, 
        int sim_num, 
        const std::string& folder_name, 
        const Eigen::MatrixXd& node, 
        const Eigen::MatrixXd& node_bound, 
        const Eigen::MatrixXd& e_matrix
    );

    // 수렴 진단 (L-BFGS 결과 분석)
    static double check_convergence(
        bool success,
        const std::string& message,
        double final_energy,
        const Eigen::VectorXd& final_grad,
        int iterations,
        double threshold = 1e-7
    );

private:
    // Helper: 파일 저장을 위한 Eigen Matrix to CSV
    static void write_matrix_to_csv(const std::string& filename, const Eigen::MatrixXd& matrix);
};

#endif
