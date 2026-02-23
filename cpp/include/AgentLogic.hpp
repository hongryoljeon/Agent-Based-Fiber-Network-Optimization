#ifndef AGENT_LOGIC_HPP
#define AGENT_LOGIC_HPP

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <random>

// 교차점 정보를 담는 구조체
struct Intersection {
    double x;
    double y;
    int index;
};

class AgentLogic {
public:
    // Python의 line_circle_intersection 대응
    static std::vector<Intersection>& find_line_circle_intersections(
        const Eigen::MatrixXd& points_bound,
        const Eigen::MatrixXd& points_inner,
        const Eigen::VectorXd& line,
        const Eigen::Vector2d& circle_center,
        double circle_radius,
        std::vector<Intersection>& set_points,
        int index
    );

    // Python의 choose_intersection_randomly 대응
    static Intersection choose_intersection_randomly(const std::vector<Intersection>& intersections);

    // Python의 calculate_distance 대응
    static double calculate_distance(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2);

    // Python의 e_calculate_distance 대응
    static double e_calculate_distance(
        const Eigen::VectorXd& line,
        const Eigen::MatrixXd& bound_point,
        const Eigen::MatrixXd& move_point
    );
};

#endif
