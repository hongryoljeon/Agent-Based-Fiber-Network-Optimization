#include "AgentLogic.hpp"
#include <cmath>
#include <stdexcept>
#include <algorithm>

std::vector<Intersection>& AgentLogic::find_line_circle_intersections(
    const Eigen::MatrixXd& points_bound,
    const Eigen::MatrixXd& points_inner,
    const Eigen::VectorXd& line,
    const Eigen::Vector2d& circle_center,
    double circle_radius,
    std::vector<Intersection>& set_points,
    int index) 
{
    // 시작점과 끝점 좌표 결정
    Eigen::Vector2d p1 = (line(0) > -0.9) ? 
        points_bound.row(static_cast<int>(line(0))).transpose() : 
        points_inner.row(static_cast<int>(line(1))).transpose();

    Eigen::Vector2d p2 = (line(2) > -0.9) ? 
        points_bound.row(static_cast<int>(line(2))).transpose() : 
        points_inner.row(static_cast<int>(line(3))).transpose();

    // 선분 벡터 d, 원점으로부터의 벡터 f
    Eigen::Vector2d d = p2 - p1;
    Eigen::Vector2d f = p1 - circle_center;

    // 2차 방정식 at^2 + bt + c = 0 계산
    double a = d.dot(d);
    double b = 2.0 * f.dot(d);
    double c = f.dot(f) - std::pow(circle_radius, 2);

    double discriminant = std::pow(b, 2) - 4.0 * a * c;

    if (discriminant >= 0) {
        discriminant = std::sqrt(discriminant);
        double t1 = (-b - discriminant) / (2.0 * a);
        double t2 = (-b + discriminant) / (2.0 * a);

        // 선분 내부에 있는 점만 필터링 (0 <= t <= 1)
        if (t1 >= 0.0 && t1 <= 1.0) {
            Eigen::Vector2d intersect = p1 + t1 * d;
            set_points.push_back({intersect.x(), intersect.y(), index});
        }
        // 중복 해 방지
        if (std::abs(t1 - t2) > 1e-9 && t2 >= 0.0 && t2 <= 1.0) {
            Eigen::Vector2d intersect = p1 + t2 * d;
            set_points.push_back({intersect.x(), intersect.y(), index});
        }
    }

    return set_points;
}

Intersection AgentLogic::choose_intersection_randomly(const std::vector<Intersection>& intersections) {
    if (intersections.empty()) {
        throw std::runtime_error("The input collection of points is empty.");
    }
    
    // C++11 표준 난수 생성기 사용
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, intersections.size() - 1);
    
    return intersections[dis(gen)];
}

double AgentLogic::calculate_distance(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) {
    return (p2 - p1).norm();
}

double AgentLogic::e_calculate_distance(
    const Eigen::VectorXd& line,
    const Eigen::MatrixXd& bound_point,
    const Eigen::MatrixXd& move_point) 
{
    Eigen::Vector2d start = (line(0) > -0.9) ? 
        bound_point.row(static_cast<int>(line(0))).transpose() : 
        move_point.row(static_cast<int>(line(1))).transpose();

    Eigen::Vector2d end = (line(2) > -0.9) ? 
        bound_point.row(static_cast<int>(line(2))).transpose() : 
        move_point.row(static_cast<int>(line(3))).transpose();

    return (end - start).norm();
}
