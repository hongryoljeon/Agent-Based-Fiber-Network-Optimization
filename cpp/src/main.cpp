#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <random>
#include <algorithm>
#include <ctime>
#include <Eigen/Dense>
#include <LBFGS.h>
#include "PhysicsEngine.hpp"
#include "AgentLogic.hpp"
#include "Utils.hpp"

using namespace Eigen;
using namespace LBFGSpp;

// L-BFGS Objective Function Wrapper
class FiberObjective {
    const MatrixXd &b, &e; double C;
public:
    FiberObjective(const MatrixXd& b, const MatrixXd& e, double c) : b(b), e(e), C(c) {}
    double operator()(const VectorXd& x, VectorXd& grad) {
        grad = PhysicsEngine::calculate_gradient(x, b, e, C);
        return PhysicsEngine::calculate_energy(x, b, e, C);
    }
};

int main() {
    // --- 시스템 시작 체크 ---
    std::cout << "Agent-Based Fiber Network Optimization Framework" << std::endl;
    std::cout << "--- Initializing Simulation ---" << std::endl << std::flush;

    // --- 1. Parameters ---
    const double radius = 0.3;
    const int num_simulations = 51;
    const int num_agents = 4;
    const double k_sp = 1.0;
    const double ten_percent = 3.0;
    const double ten = 1.0 / (1.0 + 0.01 * ten_percent);
    const double thres = 1e-8; 
    const double thr = 0.01;   
    const double GTOL = 1e-9;
    const int ensemble_size = 40;
    const double C_val = 0.009;

    std::cout << "DEBUG: ensemble_size = " << ensemble_size << ", num_simulations = " << num_simulations << std::endl << std::flush;

    std::mt19937 gen(static_cast<unsigned int>(time(0)));
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    for (int en = 0; en < ensemble_size; ++en) {
        std::cout << ">>> Start Ensemble: " << en << std::endl << std::flush;

        // Initial State
        std::vector<Vector2d> nodes_b = {{0,0}, {1,0}, {0,1}, {1,1}};
        std::vector<Vector2d> nodes_m;
        std::vector<VectorXd> edges;
        
        for(int i=0; i<4; ++i) {
            VectorXd e_init(6);
            if(i==0) e_init << 0,-1,1,-1,0,1;
            else if(i==1) e_init << 0,-1,2,-1,0,1;
            else if(i==2) e_init << 1,-1,3,-1,0,1;
            else e_init << 2,-1,3,-1,0,1;
            edges.push_back(e_init);
        }

        std::vector<int> new_lin_nums = {0, 3, 1, 2};

        int y = 0;
        while (y < num_simulations) {
            std::cout << "Ensemble: " << en << " | Step: " << y << std::endl << std::flush;

            // mapo/mapx tracking initialization for each step
            std::vector<int> mapo(num_agents, 1);
            std::vector<int> mapx(num_agents, 0);
            std::vector<Intersection> node_intersection_points;

            while (true) { // Structural Validity Search
                auto b_keep = nodes_b; auto m_keep = nodes_m; 
                auto e_keep = edges; auto agent_keep = new_lin_nums;
                
                int p = (int)nodes_m.size();
                int t = 0;
                node_intersection_points.clear();
                std::vector<VectorXd> new_sum_row;
                bool rollback_step = false;

                // Agent Movement Logic
                for (int n = 0; n < num_agents; ++n) {
                    bool agent_moved = false;
                    int attempts = 0;
                    while (!agent_moved && attempts < 100) {
                        attempts++;
                        int lin_num = new_lin_nums[n];
                        Vector2d p_s = (edges[lin_num](0) > -0.9) ? nodes_b[(int)edges[lin_num](0)] : nodes_m[(int)edges[lin_num](1)];
                        Vector2d p_e = (edges[lin_num](2) > -0.9) ? nodes_b[(int)edges[lin_num](2)] : nodes_m[(int)edges[lin_num](3)];

                        double t_lin = dist(gen);
                        Vector2d h_k = (1.0 - t_lin) * p_s + t_lin * p_e;

                        std::vector<Intersection> intersects;
                        MatrixXd b_tmp(nodes_b.size(), 2); for(int k=0; k<(int)nodes_b.size(); ++k) b_tmp.row(k)=nodes_b[k];
                        MatrixXd m_tmp(nodes_m.size(), 2); for(int k=0; k<(int)nodes_m.size(); ++k) m_tmp.row(k)=nodes_m[k];

                        for(int j=0; j < (int)edges.size(); ++j) {
                            if(j != lin_num) AgentLogic::find_line_circle_intersections(b_tmp, m_tmp, edges[j], h_k, radius, intersects, j);
                        }

                        if(intersects.empty()) continue; 

                        Intersection chosen = AgentLogic::choose_intersection_randomly(intersects);
                        Vector2d p_chosen(chosen.x, chosen.y);

                        if ((p_chosen - h_k).norm() < radius * thr) {
                            std::vector<double> weights;
                            for(auto& eg : edges) weights.push_back(AgentLogic::e_calculate_distance(eg, b_tmp, m_tmp));
                            std::discrete_distribution<int> weight_dist(weights.begin(), weights.end());
                            new_lin_nums[n] = weight_dist(gen);
                            continue;
                        }

                        VectorXd nr(6); nr << -1, nodes_m.size(), -1, nodes_m.size()+1, k_sp, ten * radius;
                        nodes_m.push_back(h_k); nodes_m.push_back(p_chosen);
                        
                        node_intersection_points.push_back({h_k.x(), h_k.y(), -10, lin_num, t++});
                        node_intersection_points.push_back({p_chosen.x(), p_chosen.y(), -10, (int)chosen.index, t++});

                        new_lin_nums[n] = (int)chosen.index;
                        new_sum_row.push_back(nr);
                        agent_moved = true;
                    }
                }

                for(auto& nr : new_sum_row) edges.push_back(nr);

                // Split & Delete Logic
                if (!node_intersection_points.empty()) {
                    std::vector<int> delete_array;
                    std::vector<VectorXd> new_line_list;

                    for (int i = 0; i < (int)edges.size(); ++i) {
                        std::vector<Intersection> filt;
                        for(auto& nip : node_intersection_points) {
                            if(nip.index == i) filt.push_back(nip);
                        }

                        for(int n=0; n<num_agents; ++n) {
                            if (i == new_lin_nums[n]) mapo[n] = 1 + (int)filt.size();
                            else if (i > new_lin_nums[n]) mapx[n] += 1 + (int)filt.size();
                        }

                        if (!filt.empty()) {
                            delete_array.push_back(i);
                            Vector2d ref = (edges[i](0) > -0.9) ? nodes_b[(int)edges[i](0)] : nodes_m[(int)edges[i](1)];
                            Vector2d end_pt = (edges[i](2) > -0.9) ? nodes_b[(int)edges[i](2)] : nodes_m[(int)edges[i](3)];
                            std::sort(filt.begin(), filt.end(), [&](const Intersection& a, const Intersection& b) {
                                return (Vector2d(a.x, a.y)-ref).norm() < (Vector2d(b.x, b.y)-ref).norm();
                            });

                            double l_ori = (end_pt - ref).norm();
                            double r0 = l_ori / (nodes_m[p + filt[0].t_id] - ref).norm();
                            VectorXd s1(6); s1 << edges[i](0), edges[i](1), -1, p + filt[0].t_id, edges[i](4)*r0, edges[i](5)/r0;
                            new_line_list.push_back(s1);

                            for(size_t q=0; q < filt.size()-1; ++q) {
                                double rq = l_ori / (nodes_m[p + filt[q+1].t_id] - nodes_m[p + filt[q].t_id]).norm();
                                VectorXd sq(6); sq << -1, p + filt[q].t_id, -1, p + filt[q+1].t_id, edges[i](4)*rq, edges[i](5)/rq;
                                new_line_list.push_back(sq);
                            }

                            double rf = l_ori / (end_pt - nodes_m[p + filt.back().t_id]).norm();
                            VectorXd sf(6); sf << -1, p + filt.back().t_id, edges[i](2), edges[i](3), edges[i](4)*rf, edges[i](5)/rf;
                            new_line_list.push_back(sf);
                        }
                    }

                    std::sort(delete_array.rbegin(), delete_array.rend());
                    for(int idx : delete_array) edges.erase(edges.begin() + idx);
                    for(auto& nl : new_line_list) edges.push_back(nl);

                    MatrixXd b_f(nodes_b.size(), 2); for(int k=0; k<(int)nodes_b.size(); ++k) b_f.row(k)=nodes_b[k];
                    MatrixXd m_f(nodes_m.size(), 2); for(int k=0; k<(int)nodes_m.size(); ++k) m_f.row(k)=nodes_m[k];
                    for(auto& nl : new_line_list) {
                        if (AgentLogic::e_calculate_distance(nl, b_f, m_f) < radius * thres) { rollback_step = true; break; }
                    }

                    if (rollback_step) {
                        nodes_b = b_keep; nodes_m = m_keep; edges = e_keep; new_lin_nums = agent_keep;
                        continue;
                    }
                }
                break;
            }

            // Optimization
            if (!nodes_m.empty()) {
                MatrixXd b_opt(nodes_b.size(), 2); for(int i=0; i<(int)nodes_b.size(); ++i) b_opt.row(i) = nodes_b[i];
                MatrixXd e_opt(edges.size(), 6); for(int i=0; i<(int)edges.size(); ++i) e_opt.row(i) = edges[i];
                VectorXd x(nodes_m.size() * 2); for(int i=0; i<(int)nodes_m.size(); ++i) x.segment<2>(i*2) = nodes_m[i];

                LBFGSParam<double> param; param.epsilon = GTOL;
                LBFGSSolver<double> solver(param);
                FiberObjective obj(b_opt, e_opt, C_val);
                double final_e;
                try { solver.minimize(obj, x, final_e); } catch (...) {}
                for(int i=0; i<(int)nodes_m.size(); ++i) nodes_m[i] = x.segment<2>(i*2);
            }

            // Agent Position Update
            for(int n=0; n<num_agents; ++n) {
                if (!node_intersection_points.empty() && new_lin_nums[n] > 3) {
                    std::vector<int> indices;
                    int start_idx = (int)edges.size()-mapx[n]-1;
                    int end_idx = (int)edges.size()-mapo[n]-mapx[n];
                    for(int idx = start_idx; idx >= end_idx; --idx) {
                        if(idx >= 0 && idx < (int)edges.size()) indices.push_back(idx);
                    }
                    if(!indices.empty()){
                        std::vector<double> probs;
                        MatrixXd b_m(nodes_b.size(), 2); for(int k=0; k<(int)nodes_b.size(); ++k) b_m.row(k)=nodes_b[k];
                        MatrixXd m_m(nodes_m.size(), 2); for(int k=0; k<(int)nodes_m.size(); ++k) m_m.row(k)=nodes_m[k];
                        for(int idx : indices) probs.push_back(AgentLogic::e_calculate_distance(edges[idx], b_m, m_m));
                        std::discrete_distribution<int> d(probs.begin(), probs.end());
                        new_lin_nums[n] = indices[d(gen)];
                    }
                }
            }

            if (y % 5 == 0) {
                MatrixXd b_s(nodes_b.size(), 2); for(int k=0; k<(int)nodes_b.size(); ++k) b_s.row(k)=nodes_b[k];
                MatrixXd m_s(nodes_m.size(), 2); for(int k=0; k<(int)nodes_m.size(); ++k) m_s.row(k)=nodes_m[k];
                MatrixXd e_s(edges.size(), 6); for(int i=0; i<(int)edges.size(); ++i) e_s.row(i)=edges[i];
                Utils::save_simulation_data(en, y, "results_cpp", m_s, b_s, e_s);
            }
            y++;
        }
    }
    std::cout << "--- Simulation Finished Successfully ---" << std::endl;
    return 0;
}
