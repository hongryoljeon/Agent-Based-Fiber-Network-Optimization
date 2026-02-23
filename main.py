import numpy as np
import random
import time
import os
import math
from scipy.optimize import minimize
from physics_engine import energy_wrapper, gradient_wrapper, gradient_numba
from agent_logic import line_circle_intersection, choose_intersection_randomly, calculate_distance, e_calculate_distance
from utils import get_color, save_simulation_data
from shapely.geometry import LineString
from matplotlib.backends.backend_pdf import PdfPages
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import yaml
import argparse

# --- Parameters ---

def load_config(path):
    if not os.path.exists(path):
        raise FileNotFoundError(f"Config file not found: {path}")
    with open(path, "r") as f:
        return yaml.safe_load(f)



def run_simulation(config):
    radius = config["radius"]
    num_simulations = config["num_simulations"]
    num_agents = config["num_agents"]
    k_sp = config["k_sp"]
    ten_percent = config["ten_percent"]
    thres = config["thres"]
    thr = config["thr"]
    GTOL = config["GTOL"]
    ensemble_size = config["ensemble_size"]
    C = config["C"]
    ten = 1 / (1 + 0.01 * ten_percent)
    
    original_dir = os.getcwd()

    for en in range(ensemble_size):
        # Initial setting before construction
        e = np.array([
            [0, -1, 1, -1, 0, 1],
            [0, -1, 2, -1, 0, 1],
            [1, -1, 3, -1, 0, 1],
            [2, -1, 3, -1, 0, 1]
        ], dtype=float)

        node_bound = [(0, 0), (1, 0), (0, 1), (1, 1)]
        node = []
        new_lin_nums = [0, 3, 1, 2]

        y = 0
        while y < num_simulations:
            print(f"Ensemble: {en} | Simulation Step: {y}")
            while True:
                start_time = time.time()
                new_sum_row = []
                p = len(node)
                t = 0
                node_intersection_points = []

                node_keep = list(node)
                node_bound_keep = list(node_bound)
                e_keep = np.copy(e)

                for n in range(num_agents):
                    while True:
                        index_prob1 = 0
                        lin_num = new_lin_nums[n]
                        t_lin = random.uniform(0, 1)

                        start_point = node_bound[int(e[lin_num, 0])] if e[lin_num, 0] > -0.9 else node[int(e[lin_num, 1])]
                        end_point = node_bound[int(e[lin_num, 2])] if e[lin_num, 2] > -0.9 else node[int(e[lin_num, 3])]

                        h, k = (1 - t_lin) * start_point[0] + t_lin * end_point[0], (1 - t_lin) * start_point[1] + t_lin * end_point[1]

                        intersection_points = []
                        for j in range(len(e)):
                            if j != lin_num:
                                intersection_points = line_circle_intersection(node_bound, node, e[j, :], (h, k), radius, intersection_points, j)

                        if not intersection_points:
                            while not intersection_points:
                                t_lin = random.uniform(0, 1)
                                h, k = (1 - t_lin) * start_point[0] + t_lin * end_point[0], (1 - t_lin) * start_point[1] + t_lin * end_point[1]
                                for j in range(len(e)):
                                    if j != lin_num:
                                        intersection_points = line_circle_intersection(node_bound, node, e[j, :], (h, k), radius, intersection_points, j)

                        chosen_point = choose_intersection_randomly(intersection_points)
                        line = LineString([(h, k), (chosen_point[0], chosen_point[1])])
                        
                        vision_node, vision_new_line = [], []
                        for i in range(4, len(e)):
                            p1 = node_bound[int(e[i, 0])] if e[i, 0] > -0.9 else node[int(e[i, 1])]
                            p2 = node_bound[int(e[i, 2])] if e[i, 2] > -0.9 else node[int(e[i, 3])]
                            intersect = line.intersection(LineString([p1, p2]))
                            if intersect.geom_type == "Point":
                                if all(math.sqrt((nx - intersect.x)**2 + (ny - intersect.y)**2) > 1e-8 for nx, ny in node):
                                    vision_node.append((intersect.x, intersect.y, i))

                        if vision_node:
                            sorted_vision = sorted(vision_node, key=lambda pt: math.sqrt((pt[0] - h)**2 + (pt[1] - k)**2))
                            vision_new_line.append((h, k, sorted_vision[0][0], sorted_vision[0][1]))
                            for q in range(len(sorted_vision)-1):
                                vision_new_line.append((sorted_vision[q][0], sorted_vision[q][1], sorted_vision[q+1][0], sorted_vision[q+1][1]))
                            vision_new_line.append((sorted_vision[-1][0], sorted_vision[-1][1], chosen_point[0], chosen_point[1]))

                        if any(math.sqrt((l[0]-l[2])**2 + (l[1]-l[3])**2) < radius*thr for l in vision_new_line):
                            weights = [e_calculate_distance(e[v, :], node_bound, node) for v in range(len(e))]
                            new_lin_nums[n] = np.random.choice(len(e), p=weights/np.sum(weights))
                            continue

                        new_row = [-1, -1, -1, -1, k_sp, ten * radius]
                        if lin_num < 4:
                            new_row[0] = len(node_bound); node_bound.append((h, k))
                        else:
                            new_row[1] = len(node); node.append((h, k))
                            node_intersection_points.append((h, k, -10, lin_num, t)); t += 1

                        new_lin_nums[n] = chosen_point[2]
                        if chosen_point[2] < 4:
                            new_row[2] = len(node_bound); node_bound.append((chosen_point[0], chosen_point[1]))
                        else:
                            new_row[3] = len(node); node.append((chosen_point[0], chosen_point[1]))
                            node_intersection_points.append((chosen_point[0], chosen_point[1], -10, chosen_point[2], t)); t += 1

                        if n == 0: new_sum_row = new_row
                        else: new_sum_row = np.vstack((new_sum_row, new_row))
                        break

                e = np.vstack((e, new_sum_row))
                
                # Intersection handling between agents' lines
                for i in range(num_agents):
                    l1_idx = len(e)-1-i
                    p1s = node_bound[int(e[l1_idx,0])] if e[l1_idx,0]>-0.9 else node[int(e[l1_idx,1])]
                    p1e = node_bound[int(e[l1_idx,2])] if e[l1_idx,2]>-0.9 else node[int(e[l1_idx,3])]
                    l1 = LineString([p1s, p1e])
                    for j in range(4, l1_idx):
                        p2s = node_bound[int(e[j,0])] if e[j,0]>-0.9 else node[int(e[j,1])]
                        p2e = node_bound[int(e[j,2])] if e[j,2]>-0.9 else node[int(e[j,3])]
                        intersect = l1.intersection(LineString([p2s, p2e]))
                        if intersect.geom_type == "Point" and all(math.sqrt((nx-intersect.x)**2+(ny-intersect.y)**2)>1e-8 for nx,ny in node):
                            node_intersection_points.append((intersect.x, intersect.y, l1_idx, j, t))
                            node.append((intersect.x, intersect.y)); t += 1

                if node_intersection_points:
                    mapo, mapx, delete_array, new_line_list = [1]*num_agents, [0]*num_agents, [], []
                    for i in range(4, len(e)):
                        arr = np.array(node_intersection_points)
                        filt = arr[arr[:,3]==i] if i < len(e)-num_agents else arr[(arr[:,3]==i)|(arr[:,2]==i)]
                        for n in range(num_agents):
                            if i == new_lin_nums[n]: mapo[n] = 1 + len(filt)
                            elif i > new_lin_nums[n]: mapx[n] += 1 + len(filt)

                        if len(filt) > 0:
                            delete_array.append(i)
                            ref = node_bound[int(e[i,0])] if e[i,0]>-0.9 else node[int(e[i,1])]
                            sorted_pts = sorted(filt, key=lambda pt: math.sqrt((pt[0]-ref[0])**2+(pt[1]-ref[1])**2))
                            end_pt = node_bound[int(e[i,2])] if e[i,2]>-0.9 else node[int(e[i,3])]
                            
                            l_ori = math.sqrt((end_pt[0]-ref[0])**2+(end_pt[1]-ref[1])**2)
                            p0 = node[p+int(sorted_pts[0][4])]
                            r0 = l_ori/math.sqrt((p0[0]-ref[0])**2+(p0[1]-ref[1])**2)
                            new_line_list.append((e[i,0], e[i,1], -1, sorted_pts[0][4]+p, e[i,4]*r0, e[i,5]/r0))
                            
                            for q in range(len(sorted_pts)-1):
                                pq, pq1 = node[p+int(sorted_pts[q][4])], node[p+int(sorted_pts[q+1][4])]
                                rq = l_ori/math.sqrt((pq[0]-pq1[0])**2+(pq[1]-pq1[1])**2)
                                new_line_list.append((-1, sorted_pts[q][4]+p, -1, sorted_pts[q+1][4]+p, e[i,4]*rq, e[i,5]/rq))
                            
                            pf = node[p+int(sorted_pts[-1][4])]
                            rf = l_ori/math.sqrt((pf[0]-end_pt[0])**2+(pf[1]-end_pt[1])**2)
                            new_line_list.append((-1, sorted_pts[-1][4]+p, e[i,2], e[i,3], e[i,4]*rf, e[i,5]/rf))

                    e = np.delete(e, delete_array, axis=0)
                    if new_line_list:
                        e = np.vstack((e, np.array(new_line_list)))
                        if any(e_calculate_distance(nl, node_bound, node) < radius*thres for nl in new_line_list):
                            e, node, node_bound = e_keep, node_keep, node_bound_keep
                            weights = [e_calculate_distance(e[v,:], node_bound, node) for v in range(len(e))]
                            for n in range(num_agents): new_lin_nums[n] = np.random.choice(len(e), p=weights/np.sum(weights))
                            continue
                break

            # --- Energy Minimization ---
            if node:
                res = minimize(energy_wrapper, np.array(node).flatten(), args=(np.array(node_bound), e, C),
                               method='L-BFGS-B', jac=gradient_wrapper, options={'gtol': GTOL, 'maxiter': 2000000})
                node = res.x.reshape(-1, 2).tolist()
                # 2. 최종 그래디언트 계산 (수렴 확인용)
                node_bound_arr = np.array(node_bound)
                final_grad = gradient_numba(res.x.reshape(-1, 2), node_bound_arr, e, C)
    
                # 3. utils의 함수 호출
                from utils import check_convergence
                max_f = check_convergence(res, final_grad, threshold=GTOL)

            # --- Agent Position Update ---
            for n in range(num_agents):
                if len(node_intersection_points) > 0 and new_lin_nums[n] > 3:
                    indices = np.arange(len(e)-mapx[n]-1, len(e)-mapo[n]-mapx[n]-1, -1)
                    probs = [e_calculate_distance(e[idx,:], node_bound, node) for idx in indices]
                    new_lin_nums[n] = np.random.choice(indices, p=probs/np.sum(probs))

            # --- Visualization & Save ---
            if y % 5 == 0:
                save_simulation_data(en, y, f"Limit_Mechanics_{ten_percent:.2f}%_r{radius:.2f}_thr{thr:.4f}", node, node_bound, e)
            y += 1


def parse_args():
    parser = argparse.ArgumentParser(description="Agent-Based Fiber Network Simulation")
    parser.add_argument(
        "--config",
        type=str,
        default="config.yaml",
        help="Path to configuration YAML file"
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    config = load_config(args.config)
    run_simulation(config)
