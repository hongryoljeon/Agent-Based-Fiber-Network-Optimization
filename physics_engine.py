import numpy as np
from numba import jit

@jit(nopython=True)
def energy_numba(move_point, bound_point, e_matrix, C_val):
    total_energy = 0.0
    compensation = 0.0
    for i in range(e_matrix.shape[0]):
        # 시작점 설정
        if e_matrix[i, 0] > -0.9:
            p1 = bound_point[int(e_matrix[i, 0])]
        else:
            p1 = move_point[int(e_matrix[i, 1])]
        # 끝점 설정
        if e_matrix[i, 2] > -0.9:
            p2 = bound_point[int(e_matrix[i, 2])]
        else:
            p2 = move_point[int(e_matrix[i, 3])]
            
        dx, dy = p2[0] - p1[0], p2[1] - p1[1]
        length = np.sqrt(dx**2 + dy**2)
        rest_length = e_matrix[i, 5]
        strain = (length / rest_length) - 1.0
        
        # Kahan summation for numerical stability
        increment = 0.5 * rest_length * (strain**2)
        y_val = increment - compensation
        t = total_energy + y_val
        compensation = (t - total_energy) - y_val
        total_energy = t
        
    return total_energy * C_val

@jit(nopython=True)
def gradient_numba(move_point, bound_point, e_matrix, C_val):
    num_movable = move_point.shape[0]
    grad = np.zeros((num_movable, 2))
    for i in range(e_matrix.shape[0]):
        is_start_movable = e_matrix[i, 0] < -0.9
        is_end_movable = e_matrix[i, 2] < -0.9
        p1 = move_point[int(e_matrix[i, 1])] if is_start_movable else bound_point[int(e_matrix[i, 0])]
        p2 = move_point[int(e_matrix[i, 3])] if is_end_movable else bound_point[int(e_matrix[i, 2])]
        
        dx, dy = p2[0] - p1[0], p2[1] - p1[1]
        L = np.sqrt(dx**2 + dy**2)
        if L < 1e-14: continue
        L0 = e_matrix[i, 5]
        coeff = C_val * ((L / L0) - 1.0) / L
        
        if is_start_movable:
            idx1 = int(e_matrix[i, 1])
            grad[idx1, 0] -= coeff * dx
            grad[idx1, 1] -= coeff * dy
        if is_end_movable:
            idx2 = int(e_matrix[i, 3])
            grad[idx2, 0] += coeff * dx
            grad[idx2, 1] += coeff * dy
            
    return grad.flatten()


# physics_engine.py 파일 하단에 추가

def energy_wrapper(flat_node, node_bound_arr, e_matrix, C_val):
    """scipy.optimize를 위한 Energy 함수 래퍼"""
    # 1D 배열로 들어오는 노드 좌표를 다시 (N, 2) 형태로 복원해서 전달합니다.
    return energy_numba(flat_node.reshape(-1, 2), node_bound_arr, e_matrix, C_val)

def gradient_wrapper(flat_node, node_bound_arr, e_matrix, C_val):
    """scipy.optimize를 위한 Gradient 함수 래퍼"""
    return gradient_numba(flat_node.reshape(-1, 2), node_bound_arr, e_matrix, C_val)
