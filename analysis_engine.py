import numpy as np
from scipy.sparse import csr_matrix, eye
from scipy.sparse.linalg import spsolve
import pandas as pd



# --- Function to calculate bulk & shear moduli: 100% Analytical Derivative Implementation ---
def analyze_stability_and_moduli(move_point, bound_point, e_mat, C_k, C_T, area):
    num_movable = move_point.shape[0]
    rows, cols, data = [], [], []
    Xi_shear = np.zeros(2 * num_movable)
    Xi_bulk = np.zeros(2 * num_movable)
    G_aff_sum = 0.0
    K_aff_sum = 0.0

    for i in range(e_mat.shape[0]):
        # Identify node indices
        is_p1_m = e_mat[i, 0] < -0.9
        is_p2_m = e_mat[i, 2] < -0.9
        idx1 = int(e_mat[i, 1]) if is_p1_m else -1
        idx2 = int(e_mat[i, 3]) if is_p2_m else -1
        
        # Get coordinates
        p1 = move_point[idx1] if is_p1_m else bound_point[int(e_mat[i, 0])]
        p2 = move_point[idx2] if is_p2_m else bound_point[int(e_mat[i, 2])]
        
        # --- Analytical Variables from Proof ---
        xi, yi = p1[0], p1[1]
        xj, yj = p2[0], p2[1]
        
        dx = xj - xi
        dy = yj - yi
        D = dx**2 + dy**2        # Squared Distance
        L = np.sqrt(D)           # Current Length
        L0 = e_mat[i, 5]         # Rest Length l0
        
        # Physical Coefficients
        k_eff = C_k / L0
        t_L = (C_T / L0) * (1.0 - L0 / L)
        
        # --- 1. Analytical Hessian (Second Derivatives) ---
        # Matches Eq. 11 & 12 in your LaTeX proof
        Hxx = ((k_eff - t_L) * dx**2 / D) + t_L
        Hyy = ((k_eff - t_L) * dy**2 / D) + t_L
        Hxy = (k_eff - t_L) * (dx * dy) / D
        
        # Assemble Global Hessian (Sparse)
        # Internal nodes connected to each other or boundary
        for idx in [idx1, idx2]:
            if idx >= 0:
                rows.extend([2*idx, 2*idx+1, 2*idx, 2*idx+1])
                cols.extend([2*idx, 2*idx+1, 2*idx+1, 2*idx])
                data.extend([Hxx, Hyy, Hxy, Hxy])
        
        if idx1 >= 0 and idx2 >= 0:
            # Interaction terms (-H)
            rows.extend([2*idx1, 2*idx1+1, 2*idx1, 2*idx1+1, 2*idx2, 2*idx2+1, 2*idx2, 2*idx2+1])
            cols.extend([2*idx2, 2*idx2+1, 2*idx2+1, 2*idx2, 2*idx1, 2*idx1+1, 2*idx1+1, 2*idx1])
            data.extend([-Hxx, -Hyy, -Hxy, -Hxy, -Hxx, -Hyy, -Hxy, -Hxy])

        # --- 2. Analytical Affine Forces (Xi = First Derivative of Force w.r.t Strain) ---
        # Bulk Case: Simplified analytically to k_eff * dr
        xi_bx = k_eff * dx
        xi_by = k_eff * dy
        
        # Shear Case: Explicit coordinate expansion from Eq. 15
        xi_sx = Hxx * dx + Hxy * (-dy)
        xi_sy = Hxy * dx + Hyy * (-dy)

        # Distribute forces to nodes
        if idx1 >= 0:
            Xi_shear[2*idx1] -= xi_sx; Xi_shear[2*idx1+1] -= xi_sy
            Xi_bulk[2*idx1] -= xi_bx;  Xi_bulk[2*idx1+1] -= xi_by
        if idx2 >= 0:
            Xi_shear[2*idx2] += xi_sx; Xi_shear[2*idx2+1] += xi_sy
            Xi_bulk[2*idx2] += xi_bx;  Xi_bulk[2*idx2+1] += xi_by

        # --- 3. Analytical Affine Moduli (Curvature) ---
        # G_aff = (1/2A) * sum( v_aff_shear \cdot Xi_shear )
        G_aff_sum += dx * xi_sx + (-dy) * xi_sy
        # K_aff = (1/2A) * sum( v_aff_bulk \cdot Xi_bulk )
        K_aff_sum += dx * xi_bx + dy * xi_by

    # Solve for Non-affine relaxations (u = H^-1 * Xi)
    H_sparse = csr_matrix((data, (rows, cols)), shape=(2*num_movable, 2*num_movable))
    H_reg = H_sparse + eye(2*num_movable) * 1e-11 # Regularization for stability
    u_s = spsolve(H_reg, Xi_shear)
    u_b = spsolve(H_reg, Xi_bulk)

    # Final Moduli Normalization (using the 1/2A factor from your proof)
    denom = 2.0 * area
    G_A = G_aff_sum / denom
    G_NA = np.dot(Xi_shear, u_s) / denom
    K_A = K_aff_sum / denom
    K_NA = np.dot(Xi_bulk, u_b) / denom

    return {
        'G_total': G_A - G_NA, 'K_total': K_A - K_NA,
        'G_A': G_A, 'G_NA': G_NA,
        'K_A': K_A, 'K_NA': K_NA
    }

