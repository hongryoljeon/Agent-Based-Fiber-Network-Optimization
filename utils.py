import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import os
import numpy as np
from matplotlib.backends.backend_pdf import PdfPages

def get_color(value):
    if value < 0: return 'red'
    elif value < 0.01: return 'orange'
    elif value < 0.02: return 'green'
    elif value < 0.03: return 'blue'
    else: return 'purple'


def save_simulation_data(en, sim_num, folder_name, node, node_bound, e):
    original_dir = os.getcwd()
    os.makedirs(folder_name, exist_ok=True)
    os.chdir(folder_name)
    
    np.savetxt(f"{en:03d}_e_matrix.csv", e, delimiter=",")
    np.savetxt(f"{en:03d}_node.csv", np.array(node), delimiter=",")
    np.savetxt(f"{en:03d}_node_bound.csv", np.array(node_bound), delimiter=",")
    
    os.chdir(original_dir)


def check_convergence(res, final_grad, threshold=1e-7):
    """
    check convergence of force and return the maximum value of the force
    """
    max_force = np.max(np.abs(final_grad))
    mean_force = np.mean(np.abs(final_grad))
    
    print("-" * 30)
    print(f"Optimization Status: {'SUCCESS' if res.success else 'FAILED'}")
    print(f"Message: {res.message}")
    print(f"Final Energy: {res.fun:.6e}")
    print(f"Maximum Force (inf-norm): {max_force:.2e}")
    print(f"Average Force: {mean_force:.2e}")
    print(f"Number of Iterations: {res.nit}")
    
    if max_force > threshold:
        print(f"⚠️ WARNING: Convergence threshold ({threshold}) not met!")
    else:
        print(f"✅ Success: System reached mechanical equilibrium.")
    print("-" * 30)
    
    return max_force
