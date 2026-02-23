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
