import numpy as np
import pandas as pd
import os
from analysis_engine import analyze_stability_and_moduli

# --- 1. Physics Parameters & Constants ---
# (Note: These should match your simulation settings in main.py)
RADIUS = 0.3
K_SP = 1.0
STRAIN_ORIGINAL = 1.0  # Reference strain for your specific model
STRAIN_NEW = 1.0
AREA = 1.0             # System Area (L x W)

# Derived model constants based on your Physica proof
C_VAL_K = K_SP * RADIUS * (1 / (1 + 0.01 * STRAIN_ORIGINAL))
C_VAL_T = K_SP * RADIUS * (STRAIN_NEW / STRAIN_ORIGINAL)

def run_moduli_analysis(ensemble_size=40):
    """
    Loads generated CSV files and calculates G and K moduli for each sample.
    """
    raw_data = []
    print(f"🚀 Starting Mechanical Analysis for {ensemble_size} samples...")

    for idx in range(ensemble_size):
        idx_str = f'{idx:03d}'
        try:
            # Load the network topology and geometry
            # Ensure these files exist in the same directory or provide full path
            e_matrix = np.loadtxt(f'{idx_str}_e_matrix.csv', delimiter=',')
            node_bound = np.loadtxt(f'{idx_str}_node_bound.csv', delimiter=',')
            node_movable = np.loadtxt(f'{idx_str}_node.csv', delimiter=',')

            # Core analytical calculation
            results = analyze_stability_and_moduli(
                node_movable, 
                node_bound, 
                e_matrix, 
                C_VAL_K, 
                C_VAL_T, 
                AREA
            )
            
            raw_data.append({'Sample': idx_str, **results})
            print(f"✅ Sample {idx_str}: Calculated successfully.")

        except OSError:
            print(f"⚠️ Sample {idx_str}: Files not found. Skipping...")
        except Exception as e:
            print(f"❌ Error in Sample {idx_str}: {e}")
            continue

    if not raw_data:
        print("Empty dataset. Check if .csv files are in the current directory.")
        return

    # --- 2. Post-Processing & Statistics ---
    df = pd.DataFrame(raw_data)
    numeric_cols = ['G_total', 'K_total', 'G_A', 'G_NA', 'K_A', 'K_NA']
    
    # Calculate Mean and Standard Deviation
    stats_summary = {
        'Sample': ['AVERAGE', 'STD_DEV'],
        **{col: [df[col].mean(), df[col].std()] for col in numeric_cols}
    }
    stats_df = pd.DataFrame(stats_summary)

    # Combine raw data with statistics
    df_final = pd.concat([df, stats_df], ignore_index=True)

    # --- 3. Export Results ---
    output_filename = 'moduli_final_report.csv'
    df_final.to_csv(output_filename, index=False)
    
    print("\n" + "="*40)
    print("       ANALYSIS SUMMARY REPORT")
    print("="*40)
    print(stats_df.to_string(index=False))
    print("="*40)
    print(f"Full report saved to: {output_filename}")

if __name__ == "__main__":
    run_moduli_analysis(ensemble_size=40)
