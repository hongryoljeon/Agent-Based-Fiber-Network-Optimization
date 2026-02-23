# Agent-Based Fiber Network Optimization

## Overview
This project develops a modular simulation and optimization framework for studying how local agent-based deposition rules give rise to emergent global mechanical behavior in fiber networks.

The system combines analytical mechanics, sparse numerical optimization, and agent-based construction to investigate how micro-scale rules program macro-scale elastic properties.

**Core Question:** Can purely local deposition rules be optimized to control global stiffness and mechanical response?



---

## Motivation
Biological and soft material systems often exhibit complex global behavior driven by simple local rules. Understanding this micro-to-macro mapping is critical for:

- Programmable soft material design
- Structural optimization under geometric constraints
- Emergent mechanics in stochastic networks

This framework explores how agent parameters:

- Radius of reach(R)
- Limiting segment(LS) 
- Preset strain(P) 


influence global properties such as:

- Stress distribution
- Bulk modulus (K)
- Shear modulus (G)


---

## 📊 Scientific Validation & Analysis

The following results demonstrate the relationship between agent-defined parameters and the resulting network's elastic moduli, averaged over **40 ensemble simulations** to ensure statistical significance.

### 1. Influence of Limiting Segment Length (LS)

* **Observation:** Increasing the $LS$ value leads to a structural transition in the network. We observe a sensitive region where the global moduli ($G, K$) shift, indicating a trade-off between local agent connectivity and global network stiffness.
  <img width="2958" height="2048" alt="image" src="https://github.com/user-attachments/assets/d6df07b8-0da9-4bf7-95f7-4b52b016d940" />


### 2. Influence of Preset Strain (P)

* **Observation:** The moduli show a clear monotonic scaling with the $P$ value. This confirms that the preset strain acts as a primary reinforcement mechanism, effectively stiffening the emergent fiber network.
  <img width="2958" height="2048" alt="image" src="https://github.com/user-attachments/assets/a551427a-89d3-4fc9-9867-be85a5c414b1" />


---

## System Architecture

The total energy is defined as:

E_total = E_elastic + Phi_constraints

The optimization pipeline is modular:

1. Agent Construction – Local rule-based fiber deposition

2. Energy Assembly – Aggregation of elastic and constraint potentials

3. Gradient/Hessian Module – Analytical sparse derivative computation

4. Optimization Engine – L-BFGS-B constrained minimization

5. Mechanical Evaluation – Virtual deformation to extract G and K


The design emphasizes separation of physics, optimization logic, and numerical backend.

---

## Implementation

1. Python Research Backend (Public)

The current implementation is written in Python using:

- SciPy (optimization and sparse solvers)
- Shapely (geometric operations)
- NumPy (vectorized numerical computation)

This version prioritizes research clarity, reproducibility, and rapid parameter exploration.

2. C++ High-Performance Backend (In Development)

To scale simulations to networks with millions of degrees of freedom, a C++ backend is under active development.

Key design elements:

- Eigen-based sparse linear algebra
- LBFGSpp integration
- Memory-efficient adjacency representation
- SLURM-compatible HPC deployment
- CMake-based build system

The backend architecture is designed for performance-critical large-scale simulations.

---

## Installation
pip install -r requirements.txt

----

## Example Usage
python main.py --config configs/default.yaml







## Author
**Hongryol Jeon** PhD Candidate | Computational Mechanics | Optimization | Soft Materials  
University of Illinois Urbana-Champaign  
📧 [hj41@illinois.edu](mailto:hj41@illinois.edu)  
🔗 GitHub: [https://github.com/hongryoljeon](https://github.com/hongryoljeon)

## How to Build

```bash
mkdir build
cd build
cmake ..
make
./fiber_optimizer
---






./spring_optimizer

