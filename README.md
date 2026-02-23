# Agent-Based Fiber Network Optimization

## 🌟 Overview
This project develops a modular simulation and optimization framework to investigate how **local agent-based deposition rules** give rise to **emergent global mechanical behavior** in fiber networks.

Inspired by the collective construction of **Tent Caterpillars**, we model caterpillars as decentralized additive manufacturing agents. This research bridges the gap between biological decentralized construction and autonomous robotic manufacturing.

---

## 🔬 Research Motivation & Core Question
Biological systems like bird nests or caterpillar tents achieve structural integrity without a central blueprint.
**Core Question:** Can we program the macro-scale elastic properties of a material by only tuning the micro-scale deposition rules of autonomous agents?

We investigate how agent parameters control:
* **Radius of Reach ($R$):** Spatial interaction limit.
* **Limiting Segment ($LS$):** Geometric constraint on fiber length.
* **Preset Strain ($P$):** Initial mechanical tension.

---

## 📊 Scientific Validation (Key Results)

Based on ensemble simulations ($n=40$), the framework demonstrates precise control over global mechanics:

### 1. Control via Limiting Segment (LS)

* **Observation:** We observed a non-monotonic trend where global moduli ($G, K$) are optimized at specific $LS$ values. Tuning $LS$ allows for "programming" the stiffness distribution of the emergent network.

<p align="center">
  <img width="2958" height="2048" alt="image" src="https://github.com/user-attachments/assets/d6df07b8-0da9-4bf7-95f7-4b52b016d940" />
</p>


### 2. Reinforcement via Preset Strain (P)

* **Observation:** Moduli exhibit a clear monotonic scaling with $P$. However, the $G/K$ ratio reveals a transition in the network's structural response, stabilizing as $P$ increases beyond a critical threshold.

<p align="center">
 <img width="2958" height="2048" alt="image" src="https://github.com/user-attachments/assets/a551427a-89d3-4fc9-9867-be85a5c414b1" />
</p>
---

## 🧮 Method & Computation
The framework evaluates mechanics using a rigorous analytical approach rather than numerical approximation:
* **Energy Formulation:** The system energy follows a harmonic potential:
  $$E = \sum \frac{1}{2}k(L-L_0)^2$$
* **Elastic Moduli Calculation:** Bulk ($K$) and Shear ($G$) moduli are derived by computing the **Hessian Matrix ($H$)** and incorporating non-affine displacement corrections:
  $$M(K,G) = \frac{1}{A}(M_{affine} - \Xi^T H^{-1} \Xi)$$
* **Stability Analysis:** Eigenvalue analysis of the Hessian ($\lambda > 10^{-3}$) is performed to ensure the structural integrity and local stability of the optimized networks.
  
---

## 🛠 Implementation Roadmap

### 1. Python Research Backend (Public)
* **Optimization:** `SciPy` (L-BFGS-B) for robust energy minimization.
* **Geometry:** `Shapely` for complex intersection and vision handling.
* **Performance:** `Numba` JIT compilation for bottleneck functions.

### 2. C++ High-Performance Backend (In Development)
* **Scaling:** Designed for meta-scale simulations (millions of nodes).
* **Numerical Backend:** `Eigen` for sparse linear algebra and `LBFGSpp` for high-speed optimization.
* **HPC Ready:** Compatible with SLURM and CMake.
* *Note: The C++ source is currently in a private repository for performance tuning.*

---
## 🚀 Quick Start

# Clone the repository
```bash
git clone [https://github.com/hongryoljeon/Agent-Based-Fiber-Network-Optimization.git](https://github.com/hongryoljeon/Agent-Based-Fiber-Network-Optimization.git)
cd Agent-Based-Fiber-Network-Optimization

# Install dependencies
pip install -r requirements.txt
```

# Run simulation
```bash
python main.py --config configs/default.yaml
```
---
## 🎓 Author
**Hongryol Jeon** PhD Candidate | Computational Mechanics | UIUC  
Email: [hj41@illinois.edu](mailto:hj41@illinois.edu)


