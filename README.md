# Agent-Based Fiber Network Optimization

## Overview
This project implements a high-performance framework for energy-based optimization of agent-built fiber networks. Fiber deposition is modeled as a local agent process, where each agent deposits fibers based on geometric constraints and local interaction rules. 

**Key Insight:** Despite purely local deposition rules, the resulting system exhibits global emergent mechanical behavior, which can be tuned for programmable soft material design.

The framework enables:
* Analytical gradient computation & Sparse Hessian assembly
* L-BFGS-B constrained optimization
* Mechanical property evaluation (Bulk & Shear Moduli)

---

## Motivation
Inspired by biological structural formation, understanding how local deposition rules influence global mechanical properties is essential for designing programmable soft materials. In this project, local agent parameters such as **Radius of reach (R)**, **Limiting segment length (LS)**, and **Preset strain (P)** are tuned to control emergent global mechanics including:
* Stress distribution & Energy landscape
* Bulk modulus ($K$) & Shear modulus ($G$)

This framework investigates the controllability of global mechanical response through local rule optimization, bridging the gap between micro-scale rules and macro-scale properties.

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

## 🛠 Current Implementation & Future Roadmap

### 🐍 Python Implementation (Fully Functional)
The core logic is currently implemented in **Python**, utilizing `SciPy` for robust numerical optimization and `Shapely` for complex geometric operations. This version is optimized for research, visualization, and rapid prototyping of mechanical parameters.

### 🚀 C++ Porting (In Progress - Meta Application Focus)
To scale this simulation for "meta-scale" networks (millions of nodes), I am developing a high-performance C++ version.
* **Status:** Core physics engine and agent logic have been successfully ported.
* **Objective:** Implementing **Eigen-based** memory management and **LBFGSpp** for production-level throughput.
* **Architecture:** Designed for HPC integration with SLURM and CMake-based build systems.

> **Note:** The C++ backend is currently hosted in a **Private Repository** for advanced performance tuning and proprietary algorithmic optimization. Access can be provided upon request for technical review.

---

## Mathematical Formulation & Architecture
The total energy of the system is defined as:
$$E_{total} = E_{elastic} + \Phi_{constraints}$$

The optimization engine follows a modular architecture:
1. **Agent Construction:** Local rule-based deposition.
2. **Energy Computation:** Summation of local potentials.
3. **Gradient/Hessian Module:** Analytical derivatives for sparse assembly.
4. **Optimization:** L-BFGS-B solver for energy minimization.
5. **Evaluation:** Extraction of $G$ and $K$ through virtual deformation.

---

## Features
* **Energy-based mechanical optimization**
* **Sparse linear algebra implementation** (Analytical derivatives)
* **Scalable architecture** with modular design
* **CMake-based build system** for cross-platform deployment

---

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

