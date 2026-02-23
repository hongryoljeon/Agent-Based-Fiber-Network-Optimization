# Agent-Based Fiber Network Optimization

## Overview

This project implements a C++ framework for energy-based optimization of agent-built fiber networks.

Fiber deposition is modeled as a local agent process, where each agent deposits fibers based on geometric constraints and local interaction rules. 

Despite purely local deposition rules, the resulting system exhibits global emergent mechanical behavior.

The framework enables:

- Analytical gradient computation
- Sparse Hessian assembly
- L-BFGS-B constrained optimization
- Mechanical property evaluation (Bulk & Shear Moduli)

---

## Motivation

Inspired by animal structures, understanding how local deposition rules influence global mechanical properties is essential for designing programmable soft materials.

In this project, local agent parameters such as:

- Radius of reach (R)
- Limiting segment length (LS)
- Preset strain (P)

are tuned to control emergent global mechanics including:

- Stress distribution
- Bulk modulus
- Shear modulus

This framework investigates the controllability of global mechanical response through local rule optimization.

---

## Mathematical Formulation

The total energy of the system is defined as:

E = Elastic Energy + Constraint Penalty

The optimization is performed using:

- Analytical gradient computation
- Analytical Hessian construction
- Sparse matrix representation
- L-BFGS-B constrained solver

---

## Architecture

The framework is structured into modular components:

- Energy module
- Gradient module
- Hessian assembly
- Optimization engine
- Network update system

Data flow:

Agent construction → Energy computation →  Optimization → Network Update → Hessian assembly → Mechanical property evaluation

---

## Features

- Energy-based mechanical optimization
- Sparse linear algebra implementation
- Analytical derivatives
- Scalable architecture
- CMake-based build system

----
## 🛠 Current Implementation & Future Roadmap

### 🐍 Python Implementation (Fully Functional)
The core logic of the agent-based fiber network optimization is currently implemented in **Python**. It utilizes `SciPy` for robust numerical optimization and `Shapely` for complex geometric operations. This version is ideal for research, visualization, and rapid prototyping of different mechanical parameters.

### 🚀 C++ Porting (In Progress - Meta Application Focus)
To scale this simulation for massive networks (millions of nodes), I am currently developing a high-performance C++ version. 
* **Status:** Core physics engine and agent logic have been ported.
* **Objective:** Implementing **Eigen-based** memory optimization and **LBFGSpp** for production-level performance.
* **Target:** This C++ backend is designed to handle "meta-scale" applications where real-time simulation and heavy computational throughput are required.

> **Note:** For the current evaluation of the algorithm's mechanical accuracy, please refer to the Python source code. The C++ branch is under active optimization to ensure numerical stability matches the Python baseline.

----
## Author

**Hongryol Jeon**  
PhD Candidate | Computational Mechanics | Optimization | Soft Materials  

University of Illinois Urbana-Champaign  
📧 hj41@illinois.edu  
🔗 GitHub: https://github.com/hongryoljeon
---

## How to Build

```bash
mkdir build
cd build
cmake ..
make
./fiber_optimizer
---






./spring_optimizer

