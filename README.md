# Agent-built Fiber Network Optimization

## Overview
This projects implements C++ framework for energy optimization of agent-built fiber networks.

The system models fiber deposition as an agent-based process and solve mechanical equilibrium using energy minimization techniques.

----

## Mathematical Formulation

The total energy of the system is defined as:

E = Elastic Energy + Constraint Penalty

Optimization is performed using:
- Analytical gradient computation
- L-BFGS-B constrained optimization

---

## Features
- Energy-based optimization
- Analytical gradient implementation
- Sparse linear solver structure
- Modular C++ architecture
- CMake build system



----
## Future Work
- GPU acceleration (CUDA)
- Parallel optimization
- Large-scale network benchmarking
- Visualization integration

---

## Author

**Hongryol Jeon**  
PhD Candidate | Computational Mechanics | Optimization | Soft Materials  

University of Illinois Urbana-Champaign  
📧 hj41@illinois.edu  
🔗 GitHub: https://github.com/hongryoljeon


-----


## How to Build

```bash
mkdir build
cd build
cmake ..
make
./spring_optimizer

