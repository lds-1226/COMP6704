# COMP6704
# Exact Joint Optimization of Airport Gate Assignment and Taxiway Routing  
### — Benders Decomposition Approach on Kunming Changshui International Airport (KMG)

## Project Overview

This repository implements an **exact Benders decomposition algorithm** for the **joint optimization of airport gate assignment and conflict-free taxiway routing/scheduling**, tested on real-scale instances of **Kunming Changshui International Airport (KMG/ZPPP)** — one of China's busiest single-airport hubs with over 500 daily flights.

The method successfully solves to **proven global optimality** instances with up to **83 contact gates**, and a detailed taxiway network (218 nodes, 612 arcs) — a scale at which commercial solvers like Gurobi fail completely when solving the monolithic MILP.

---

## Key Features

- **Exact solution** via classical and accelerated Benders decomposition  
- **Magnanti–Wong Pareto-optimal cuts** + per-flight cut disaggregation (acceleration)  
- Proven optimality gap ≤ 0.5% in all tested cases  
- Comparison with:  
  - Sequential heuristic (industry practice)  
  - Monolithic MILP solved by Gurobi (times out beyond ~60 flights)  
- Full convergence plots and result tables automatically generated  
- Real airport topology and realistic flight schedules

