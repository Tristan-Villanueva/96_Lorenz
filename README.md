# Lorenz 96 Control Framework

This directory contains a complete framework for optimal control of the Lorenz 96 system using polynomial policies and information-theoretic sensor placement.

## Overview

The goal is to make the region `x_3 > t_threshold` as forward invariant as possible through optimal sensor placement and polynomial control policies.

**System**: Lorenz 96 with N=40 degrees of freedom, forcing parameter F=15

**Target**: Maximize P(x_3 > t_threshold) where t_threshold is configurable (default: 0)

## Workflow

### ⚡ Quick Start (Integrated Workflow)

The **recommended approach** is to run just the `controlled.py` script, which automatically handles sensor optimization, actuator optimization (optional), and control policy optimization:

```bash
# One-step optimization: sensors + control policy (manual actuators)
python controlled.py --auto-sensors --num-sensors 2 --actuate 2 --c-max 1.0 --threshold 0.0 --maxiter 50 --n-cpus 8

# OR: Optimize sensors AND actuators automatically (NEW!)
python controlled.py --auto-sensors --num-sensors 2 --auto-actuators --num-actuators 2 \
  --actuator-maxiter 50 --actuator-policy-maxiter 10 --threshold 0.0 --maxiter 30 --n-cpus 8
```

This will:
1. Automatically optimize sensor locations via mutual information
2. (Optional) Automatically optimize actuator locations to maximize containment probability
3. Use those sensors and actuators to optimize the control policy
4. Save sensor results, actuator results (if optimized), and policy parameters

**Key parameters:**
- `--auto-sensors`: Enable automatic sensor optimization
- `--num-sensors`: Number of sensors (default: 2)
- `--sensor-T`: Simulation time for sensor data (default: 10000)
- `--sensor-maxiter`: CMA-ES iterations for sensors (default: 200)
- `--actuate`: State indices to actuate (default: [2] for x_3 only; optional if using --auto-actuators)
- `--auto-actuators`: Enable automatic actuator optimization (NEW!)
- `--num-actuators`: Number of actuators for auto-optimization (default: 2)
- `--actuator-maxiter`: CMA-ES iterations for actuators (default: 100)
- `--actuator-policy-maxiter`: Inner policy iterations during actuator search (default: 20)
- `--c-max`: Actuation capacity (default: 1.0)
- `--threshold`: Target threshold t (default: 0.0)
- `--poly-order`: Polynomial order (1, 2, or 3; default: 2)
- `--maxiter`: CMA-ES iterations for policy (default: 50)
- `--n-cpus`: Number of CPUs for parallel evaluation (default: 1)

**Outputs:**
- `sensors_auto_2.npz`: Optimized sensor locations (if using --auto-sensors)
- `actuators_auto_2.npz`: Optimized actuator locations (if using --auto-actuators)
- `policy_cm1p0_act2_p2.npz`: Optimized control policy (manual actuators)
- `policy_cm1p0_actopt2_p2.npz`: Optimized control policy (optimized actuators)

### Alternative: Manual Sensor Specification

If you already know which sensors to use:

```bash
python controlled.py --sense 5 12 --c-max 1.0 --threshold 0.0 --maxiter 50
```

### Alternative: Two-Step Workflow

For more control, you can still run sensor optimization and policy optimization separately:

#### Step 1: Optimal Sensor Placement

```bash
python optimal_sensors.py --num-sensors 2 --target-variable 2 --T 10000 --seed 42
```

**Output:** `sensors.npz` containing optimal sensor indices

#### Step 2: Optimal Control Policy

```bash
python controlled.py --sensor-file sensors.npz --c-max 1.0 --threshold 0.0 --poly-order 2 --maxiter 50
```

**Output:** `policy_cm1p0_act2_p2.npz` containing optimized policy parameters

### Step 3: Post-Processing and Visualization

Load the optimized policy and generate comparison plots.

```bash
python post.py --policy policy_cm1p0_act2_p2.npz --T 1000 --num-thresholds 100
```

**Key parameters:**
- `--policy`: Policy file from Step 2 (default: policy.npz)
- `--T`: Simulation time (default: from policy file)
- `--c-max`: Control scaling (default: from policy file)
- `--num-thresholds`: Number of thresholds for containment plot (default: 100)
- `--output-dir`: Directory for plots (default: post_plots)

**Outputs:**
- `timeseries_comparison.png`: Time series of x_3 (controlled vs uncontrolled)
- `histogram_comparison.png`: Probability distributions
- `phase_portrait_3d.png`: 3D phase portraits
- `containment_probabilities.png`: P(x_3 > threshold) vs threshold
- `control_and_sensors.png`: Control signals and sensor measurements

## Example Complete Workflow

### Integrated Approach (Recommended)

```bash
# One command for everything (sensors + control)
python controlled.py --auto-sensors --num-sensors 2 --c-max 1.0 --threshold 0.0 --maxiter 50 --n-cpus 8

# Generate plots
python post.py --T 2000
```

### Two-Step Approach

```bash
# Step 1: Find optimal sensors
python optimal_sensors.py --num-sensors 2 --T 10000 --maxiter 200

# Step 2: Optimize control policy
python controlled.py --c-max 1.0 --threshold 0.0 --maxiter 50 --n-cpus 8

# Step 3: Generate plots
python post.py --T 2000
```

## Files

- **uncontrolled.py**: Lorenz 96 simulation without control
- **optimal_sensors.py**: Standalone mutual information-based sensor placement
- **controlled.py**: **Integrated sensor + actuator + policy optimization** via CMA-ES
- **post.py**: Post-processing and visualization
- **kl_bound_cmaes.py**: Lower bound computation utilities
- **plot_style.mplstyle**: Matplotlib style configuration
- **ACTUATOR_OPTIMIZATION.md**: Detailed documentation for actuator optimization feature

> **Note**: `controlled.py` now includes integrated sensor AND actuator optimization! You can run just this script with `--auto-sensors` and `--auto-actuators` flags to optimize everything at once!

## Requirements

```bash
pip install numpy scipy matplotlib cma scikit-learn
```

## Key Features

- **Polynomial Control Policies**: Configurable order (1, 2, or 3) with product terms
- **Bounded Control**: Uses tanh to enforce actuation limits
- **Information-Theoretic Sensor Placement**: Maximizes mutual information I(x_3; sensors)
- **Rigorous Lower Bounds**: Uses KL divergence-based bounds on containment probabilities
- **Parallel Optimization**: Multi-CPU support for faster policy optimization
- **Comprehensive Visualization**: Time series, histograms, phase portraits, and more

## System Dynamics

The Lorenz 96 system with control:

```
dx_i/dt = (x_{i+1} - x_{i-2}) * x_{i-1} - x_i + F + u_i
```

where:
- N = 40 (number of variables)
- F = 15 (forcing parameter)
- u_i is the control input (zero for non-actuated states)

## Control Policy

The polynomial policy maps sensor measurements s to control actions u:

```
u_i = c_max * tanh(Σ a_ij * (α_i * s)^powers)
```

where:
- powers are multi-indices with total degree ≤ poly_order
- α_i are learnable scaling parameters
- tanh enforces the control bound [-c_max, c_max]

## Notes

- All solvers use RK23 integration with rtol=1e-4, atol=1e-6
- Default parameters match the uncontrolled system in `uncon.py`
- The framework does NOT include IND decomposition (see Lorenz_pinsker_kl for that)
- Sensor indices are 0-indexed (x_1 = index 0, x_2 = index 1, x_3 = index 2, etc.)

