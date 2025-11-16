# Actuator Optimization Feature

## Overview

This branch adds automatic optimization of actuator locations to maximize containment probability. Instead of manually specifying which states to actuate, the system can now find the optimal actuation locations automatically.

## What Changed

### New Command-Line Arguments

- `--auto-actuators`: Enable automatic actuator location optimization
- `--num-actuators N`: Number of actuators to optimize (default: 2)
- `--actuator-maxiter N`: CMA-ES iterations for actuator optimization (default: 100)
- `--actuator-policy-maxiter N`: CMA-ES iterations for policy optimization during actuator search (default: 20)

### Modified Arguments

- `--actuate`: Now optional; if not specified and `--auto-actuators` is set, actuators will be optimized

## How It Works

The actuator optimization performs a **nested optimization**:

1. **Outer loop**: CMA-ES searches over possible actuator locations
2. **Inner loop**: For each actuator configuration, CMA-ES optimizes the control policy
3. **Objective**: Maximize containment probability P(x_target > threshold)

This is computationally expensive, so the inner loop uses fewer iterations (`--actuator-policy-maxiter`) than the final policy optimization.

## Usage Examples

### Basic Usage

Find optimal locations for 2 actuators:

```bash
python controlled.py --auto-actuators --auto-sensors --num-actuators 2 \
  --actuator-maxiter 50 --actuator-policy-maxiter 10 \
  --threshold 0.0 --maxiter 30
```

### With Manual Sensors

If you already know which sensors to use:

```bash
python controlled.py --auto-actuators --sense 5 12 \
  --num-actuators 2 --threshold 0.0
```

### Different Number of Actuators

Optimize for 3 actuators:

```bash
python controlled.py --auto-actuators --auto-sensors --num-actuators 3 \
  --threshold 0.0
```

### Fast Testing

Quick test with minimal iterations:

```bash
python controlled.py --auto-actuators --sense 0 10 \
  --num-actuators 1 --actuator-maxiter 10 --actuator-policy-maxiter 5 \
  --maxiter 10 --T 100
```

## Output Files

When using `--auto-actuators`, the following files are created:

1. **`actuators_auto_N.npz`**: Contains optimal actuator locations and performance metrics
   - `actuator_indices`: The optimal actuator locations (0-indexed)
   - `best_containment_prob`: Best containment probability achieved
   - `num_actuators`: Number of actuators
   - `sensed_dims`: Sensor locations used
   - Other configuration parameters

2. **`policy_cmX_actoptN_pY.npz`**: The optimized control policy
   - Same format as before, but with optimized actuator locations
   - Includes `actuator_optimized=True` flag

## Performance Considerations

### Computational Cost

Actuator optimization is **much more expensive** than sensor optimization because:
- Each actuator configuration requires a full policy optimization
- This creates a nested optimization problem

### Recommendations

1. **Start with few iterations**: Test with `--actuator-maxiter 10 --actuator-policy-maxiter 5`
2. **Use fewer actuators**: `--num-actuators 1` is faster than `--num-actuators 3`
3. **Reduce simulation time**: Use `--T 100` during testing
4. **No parallelization in inner loop**: The nested optimization runs serially (parallelization would require too much memory)

### Typical Runtime

For N=40 Lorenz 96 system with default parameters:
- 1 actuator, 10 outer iterations, 5 inner iterations: ~5-10 minutes
- 2 actuators, 50 outer iterations, 10 inner iterations: ~30-60 minutes
- 2 actuators, 100 outer iterations, 20 inner iterations: ~2-4 hours

## Algorithm Details

### Actuator Encoding

Continuous parameters are mapped to discrete actuator indices:
- Each parameter θᵢ ∈ [0, N-1] maps to an integer index
- Duplicates are resolved by searching nearby indices
- Similar to sensor optimization encoding

### Objective Function

For each set of actuator locations:
1. Create a polynomial policy with those actuators
2. Run CMA-ES to optimize policy parameters (quick, few iterations)
3. Return the negative of the containment probability (to minimize)

### Final Refinement

After finding optimal actuators, the system performs a final policy optimization with full iterations (`--maxiter`) to refine the policy.

## Example Workflow

### Full Pipeline

```bash
# Step 1: Optimize both sensors and actuators
python controlled.py \
  --auto-sensors --num-sensors 2 --sensor-maxiter 100 \
  --auto-actuators --num-actuators 2 --actuator-maxiter 50 --actuator-policy-maxiter 10 \
  --threshold 0.0 --maxiter 50 --n-cpus 8

# Step 2: Visualize results
python post.py --T 2000
```

### Output Interpretation

The script will print:
```
======================================================================
STEP 1: OPTIMAL ACTUATOR PLACEMENT
======================================================================
Optimizing 2 actuator locations...
  Sensors: [4, 12]
  Target: x_3 (index 2)
  Threshold: 0.0
  Objective: Maximize P(x_3 > 0.0)

Running CMA-ES for actuator optimization...
  Outer iterations (actuators): 50
  Inner iterations (policy per actuator config): 10
  WARNING: This is a nested optimization and may take significant time!

[CMA-ES progress with evaluations showing different actuator configs...]

Actuator optimization complete!
  Best containment probability: 0.723456
  Optimal actuator indices:
    Actuator 1: x_3 (index 2)
    Actuator 2: x_15 (index 14)
======================================================================
```

## Backward Compatibility

The implementation maintains backward compatibility:
- If neither `--actuate` nor `--auto-actuators` is specified, defaults to `[2]` (x_3)
- All existing command-line usage continues to work
- Old policy files remain compatible

## Comparison: Manual vs Optimized Actuators

To compare manual and optimized actuator placement:

```bash
# Manual placement (baseline)
python controlled.py --sense 5 12 --actuate 2 --threshold 0.0 --maxiter 50

# Optimized placement
python controlled.py --sense 5 12 --auto-actuators --num-actuators 1 \
  --actuator-maxiter 50 --threshold 0.0 --maxiter 50
```

Compare the final containment probabilities to see the improvement from optimization.

## Technical Notes

### Why Nested Optimization?

Unlike sensors (which use mutual information), actuators must be evaluated by their **control effectiveness**. This requires actually running the controlled system and measuring containment probability, which in turn requires optimizing the policy for each actuator configuration.

### Alternative Approaches

Possible future improvements:
1. **Gradient-based methods**: Use differentiable simulators
2. **Heuristics**: Pre-filter actuator locations based on system structure
3. **Multi-fidelity**: Use cheaper surrogates in outer loop
4. **Joint optimization**: Optimize actuators and policy simultaneously

### Limitations

- Computational cost limits the search space
- Inner loop uses fewer iterations (may not find optimal policy for each actuator config)
- No parallelization in inner loop
- Assumes sensors are fixed (doesn't jointly optimize sensors and actuators)

## Questions?

See `README.md` for general usage and `QUICKSTART.md` for quick examples.
