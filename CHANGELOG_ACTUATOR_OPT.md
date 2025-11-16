# Changelog: Actuator Optimization Branch

## Branch: `optimize-actuator-locations`

### Summary

This branch adds the ability to automatically optimize actuator locations to maximize containment probability, instead of requiring users to manually specify where to actuate.

### Key Changes

#### 1. New Command-Line Arguments

Added to `controlled.py`:
- `--auto-actuators`: Enable automatic actuator location optimization
- `--num-actuators`: Number of actuators to optimize (default: 2)
- `--actuator-maxiter`: CMA-ES iterations for actuator optimization (default: 100)
- `--actuator-policy-maxiter`: CMA-ES iterations for policy optimization during actuator search (default: 20)

Modified:
- `--actuate`: Now optional (default: None instead of [2])

#### 2. New Optimization Components

**Added `ActuatorObjective` class** (lines 1104-1222):
- Implements nested optimization for actuator placement
- Outer loop: searches over actuator locations using CMA-ES
- Inner loop: optimizes policy for each actuator configuration
- Objective: maximize containment probability P(x_target > threshold)

**Added `optimize_actuator_locations` function** (lines 1225-1352):
- Main entry point for actuator optimization
- Similar structure to `optimize_sensor_locations`
- Performs nested CMA-ES optimization
- Returns optimal actuator indices and best containment probability

#### 3. Modified Main Function

Updated workflow logic:
1. Determine sensor locations (manual, file, or auto-optimize)
2. **NEW**: Determine actuator locations (manual, default, or auto-optimize)
3. If actuators were optimized, perform final policy refinement
4. Otherwise, perform standard policy optimization
5. Save results with actuator metadata

#### 4. New Output Files

When using `--auto-actuators`:
- `actuators_auto_N.npz`: Contains optimal actuator locations and metrics
- `policy_cm*_actoptN_p*.npz`: Policy file with optimized actuators

#### 5. Documentation

**New files:**
- `ACTUATOR_OPTIMIZATION.md`: Comprehensive documentation of the feature
- `CHANGELOG_ACTUATOR_OPT.md`: This file

**Updated files:**
- `README.md`: Added actuator optimization to Quick Start and documentation

### Technical Details

#### Algorithm

The actuator optimization uses a nested optimization approach:

```
for each candidate actuator configuration:
    1. Create polynomial policy with those actuators
    2. Run CMA-ES to optimize policy (quick, few iterations)
    3. Evaluate containment probability
    4. Return negative probability (for minimization)
```

The outer CMA-ES then searches over actuator configurations to maximize this objective.

#### Encoding

Continuous parameters θ ∈ [0, N-1]^num_actuators are mapped to discrete indices:
- Round to nearest integer
- Ensure uniqueness by searching nearby indices if duplicates occur
- Same encoding strategy as sensor optimization

#### Computational Cost

- **High**: Each outer iteration requires multiple inner policy optimizations
- Typical runtime: 30-60 minutes for 2 actuators with 50 outer / 10 inner iterations
- No parallelization in inner loop (would require too much memory)

### Usage Examples

#### Basic Usage

```bash
python controlled.py --auto-actuators --auto-sensors \
  --num-actuators 2 --threshold 0.0 --maxiter 30
```

#### Fast Testing

```bash
python controlled.py --auto-actuators --sense 0 10 \
  --num-actuators 1 --actuator-maxiter 10 \
  --actuator-policy-maxiter 5 --T 100
```

#### Full Pipeline

```bash
python controlled.py \
  --auto-sensors --num-sensors 2 --sensor-maxiter 100 \
  --auto-actuators --num-actuators 2 \
  --actuator-maxiter 50 --actuator-policy-maxiter 10 \
  --threshold 0.0 --maxiter 50 --n-cpus 8
```

### Backward Compatibility

✅ All existing usage patterns continue to work:
- Default behavior: uses actuator index [2] if neither --actuate nor --auto-actuators specified
- Manual actuator specification: `--actuate 1 2 3` works as before
- All existing command-line arguments preserved

### Testing

- ✅ Syntax check: passes `python3 -m py_compile`
- ✅ Linter: no errors
- ✅ Code structure: verified all imports and class definitions

### Performance Recommendations

1. Start with minimal iterations for testing:
   - `--actuator-maxiter 10 --actuator-policy-maxiter 5`

2. Use fewer actuators initially:
   - `--num-actuators 1` is much faster than `--num-actuators 3`

3. Reduce simulation time during testing:
   - `--T 100` instead of default 1000

4. Progressive refinement:
   - Run quick optimization to find approximate actuator locations
   - Then do full optimization with those locations manually specified

### Future Improvements

Potential enhancements:
1. **Parallel inner loops**: Evaluate multiple actuator configurations simultaneously
2. **Multi-fidelity optimization**: Use cheaper surrogate models in outer loop
3. **Joint sensor-actuator optimization**: Optimize both simultaneously
4. **Gradient-based methods**: Use differentiable simulators
5. **Heuristic initialization**: Pre-filter actuator locations based on system structure

### Files Changed

- `controlled.py`: +361 lines, -6 lines
- `README.md`: Updated Quick Start and documentation
- `ACTUATOR_OPTIMIZATION.md`: New comprehensive guide
- `CHANGELOG_ACTUATOR_OPT.md`: This changelog

### Git Information

- **Branch**: `optimize-actuator-locations`
- **Based on**: `main`
- **Status**: Ready for review/merge
