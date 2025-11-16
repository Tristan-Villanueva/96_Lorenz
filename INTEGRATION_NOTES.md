# Sensor + Control Integration Notes

## Overview

The `controlled.py` script has been enhanced to integrate both sensor optimization and control policy optimization into a single workflow. This eliminates the need to run `optimal_sensors.py` separately.

## What Changed

### New Features in `controlled.py`

1. **Integrated Sensor Optimization**: Automatically finds optimal sensor locations via mutual information maximization
2. **Smart Sensor Loading**: Intelligently determines sensor locations from multiple sources with priority:
   - Manual specification via `--sense` (highest priority)
   - Existing sensor file via `--sensor-file`
   - Automatic optimization via `--auto-sensors` (if neither above is provided)

### New Command-Line Arguments

```
--auto-sensors              Enable automatic sensor optimization
--num-sensors NUM_SENSORS   Number of sensors (default: 2)
--sensor-T SENSOR_T         Simulation time for sensor data (default: 10000)
--sensor-maxiter MAXITER    CMA-ES iterations for sensors (default: 200)
--num-bins NUM_BINS         Histogram bins for MI computation (default: 100)
```

### New Functions Added

1. `compute_mutual_information()`: Calculates I(target; sensors)
2. `SensorMIObjective`: CMA-ES objective for sensor placement
3. `optimize_sensor_locations()`: Main sensor optimization function

### Modified Behavior

The main workflow now operates in two steps (if sensors aren't provided):

**STEP 1: OPTIMAL SENSOR PLACEMENT**
- Runs uncontrolled simulation with long trajectory (default 10000 time units)
- Optimizes sensor indices to maximize I(x_3; sensors)
- Saves results to `sensors_auto_N.npz`

**STEP 2: CONTROL POLICY OPTIMIZATION**
- Uses optimized sensors from Step 1
- Optimizes polynomial control policy
- Saves policy to `policy_*.npz` with sensor MI info included

## Usage Examples

### 1. Fully Automated (Recommended)

```bash
python controlled.py --auto-sensors --num-sensors 2 --c-max 1.0 --maxiter 50 --n-cpus 8
```

This single command:
- Optimizes 2 sensor locations
- Uses those sensors to optimize control policy
- Saves both sensor results and policy

### 2. Manual Sensors

```bash
python controlled.py --sense 5 12 --c-max 1.0 --maxiter 50
```

Skips sensor optimization entirely, uses x_6 and x_13 directly.

### 3. Pre-computed Sensors

```bash
# First run (or use existing sensors.npz)
python optimal_sensors.py --num-sensors 2 --output mysensors.npz

# Then use them
python controlled.py --sensor-file mysensors.npz --c-max 1.0 --maxiter 50
```

Uses sensors from file without re-optimization.

### 4. Force Re-optimization

```bash
python controlled.py --auto-sensors --sensor-file sensors.npz --c-max 1.0 --maxiter 50
```

Even if `sensors.npz` exists, `--auto-sensors` flag forces re-optimization.

## Priority System

The script determines sensors in this order:

1. **`--sense`**: Manual specification (highest priority)
2. **`--sensor-file`** (without `--auto-sensors`): Load from file
3. **Auto-optimization**: If neither above, or if `--auto-sensors` is set

## Output Files

### When Auto-Optimizing Sensors

Two files are created:

1. **`sensors_auto_N.npz`**: Contains sensor optimization results
   - `sensor_indices`: Optimal sensor locations (0-indexed)
   - `mutual_information`: Achieved MI value
   - System parameters (N, F, seed, etc.)

2. **`policy_*.npz`**: Contains control policy results
   - All standard policy data
   - **NEW**: `sensor_mutual_information` field (if available)

### When Using Pre-existing Sensors

Only the policy file is created, but it includes sensor MI if available in the input file.

## Backward Compatibility

The integration maintains full backward compatibility:

- `optimal_sensors.py` still works as a standalone script
- Existing `sensors.npz` files can still be used
- Two-step workflow (separate scripts) still works
- All existing command-line arguments preserved

## Performance Considerations

### Integrated Workflow
- **Pros**: Single command, no manual file management
- **Cons**: Longer initial run time (sensor optimization + policy optimization)
- **Best for**: Quick experiments, automated pipelines

### Two-Step Workflow
- **Pros**: Can reuse expensive sensor optimization, easier debugging
- **Cons**: Must manage sensor files manually
- **Best for**: Multiple control policies with same sensors

## Technical Details

### Sensor Optimization

Uses the same algorithm as `optimal_sensors.py`:
- Uncontrolled Lorenz 96 simulation with T=10000 (default)
- Mutual information estimation via histograms
- CMA-ES optimization over continuous parameters → discrete indices
- Uniqueness constraint enforcement

### Integration Points

1. **Import**: Added `from uncontrolled import solve_lorenz96_uncontrolled`
2. **Sensor Logic**: Smart decision tree in `main()` determines sensor source
3. **Step Labels**: Clear "STEP 1" and "STEP 2" output for user clarity
4. **Data Flow**: Sensor MI value passed through to policy save file

## Migration Guide

### From Old Workflow

**Before:**
```bash
python optimal_sensors.py --num-sensors 2 --T 10000
python controlled.py --sensor-file sensors.npz --c-max 1.0
python post.py
```

**After:**
```bash
python controlled.py --auto-sensors --num-sensors 2 --c-max 1.0
python post.py
```

### For Existing Scripts

If you have scripts that call `optimal_sensors.py` and `controlled.py` separately:

**No changes required!** Both scripts still work independently.

To migrate to integrated workflow, replace:
```python
subprocess.run(["python", "optimal_sensors.py", "--num-sensors", "2"])
subprocess.run(["python", "controlled.py", "--sensor-file", "sensors.npz"])
```

With:
```python
subprocess.run(["python", "controlled.py", "--auto-sensors", "--num-sensors", "2"])
```

## Testing

Verified:
✅ Integration compiles and imports correctly
✅ Command-line interface shows new options
✅ Backward compatibility with existing sensor files
✅ Manual sensor specification still works
✅ Default behavior maintains compatibility

## Summary

The integration provides a streamlined workflow while maintaining full backward compatibility. Users can now run a single command to solve both optimization problems, but the original two-step approach remains fully functional for those who prefer it.

