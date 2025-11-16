# Quick Start Guide: Lorenz 96 Control

## TL;DR - Run This

```bash
# Single command to do everything (sensors + control)
python controlled.py --auto-sensors --num-sensors 2 --c-max 1.0 --threshold 0.0 --maxiter 50 --n-cpus 8

# Then visualize
python post.py --T 2000
```

That's it! The script will:
1. ✅ Find optimal sensor locations (maximizing I(x_3; sensors))
2. ✅ Optimize control policy (maximizing P(x_3 > 0))
3. ✅ Save results to `sensors_auto_2.npz` and `policy_*.npz`

## What Just Happened?

### Step 1: Sensor Optimization (Automatic)
- Simulated uncontrolled Lorenz 96 for 10,000 time units
- Found 2 sensor locations that maximize mutual information with x_3
- Used CMA-ES with 200 iterations
- Saved results to `sensors_auto_2.npz`

### Step 2: Control Policy Optimization
- Used the optimized sensors from Step 1
- Optimized a polynomial (order 2) control policy
- Actuated only x_3 with capacity c_max=1.0
- Maximized P(x_3 > 0) using CMA-ES with 50 iterations
- Saved policy to `policy_cm1p0_act2_p2.npz`

### Step 3: Visualization
- Compared controlled vs uncontrolled trajectories
- Generated time series, histograms, phase portraits
- Computed containment probabilities
- Saved plots to `post_plots/` directory

## Common Usage Patterns

### Fast Testing (Fewer Iterations)
```bash
python controlled.py --auto-sensors --num-sensors 2 --sensor-maxiter 50 --maxiter 20 --n-cpus 4
```

### Production Run (More Iterations)
```bash
python controlled.py --auto-sensors --num-sensors 2 --sensor-maxiter 300 --maxiter 100 --n-cpus 16
```

### Different Threshold
```bash
python controlled.py --auto-sensors --threshold 5.0 --maxiter 50
```

### Multiple Actuators
```bash
python controlled.py --auto-sensors --actuate 1 2 3 --maxiter 50
```

### Manual Sensors (Skip Optimization)
```bash
python controlled.py --sense 5 12 --maxiter 50
```

## Understanding the Output

### Console Output
```
======================================================================
Lorenz 96 System: Integrated Sensor + Control Optimization
======================================================================

======================================================================
STEP 1: OPTIMAL SENSOR PLACEMENT
======================================================================
Running uncontrolled simulation for sensor optimization...
  T = 10000.0, dt = 0.01, transient = 100.0
  Collected 1000000 samples
  Target: x_3 (index 2)

Optimizing 2 sensor locations via CMA-ES...
  Maximum iterations: 200
[CMA-ES output...]

Sensor optimization complete!
  Mutual information: 2.345678 nats
  Optimal sensor indices:
    Sensor 1: x_5 (index 4)
    Sensor 2: x_13 (index 12)
======================================================================

======================================================================
STEP 2: CONTROL POLICY OPTIMIZATION
======================================================================
System: N = 40, F = 15.0
Actuated indices: [2]
Sensed indices: [4, 12]
Polynomial order: 2
Target variable: x_3 (index 2)
Threshold: 0.0
Objective: Maximize P(x_3 > 0.0)
======================================================================

[CMA-ES optimization...]

======================================================================
Optimization complete!
Best objective: 0.856234
Best lower bound: 0.823456
Best true P(x_3 > 0.0): 0.856234
Control penalty: 0.000000
======================================================================
```

### Files Created

1. **`sensors_auto_2.npz`**
   - Sensor indices: [4, 12]
   - Mutual information: 2.34 nats
   - System parameters

2. **`policy_cm1p0_act2_p2.npz`**
   - Polynomial coefficients
   - Actuated/sensed dimensions
   - Optimization results
   - Sensor MI (included for reference)

3. **`post_plots/`** (after running post.py)
   - `timeseries_comparison.png`
   - `histogram_comparison.png`
   - `phase_portrait_3d.png`
   - `containment_probabilities.png`
   - `control_and_sensors.png`

## Troubleshooting

### "Sensor file not found" Warning
This is normal! The script will automatically optimize sensors for you. To suppress the warning, add `--auto-sensors` flag.

### Out of Memory
Reduce the sensor simulation time:
```bash
python controlled.py --auto-sensors --sensor-T 5000 --maxiter 50
```

### Taking Too Long
Reduce iterations:
```bash
python controlled.py --auto-sensors --sensor-maxiter 100 --maxiter 30
```

### Want to Use Specific Sensors
Use `--sense` to skip sensor optimization:
```bash
python controlled.py --sense 0 10 20 --maxiter 50
```

## Advanced Options

### Custom Sensor Optimization Parameters
```bash
python controlled.py \
  --auto-sensors \
  --num-sensors 3 \
  --sensor-T 15000 \
  --sensor-maxiter 250 \
  --num-bins 150 \
  --maxiter 50
```

### Different Target Variable
To maximize P(x_10 > threshold) instead:
```bash
python controlled.py --auto-sensors --target-variable 9 --maxiter 50
```

### Multiple Actuators with Custom Capacity
```bash
python controlled.py \
  --auto-sensors \
  --actuate 1 2 3 \
  --c-max 2.0 \
  --maxiter 50
```

### Higher-Order Polynomial
```bash
python controlled.py --auto-sensors --poly-order 3 --maxiter 50
```

## Performance Tips

1. **Use multiple CPUs**: `--n-cpus 8` (or more) for parallel CMA-ES
2. **Start with fewer iterations**: Test with `--maxiter 20` before full run
3. **Reduce sensor data**: Use `--sensor-T 5000` for faster sensor optimization
4. **Reuse sensors**: Run sensor optimization once, then use `--sensor-file`

## Next Steps

After running the optimization:

1. **Visualize Results**
   ```bash
   python post.py --T 2000
   ```

2. **Try Different Thresholds**
   ```bash
   python controlled.py --auto-sensors --threshold 5.0 --maxiter 50
   python controlled.py --auto-sensors --threshold -5.0 --maxiter 50
   ```

3. **Compare Different Sensor Counts**
   ```bash
   for n in 1 2 3 4; do
     python controlled.py --auto-sensors --num-sensors $n --maxiter 50
   done
   ```

4. **Test Uncontrolled System**
   ```bash
   python uncontrolled.py --T 1000 --plot
   ```

## Getting Help

```bash
# Detailed help for controlled.py
python controlled.py --help

# Detailed help for post-processing
python post.py --help

# Standalone sensor optimization help
python optimal_sensors.py --help
```

## See Also

- `README.md` - Complete documentation
- `INTEGRATION_NOTES.md` - Technical details of sensor integration
- `SUMMARY.md` - Implementation summary

