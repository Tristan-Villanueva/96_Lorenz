# Quick Start Guide: Lorenz 96 Control

## TL;DR - Run This

```bash
# Option 1: Sensors + Control (manual actuators)
python controlled.py --auto-sensors --num-sensors 2 --actuate 2 --c-max 1.0 --threshold 0.0 --maxiter 50 --n-cpus 8

# Option 2: Sensors + Actuators + Control (fully automatic) - NEW!
python controlled.py --auto-sensors --num-sensors 2 --auto-actuators --num-actuators 2 \
  --actuator-maxiter 30 --actuator-policy-maxiter 10 --threshold 0.0 --maxiter 30 --n-cpus 8

# Then visualize
python post.py --T 2000
```

**Option 1** (sensors + control):
1. ✅ Find optimal sensor locations (maximizing I(x_3; sensors))
2. ✅ Optimize control policy (maximizing P(x_3 > 0))
3. ✅ Save results to `sensors_auto_2.npz` and `policy_*.npz`

**Option 2** (sensors + actuators + control):
1. ✅ Find optimal sensor locations (maximizing I(x_3; sensors))
2. ✅ Find optimal actuator locations (maximizing P(x_3 > 0))
3. ✅ Optimize control policy with those actuators
4. ✅ Save results to `sensors_auto_2.npz`, `actuators_auto_2.npz`, and `policy_*.npz`

## What Just Happened?

### Step 1: Sensor Optimization (Automatic)
- Simulated uncontrolled Lorenz 96 for 10,000 time units
- Found 2 sensor locations that maximize mutual information with x_3
- Used CMA-ES with 200 iterations
- Saved results to `sensors_auto_2.npz`

### Step 2: Actuator Optimization (NEW - Optional)
**Only if using `--auto-actuators`:**
- Searches for optimal actuator locations via nested optimization
- **Outer loop**: CMA-ES searches over actuator configurations (30 iterations)
- **Inner loop**: For each configuration, optimizes policy (10 iterations)
- **Objective**: Maximize containment probability P(x_3 > 0)
- Saved results to `actuators_auto_2.npz`
- ⚠️ **Note**: This is computationally expensive (~30-60 min for 2 actuators)

### Step 3: Control Policy Optimization
- Used the optimized sensors from Step 1
- Used either manual or optimized actuators
- Optimized a polynomial (order 2) control policy
- Maximized P(x_3 > 0) using CMA-ES with 50 iterations (or 30 if actuators optimized)
- Saved policy to `policy_cm1p0_act2_p2.npz` (or `policy_cm1p0_actopt2_p2.npz`)

### Step 4: Visualization
- Compared controlled vs uncontrolled trajectories
- Generated time series, histograms, phase portraits
- Computed containment probabilities
- Saved plots to `post_plots/` directory

## Common Usage Patterns

### Fast Testing (Fewer Iterations)
```bash
# Without actuator optimization
python controlled.py --auto-sensors --num-sensors 2 --sensor-maxiter 50 --maxiter 20 --n-cpus 4

# With actuator optimization (slower)
python controlled.py --auto-sensors --num-sensors 2 --auto-actuators --num-actuators 1 \
  --actuator-maxiter 10 --actuator-policy-maxiter 5 --maxiter 10 --T 100
```

### Production Run (More Iterations)
```bash
# Without actuator optimization
python controlled.py --auto-sensors --num-sensors 2 --sensor-maxiter 300 --maxiter 100 --n-cpus 16

# With actuator optimization (much slower)
python controlled.py --auto-sensors --num-sensors 2 --auto-actuators --num-actuators 2 \
  --actuator-maxiter 100 --actuator-policy-maxiter 20 --maxiter 50 --n-cpus 8
```

### Different Threshold
```bash
python controlled.py --auto-sensors --threshold 5.0 --maxiter 50

# Or with actuator optimization
python controlled.py --auto-sensors --auto-actuators --threshold 5.0 \
  --actuator-maxiter 30 --actuator-policy-maxiter 10 --maxiter 30
```

### Multiple Actuators

**Manual specification:**
```bash
python controlled.py --auto-sensors --actuate 1 2 3 --maxiter 50
```

**Automatic optimization:**
```bash
python controlled.py --auto-sensors --auto-actuators --num-actuators 3 \
  --actuator-maxiter 50 --actuator-policy-maxiter 10 --maxiter 30
```

### Manual Sensors (Skip Sensor Optimization)
```bash
# Manual sensors, manual actuators
python controlled.py --sense 5 12 --actuate 2 --maxiter 50

# Manual sensors, optimize actuators
python controlled.py --sense 5 12 --auto-actuators --num-actuators 2 \
  --actuator-maxiter 30 --actuator-policy-maxiter 10 --maxiter 30
```

### Optimize Only Actuators (Use Existing Sensors)
```bash
# If you already have optimized sensors
python controlled.py --sensor-file sensors_auto_2.npz --auto-actuators --num-actuators 2 \
  --actuator-maxiter 50 --actuator-policy-maxiter 10 --maxiter 30
```

## Understanding the Output

### Console Output (without `--auto-actuators`)
```
======================================================================
Lorenz 96 System: Integrated Sensor + Actuator + Control Optimization
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

### Console Output (with `--auto-actuators`)
```
======================================================================
Lorenz 96 System: Integrated Sensor + Actuator + Control Optimization
======================================================================

[Sensor optimization as above...]

======================================================================
STEP 1: OPTIMAL ACTUATOR PLACEMENT
======================================================================
Optimizing 2 actuator locations...
  Sensors: [4, 12]
  Target: x_3 (index 2)
  Threshold: 0.0
  Objective: Maximize P(x_3 > 0.0)

Running CMA-ES for actuator optimization...
  Outer iterations (actuators): 30
  Inner iterations (policy per actuator config): 10
  WARNING: This is a nested optimization and may take significant time!

[CMA-ES evaluations with progress...]
  [Eval 1] Actuators [20, 30] → P(x_target > threshold) = 0.723456
  [Eval 2] Actuators [2, 15] → P(x_target > threshold) = 0.756234
  ...

Actuator optimization complete!
  Best containment probability: 0.789234
  Optimal actuator indices:
    Actuator 1: x_3 (index 2)
    Actuator 2: x_16 (index 15)
======================================================================

======================================================================
STEP 2: FINAL POLICY REFINEMENT
======================================================================
System: N = 40, F = 15.0
Actuated indices: [2, 15]
Sensed indices: [4, 12]
...
======================================================================
```

### Files Created

**Without `--auto-actuators`:**

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

**With `--auto-actuators`:**

1. **`sensors_auto_2.npz`** (same as above)

2. **`actuators_auto_2.npz`** (NEW!)
   - Actuator indices: [2, 15]
   - Best containment probability: 0.789
   - Sensor locations used
   - System parameters

3. **`policy_cm1p0_actopt2_p2.npz`**
   - Polynomial coefficients
   - Actuated/sensed dimensions (with optimized actuators)
   - Optimization results
   - Sensor MI and actuator optimization info

4. **`post_plots/`** (same as above)

## Troubleshooting

### "Sensor file not found" Warning
This is normal! The script will automatically optimize sensors for you. To suppress the warning, add `--auto-sensors` flag.

### Actuator Optimization Taking Too Long
Actuator optimization is **much slower** than sensor optimization because it requires optimizing a policy for each actuator configuration.

**Quick fixes:**
```bash
# Use fewer actuators
--num-actuators 1

# Reduce outer iterations
--actuator-maxiter 10

# Reduce inner iterations
--actuator-policy-maxiter 5

# Reduce simulation time
--T 100

# Complete example for fast testing:
python controlled.py --auto-sensors --auto-actuators --num-actuators 1 \
  --actuator-maxiter 10 --actuator-policy-maxiter 5 --T 100 --maxiter 10
```

### Out of Memory
Reduce the sensor simulation time:
```bash
python controlled.py --auto-sensors --sensor-T 5000 --maxiter 50
```

### Taking Too Long (General)
Reduce iterations:
```bash
# Without actuator optimization
python controlled.py --auto-sensors --sensor-maxiter 100 --maxiter 30

# With actuator optimization
python controlled.py --auto-sensors --auto-actuators \
  --actuator-maxiter 20 --actuator-policy-maxiter 5 --maxiter 20
```

### Want to Use Specific Sensors or Actuators
Use `--sense` and `--actuate` to skip optimization:
```bash
# Specific sensors and actuators
python controlled.py --sense 0 10 20 --actuate 2 5 --maxiter 50

# Specific sensors, optimize actuators
python controlled.py --sense 0 10 20 --auto-actuators --num-actuators 2 \
  --actuator-maxiter 30 --actuator-policy-maxiter 10
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

**Manual actuators:**
```bash
python controlled.py \
  --auto-sensors \
  --actuate 1 2 3 \
  --c-max 2.0 \
  --maxiter 50
```

**Optimized actuators:**
```bash
python controlled.py \
  --auto-sensors \
  --auto-actuators --num-actuators 3 \
  --actuator-maxiter 30 --actuator-policy-maxiter 10 \
  --c-max 2.0 \
  --maxiter 30
```

### Higher-Order Polynomial
```bash
python controlled.py --auto-sensors --poly-order 3 --maxiter 50

# Or with actuator optimization
python controlled.py --auto-sensors --auto-actuators --poly-order 3 \
  --actuator-maxiter 30 --actuator-policy-maxiter 10 --maxiter 30
```

### Full Optimization (Sensors + Actuators + Policy)
```bash
python controlled.py \
  --auto-sensors --num-sensors 2 --sensor-maxiter 100 \
  --auto-actuators --num-actuators 2 \
  --actuator-maxiter 50 --actuator-policy-maxiter 10 \
  --threshold 0.0 --maxiter 30 --n-cpus 8
```

## Performance Tips

1. **Use multiple CPUs**: `--n-cpus 8` (or more) for parallel CMA-ES in final policy optimization
   - Note: Actuator optimization inner loop is **not** parallelized (to save memory)
   
2. **Start with fewer iterations**: Test with minimal iterations before full run
   - Standard: `--maxiter 20`
   - With actuators: `--actuator-maxiter 10 --actuator-policy-maxiter 5`
   
3. **Reduce sensor data**: Use `--sensor-T 5000` for faster sensor optimization

4. **Reuse sensors**: Run sensor optimization once, then use `--sensor-file`
   ```bash
   # First run: optimize sensors
   python controlled.py --auto-sensors --num-sensors 2
   
   # Later runs: reuse sensors
   python controlled.py --sensor-file sensors_auto_2.npz --actuate 2 --maxiter 50
   ```

5. **Progressive actuator optimization**: 
   - Start with 1 actuator, then try 2, then 3
   - Start with low iterations, then increase
   ```bash
   # Quick test
   python controlled.py --sense 0 10 --auto-actuators --num-actuators 1 \
     --actuator-maxiter 10 --actuator-policy-maxiter 5
   
   # If it looks good, run longer
   python controlled.py --sense 0 10 --auto-actuators --num-actuators 2 \
     --actuator-maxiter 50 --actuator-policy-maxiter 10
   ```

6. **Actuator optimization is slow**: Budget 30-60 minutes for 2 actuators with reasonable iterations

## Next Steps

After running the optimization:

1. **Visualize Results**
   ```bash
   python post.py --T 2000
   ```

2. **Try Different Thresholds**
   ```bash
   # Without actuator optimization
   python controlled.py --auto-sensors --threshold 5.0 --maxiter 50
   python controlled.py --auto-sensors --threshold -5.0 --maxiter 50
   
   # With actuator optimization
   python controlled.py --auto-sensors --auto-actuators --threshold 5.0 \
     --actuator-maxiter 30 --actuator-policy-maxiter 10 --maxiter 30
   ```

3. **Compare Different Sensor Counts**
   ```bash
   for n in 1 2 3 4; do
     python controlled.py --auto-sensors --num-sensors $n --maxiter 50
   done
   ```

4. **Compare Manual vs Optimized Actuators**
   ```bash
   # Manual actuator at x_3
   python controlled.py --auto-sensors --actuate 2 --maxiter 50
   
   # Optimized actuator
   python controlled.py --auto-sensors --auto-actuators --num-actuators 1 \
     --actuator-maxiter 30 --actuator-policy-maxiter 10 --maxiter 30
   
   # Compare the final containment probabilities!
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
- `ACTUATOR_OPTIMIZATION.md` - Detailed guide for actuator optimization feature
- `INTEGRATION_NOTES.md` - Technical details of sensor integration
- `SUMMARY.md` - Implementation summary

## Key Takeaways

### When to Use Actuator Optimization

✅ **Use `--auto-actuators` when:**
- You don't know where to actuate
- You want to maximize performance
- You have computational resources and time
- You're exploring different system configurations

❌ **Don't use `--auto-actuators` when:**
- You have physical constraints on actuator placement
- You need quick results (use manual `--actuate` instead)
- You're just testing the system
- You already know good actuator locations

### Computational Cost Summary

| Operation | Time (rough estimate) | Parallelizable |
|-----------|----------------------|----------------|
| Sensor optimization (2 sensors, 200 iter) | 5-10 min | ✅ Yes |
| Policy optimization (50 iter, 8 CPUs) | 10-20 min | ✅ Yes |
| **Actuator optimization (2 actuators, 50/10 iter)** | **30-60 min** | ❌ No (inner loop) |

**Total for full pipeline with actuator opt**: ~60-90 minutes

