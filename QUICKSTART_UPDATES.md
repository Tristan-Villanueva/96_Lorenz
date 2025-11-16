# QUICKSTART.md Updates Summary

## Changes Made

Updated the QUICKSTART.md guide to comprehensively document the new actuator optimization feature.

### Statistics
- **Lines added**: +264
- **Lines removed**: -18
- **Total lines**: 490 (up from 226)

### Key Additions

#### 1. TL;DR Section - Now with Two Options

**Before**: Single command for sensors + control
**After**: 
- Option 1: Sensors + Control (manual actuators)
- Option 2: Sensors + Actuators + Control (fully automatic) - NEW!

```bash
# NEW Option 2
python controlled.py --auto-sensors --num-sensors 2 --auto-actuators --num-actuators 2 \
  --actuator-maxiter 30 --actuator-policy-maxiter 10 --threshold 0.0 --maxiter 30 --n-cpus 8
```

#### 2. "What Just Happened?" - Added Step 2

New section explaining actuator optimization:
- Nested optimization approach (outer + inner loops)
- Objective: Maximize P(x_3 > 0)
- Computational cost warning (~30-60 min)
- Output file: `actuators_auto_2.npz`

#### 3. Common Usage Patterns - Expanded

Added examples for:
- **Fast testing with actuators**: `--actuator-maxiter 10 --actuator-policy-maxiter 5`
- **Production runs with actuators**: Full iterations
- **Manual vs optimized actuators**: Side-by-side comparison
- **Optimize only actuators**: Using existing sensors
- **Multiple actuators**: Both manual and automatic

#### 4. Console Output Examples

Added two output examples:
1. **Without `--auto-actuators`**: Standard workflow
2. **With `--auto-actuators`**: Shows actuator optimization step with progress

Example snippet:
```
======================================================================
STEP 1: OPTIMAL ACTUATOR PLACEMENT
======================================================================
Optimizing 2 actuator locations...
  [Eval 1] Actuators [20, 30] → P(x_target > threshold) = 0.723456
  [Eval 2] Actuators [2, 15] → P(x_target > threshold) = 0.756234
  ...
```

#### 5. Files Created - Separated Scenarios

**Without `--auto-actuators`**:
- sensors_auto_2.npz
- policy_cm1p0_act2_p2.npz
- post_plots/

**With `--auto-actuators`** (NEW!):
- sensors_auto_2.npz
- **actuators_auto_2.npz** ← New file!
- policy_cm1p0_actopt2_p2.npz
- post_plots/

#### 6. Troubleshooting - New Section

Added "Actuator Optimization Taking Too Long" with quick fixes:
- Use fewer actuators: `--num-actuators 1`
- Reduce iterations: `--actuator-maxiter 10 --actuator-policy-maxiter 5`
- Reduce simulation time: `--T 100`
- Complete fast test example

#### 7. Advanced Options - Actuator Examples

Added actuator optimization to:
- Multiple actuators with custom capacity (manual vs optimized)
- Higher-order polynomial with actuators
- **Full optimization** example combining all features

#### 8. Performance Tips - Actuator-Specific

Added tips:
- Actuator inner loop is NOT parallelized
- Minimal iterations recommendations
- Progressive optimization strategy (start with 1 actuator, then 2, then 3)
- Budget 30-60 minutes for 2 actuators

#### 9. Next Steps - Comparison Example

Added:
- **Compare Manual vs Optimized Actuators**: Run both and compare containment probabilities

#### 10. New Section: "Key Takeaways"

Completely new section with:

**When to Use Actuator Optimization**:
- ✅ Use when: Don't know where to actuate, want max performance, have time
- ❌ Don't use when: Physical constraints, need quick results, just testing

**Computational Cost Summary Table**:
| Operation | Time | Parallelizable |
|-----------|------|----------------|
| Sensor opt (2 sensors, 200 iter) | 5-10 min | ✅ Yes |
| Policy opt (50 iter, 8 CPUs) | 10-20 min | ✅ Yes |
| **Actuator opt (2 actuators, 50/10 iter)** | **30-60 min** | ❌ No |

Total pipeline: ~60-90 minutes

### Cross-References Added

- Link to ACTUATOR_OPTIMIZATION.md in "See Also"
- Multiple references to actuator flags throughout

## Impact

The QUICKSTART.md now serves as a complete guide for:
1. Quick testing with actuator optimization
2. Understanding computational costs
3. Choosing when to use the feature
4. Troubleshooting common issues
5. Progressive optimization strategies

Users can now easily:
- See both workflows at a glance (with/without actuator opt)
- Understand the nested optimization approach
- Make informed decisions about computational resources
- Get started quickly with sensible defaults
