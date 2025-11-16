"""Optimal sensor placement for Lorenz 96 system via mutual information maximization.

This module finds the optimal sensor locations to maximize mutual information
with a target variable (x_3 by default).

Usage:
    python optimal_sensors.py --num-sensors 2 --target-variable 2
"""

from __future__ import annotations

import argparse
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

try:
    import cma
except ImportError as exc:
    raise ImportError("CMA-ES requires the `cma` package. Install it via `pip install cma`.") from exc

from uncontrolled import solve_lorenz96_uncontrolled

# Apply shared plotting style if available
import matplotlib.pyplot as plt
style_path = os.path.join(os.path.dirname(__file__), 'plot_style.mplstyle')
if os.path.exists(style_path):
    plt.style.use(style_path)


@dataclass
class SimulationData:
    """Container for Lorenz 96 simulation data."""
    trajectory: np.ndarray  # shape: (n_times, N)
    times: np.ndarray
    target: np.ndarray  # Target variable time series
    N: int  # Number of state variables


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Optimize sensor locations to maximize mutual information with target variable."
    )
    parser.add_argument("--N", type=int, default=40, help="Number of Lorenz 96 variables")
    parser.add_argument("--F", type=float, default=15.0, help="Forcing parameter")
    parser.add_argument("--transient", type=float, default=100.0, help="Transient time to discard")
    parser.add_argument("--T", type=float, default=10000.0, help="Final simulation time for data collection")
    parser.add_argument("--dt", type=float, default=0.01, help="Time step")
    parser.add_argument(
        "--target-steps",
        type=int,
        default=None,
        help="Number of post-transient time steps to retain (default keeps all available)",
    )
    parser.add_argument("--seed", type=int, default=42, help="Random seed for reproducibility")
    parser.add_argument("--num-bins", type=int, default=100, help="Number of histogram bins per dimension")
    parser.add_argument("--num-sensors", type=int, default=2, help="Number of sensors to place (N_s)")
    parser.add_argument(
        "--target-variable",
        type=int,
        default=2,
        help="Index of target variable (default 2 for x_3, 0-indexed)",
    )
    parser.add_argument("--sigma0", type=float, default=3.0, help="Initial CMA-ES step size")
    parser.add_argument("--maxiter", type=int, default=200, help="Maximum CMA-ES iterations")
    parser.add_argument("--popsize", type=int, default=50, help="Population size for CMA-ES")
    parser.add_argument("--output", type=str, default="sensors.npz", help="Output file for sensor locations")
    return parser.parse_args()


def select_time_samples(
    times: np.ndarray,
    trajectory: np.ndarray,
    target_samples: int
) -> Tuple[np.ndarray, np.ndarray]:
    """Uniformly select `target_samples` entries from the provided time-series arrays."""
    if times.shape[0] != trajectory.shape[0]:
        raise ValueError("`times` and `trajectory` must have matching leading dimension.")
    if target_samples <= 0:
        raise ValueError("`target_samples` must be positive.")
    if trajectory.shape[0] < target_samples:
        raise ValueError(
            f"Requested {target_samples} samples, but only {trajectory.shape[0]} are available. "
            "Increase `T` or reduce `target_samples`."
        )
    if trajectory.shape[0] == target_samples:
        return times, trajectory
    indices = np.linspace(0, trajectory.shape[0] - 1, target_samples, dtype=int)
    return times[indices], trajectory[indices]


def generate_dataset(
    *,
    N: int,
    F: float,
    transient: float,
    T: float,
    dt: float,
    seed: int,
    target_variable: int,
    target_steps: Optional[int] = None,
) -> SimulationData:
    """Run the Lorenz 96 solver and return uniformly sampled trajectory data."""
    if target_steps is not None and target_steps <= 0:
        raise ValueError("`target_steps` must be positive when provided.")
    
    trajectory, times = solve_lorenz96_uncontrolled(
        N=N,
        F=F,
        t0=0.0,
        t1=T,
        dt=dt,
        transient=transient,
        seed=seed,
    )
    
    total_samples = trajectory.shape[0]
    if target_steps is None:
        target_samples = total_samples
    else:
        # Include the initial condition
        target_samples = target_steps + 1
        if target_samples > total_samples:
            raise ValueError(
                f"`target_steps` ({target_steps}) exceeds available samples ({total_samples - 1}) "
                f"for T={T} and dt={dt}. Increase `T` or reduce `target_steps`."
            )
    
    times_sel, trajectory_sel = select_time_samples(times, trajectory, target_samples)
    target = trajectory_sel[:, target_variable]
    
    return SimulationData(
        trajectory=trajectory_sel,
        times=times_sel,
        target=target,
        N=N,
    )


def compute_mutual_information(
    target: np.ndarray,
    sensors: np.ndarray,
    num_bins: int,
) -> float:
    """Compute mutual information I(target; sensors) between scalar target and vector sensor measurements.
    
    Parameters
    ----------
    target : array of shape (n_samples,)
        Target variable time series
    sensors : array of shape (n_samples, n_sensors)
        Sensor measurements
    num_bins : int
        Number of bins for histogram
    
    Returns
    -------
    float
        Mutual information in nats
    """
    if target.ndim != 1:
        raise ValueError("`target` must be one-dimensional.")
    if sensors.ndim != 2:
        raise ValueError("`sensors` must be a 2D array with shape (n_samples, n_sensors).")
    if target.shape[0] != sensors.shape[0]:
        raise ValueError("`target` and `sensors` must share the same number of samples.")
    if num_bins < 2:
        raise ValueError("`num_bins` must be at least 2.")
    
    samples = np.column_stack((target, sensors))
    
    # Determine histogram edges for each dimension
    bin_edges: List[np.ndarray] = []
    for col in samples.T:
        col = np.asarray(col, dtype=float)
        if np.allclose(col.min(), col.max()):
            width = max(1.0, abs(col[0])) * 1e-6 + 1e-6
            edges = np.linspace(col[0] - width, col[0] + width, num_bins + 1)
        else:
            _, edges = np.histogram(col, bins=num_bins)
        bin_edges.append(edges)
    
    # Compute joint histogram
    joint_counts, _ = np.histogramdd(samples, bins=bin_edges)
    total = joint_counts.sum()
    if total <= 0:
        return 0.0
    joint_prob = joint_counts / total
    
    # Marginal distributions
    # P(target)
    sensor_axes = tuple(range(1, joint_prob.ndim))
    p_target = np.sum(joint_prob, axis=sensor_axes)
    # P(sensors)
    p_sensors = np.sum(joint_prob, axis=0)
    
    # Expand for broadcasting
    expanded_p_target = p_target.reshape((-1,) + (1,) * (joint_prob.ndim - 1))
    expanded_p_sensors = p_sensors.reshape((1,) + p_sensors.shape)
    
    # Compute MI: sum_ij p(i,j) log(p(i,j) / (p(i) * p(j)))
    with np.errstate(divide="ignore", invalid="ignore"):
        log_joint = np.where(joint_prob > 0.0, np.log(joint_prob), 0.0)
        log_p_target = np.where(expanded_p_target > 0.0, np.log(expanded_p_target), 0.0)
        log_p_sensors = np.where(expanded_p_sensors > 0.0, np.log(expanded_p_sensors), 0.0)
        log_ratio = log_joint - log_p_target - log_p_sensors
    
    mi = float(np.sum(joint_prob * log_ratio * (joint_prob > 0.0)))
    return max(mi, 0.0)


class SensorMIObjective:
    """Objective function for sensor placement optimization."""
    
    def __init__(
        self,
        data: SimulationData,
        num_bins: int,
        num_sensors: int,
    ) -> None:
        self.data = data
        self.num_bins = num_bins
        self.num_sensors = num_sensors
        if num_sensors <= 0:
            raise ValueError("`num_sensors` must be positive.")
        if num_sensors >= data.N:
            raise ValueError(f"`num_sensors` ({num_sensors}) must be less than N ({data.N}).")
    
    def __call__(self, theta: np.ndarray) -> float:
        """CMA-ES objective: minimize negative MI."""
        if theta.size != self.num_sensors:
            raise ValueError(f"Expected {self.num_sensors} parameters, received {theta.size}.")
        mi = self.mutual_information(theta)
        if not np.isfinite(mi):
            return 1e6
        return -mi
    
    def mutual_information(self, theta: np.ndarray) -> float:
        """Compute MI for given sensor parameters."""
        indices = self._parameters_to_indices(theta)
        sensors = self.data.trajectory[:, indices]
        mi = compute_mutual_information(self.data.target, sensors, self.num_bins)
        return mi
    
    def _parameters_to_indices(self, theta: np.ndarray) -> np.ndarray:
        """Convert continuous parameters to discrete sensor indices.
        
        Maps theta[i] in [0, N) to integer indices, ensuring uniqueness.
        """
        # Clip to valid range
        indices = np.clip(theta, 0, self.data.N - 1)
        # Round to nearest integer
        indices = np.round(indices).astype(int)
        # Ensure uniqueness by adjusting duplicates
        indices = self._ensure_unique_indices(indices)
        return indices
    
    def _ensure_unique_indices(self, indices: np.ndarray) -> np.ndarray:
        """Ensure all indices are unique by adjusting duplicates."""
        unique_indices = []
        used = set()
        
        for idx in indices:
            original_idx = idx
            # If duplicate, search for nearest unused index
            if idx in used:
                # Search in both directions
                for offset in range(1, self.data.N):
                    # Try lower
                    candidate_low = (idx - offset) % self.data.N
                    if candidate_low not in used:
                        idx = candidate_low
                        break
                    # Try higher
                    candidate_high = (idx + offset) % self.data.N
                    if candidate_high not in used:
                        idx = candidate_high
                        break
            
            unique_indices.append(idx)
            used.add(idx)
        
        return np.array(unique_indices, dtype=int)
    
    def decode_parameters(self, theta: np.ndarray) -> np.ndarray:
        """Convert parameters to sensor indices."""
        if theta.size != self.num_sensors:
            raise ValueError(f"Expected {self.num_sensors} parameters, received {theta.size}.")
        return self._parameters_to_indices(theta)


def initial_guess(obj: SensorMIObjective) -> np.ndarray:
    """Generate initial sensor placement (uniformly spaced)."""
    if obj.num_sensors == 1:
        return np.array([obj.data.N / 2.0])
    spacing = obj.data.N / obj.num_sensors
    return (np.arange(obj.num_sensors) + 0.5) * spacing


def optimize_sensor_locations(
    obj: SensorMIObjective,
    *,
    sigma0: float,
    maxiter: int,
    seed: int,
    popsize: int,
) -> Tuple[np.ndarray, float]:
    """Optimize sensor locations using CMA-ES."""
    mean0 = initial_guess(obj)
    lower, upper = 0.0, float(obj.data.N - 1)
    
    es = cma.CMAEvolutionStrategy(
        mean0,
        sigma0,
        {
            "bounds": [lower, upper],
            "maxiter": maxiter,
            "popsize": popsize,
            "verb_disp": 1,
            "seed": seed,
        },
    )
    es.optimize(obj)
    
    if es.result is None:
        raise RuntimeError("CMA-ES did not return a result.")
    
    best_theta = es.result.xbest
    if not np.all(np.isfinite(best_theta)):
        best_theta = mean0
    
    if np.isfinite(es.result.fbest):
        best_mi = -float(es.result.fbest)
    else:
        best_mi = obj.mutual_information(best_theta)
    
    return np.asarray(best_theta, dtype=float), float(best_mi)


def main() -> None:
    args = parse_args()
    
    print("=" * 72)
    print("Optimal Sensor Placement for Lorenz 96 via Mutual Information")
    requested_samples = "all available" if args.target_steps is None else f"{args.target_steps} steps"
    print(f"System: N={args.N}, F={args.F}, sensors={args.num_sensors}")
    print(f"Target variable: x_{args.target_variable + 1} (index {args.target_variable})")
    print(f"Final time T={args.T}, dt={args.dt}, sampling request: {requested_samples}")
    print("=" * 72)
    
    # Generate dataset
    data = generate_dataset(
        N=args.N,
        F=args.F,
        transient=args.transient,
        T=args.T,
        dt=args.dt,
        seed=args.seed,
        target_variable=args.target_variable,
        target_steps=args.target_steps,
    )
    available_steps = data.trajectory.shape[0] - 1
    print(f"Collected {available_steps} steps (total samples including initial: {data.trajectory.shape[0]}).")
    
    # Create objective
    obj = SensorMIObjective(
        data=data,
        num_bins=args.num_bins,
        num_sensors=args.num_sensors,
    )
    
    # Optimize
    print("\nOptimizing sensor locations...")
    best_theta, best_mi = optimize_sensor_locations(
        obj,
        sigma0=args.sigma0,
        maxiter=args.maxiter,
        seed=args.seed,
        popsize=args.popsize,
    )
    
    sensor_indices = obj.decode_parameters(best_theta)
    
    print("\n" + "=" * 72)
    print("Optimization complete!")
    print(f"Mutual information (nats): {best_mi:.6f}")
    print(f"Sensor indices (0-indexed):")
    for i, idx in enumerate(sensor_indices, start=1):
        print(f"  Sensor {i}: x_{idx + 1} (index {idx})")
    print("=" * 72)
    
    # Save results
    np.savez(
        args.output,
        sensor_indices=sensor_indices,
        mutual_information=best_mi,
        num_sensors=args.num_sensors,
        target_variable=args.target_variable,
        N=args.N,
        F=args.F,
        seed=args.seed,
    )
    print(f"\nSensor locations saved to: {args.output}")


if __name__ == "__main__":
    main()

