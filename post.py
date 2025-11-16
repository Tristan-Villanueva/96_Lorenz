"""Post-processing utilities for controlled Lorenz 96 simulations.

Loads optimized policy and generates comparison plots between controlled
and uncontrolled trajectories.
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path
from typing import Dict, Tuple

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

# Import from local directory
sys.path.insert(0, os.path.dirname(__file__))
from controlled import PolynomialPolicy, solve_lorenz96_controlled, ControlledSimulationResult
from uncontrolled import solve_lorenz96_uncontrolled
from kl_bound_cmaes import compute_lb, ensure_thresholds_in_edges

# Try to load user's plot style
style_path = os.path.join(os.path.dirname(__file__), 'plot_style.mplstyle')
if os.path.exists(style_path):
    plt.style.use(style_path)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Post-process controlled Lorenz 96 simulations.")
    parser.add_argument(
        "--policy",
        type=str,
        default="policy.npz",
        help="Path to .npz file containing controller parameters and simulation config.",
    )
    parser.add_argument(
        "--c-max",
        type=float,
        default=None,
        help="Control scaling factor for forward simulation (defaults to value from policy).",
    )
    parser.add_argument(
        "--dt",
        type=float,
        default=None,
        help="Override simulation time step (defaults to value from policy).",
    )
    parser.add_argument(
        "--T",
        type=float,
        default=None,
        help="Override total simulation time (defaults to value from policy).",
    )
    parser.add_argument(
        "--transient",
        type=float,
        default=None,
        help="Override transient time (defaults to value from policy).",
    )
    parser.add_argument(
        "--bin-width",
        type=float,
        default=1.0,
        help="Width of bins for histogram.",
    )
    parser.add_argument(
        "--lb-maxiter",
        type=int,
        default=100,
        help="CMA-ES maximum iterations for computing lower bounds.",
    )
    parser.add_argument(
        "--num-thresholds",
        type=int,
        default=100,
        help="Number of thresholds for containment probabilities.",
    )
    parser.add_argument(
        "--threshold-min",
        type=float,
        default=None,
        help="Minimum threshold for containment evaluation (auto if None).",
    )
    parser.add_argument(
        "--threshold-max",
        type=float,
        default=None,
        help="Maximum threshold for containment evaluation (auto if None).",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="post_plots",
        help="Directory to save generated plots.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Override random seed (defaults to value from policy).",
    )
    return parser.parse_args()


def load_policy_bundle(policy_path: str) -> Dict:
    """Load policy and configuration from .npz file."""
    if not os.path.exists(policy_path):
        raise FileNotFoundError(f"Policy file not found: {policy_path}")
    
    data = np.load(policy_path, allow_pickle=True)
    
    # Extract config
    config_item = data["config"]
    if isinstance(config_item, np.ndarray) and config_item.shape == ():
        config = config_item.item()
    else:
        config = dict(config_item)
    
    # Create policy
    policy = PolynomialPolicy(
        actuated_dims=list(data["actuated_dims"]),
        sensed_dims=list(data["sensed_dims"]),
        poly_order=int(data["poly_order"]),
        N=int(data["N"]),
    )
    policy.set_params_from_array(data["params"])
    
    return {
        "policy": policy,
        "config": config,
        "params": data["params"],
        "coefficients": data["coefficients"],
        "alphas": data["alphas"],
        "actuated_dims": data["actuated_dims"],
        "sensed_dims": data["sensed_dims"],
        "poly_order": data["poly_order"],
        "N": data["N"],
    }


def compute_containment_probabilities(
    target_values: np.ndarray,
    thresholds: np.ndarray,
    bin_width: float,
    lb_maxiter: int,
    seed: int,
) -> Tuple[np.ndarray, np.ndarray]:
    """Compute true probabilities and lower bounds for multiple thresholds.
    
    Returns
    -------
    true_probs : array
        P(target > threshold) for each threshold
    lower_bounds : array
        Lower bounds for each threshold
    """
    true_probs = []
    lower_bounds = []
    
    for t in thresholds:
        # True probability
        true_prob = float(np.mean(target_values > t))
        true_probs.append(true_prob)
        
        # Lower bound
        target_max = float(np.max(target_values))
        target_min = float(np.min(target_values))
        lower = min(target_min, t)
        upper = max(target_max, t)
        
        num_bins = max(10, int(np.ceil((upper - lower) / bin_width)))
        bin_edges = np.linspace(lower, lower + num_bins * bin_width, num_bins + 1, dtype=float)
        bin_edges = ensure_thresholds_in_edges(bin_edges=bin_edges, thresholds=[t])
        num_bins = len(bin_edges) - 1
        
        try:
            lb_data = compute_lb(
                energy=target_values,
                thresholds=np.array([t]),
                num_bins=num_bins,
                bin_edges=bin_edges,
                seed=seed,
                critical_thresholds=[t],
                lb_maxiter=lb_maxiter,
            )
            lower_bound = float(lb_data["lower_bounds"][0])
        except:
            lower_bound = 0.0
        
        lower_bounds.append(lower_bound)
    
    return np.array(true_probs), np.array(lower_bounds)


def main():
    args = parse_args()
    
    print("=" * 70)
    print("Lorenz 96 Post-Processing")
    print("=" * 70)
    
    # Load policy
    print(f"\nLoading policy from: {args.policy}")
    bundle = load_policy_bundle(args.policy)
    policy = bundle["policy"]
    config = bundle["config"]
    
    print(f"  Policy order: {bundle['poly_order']}")
    print(f"  Actuated dims: {list(bundle['actuated_dims'])}")
    print(f"  Sensed dims: {list(bundle['sensed_dims'])}")
    print(f"  Number of parameters: {policy.num_params}")
    
    # Simulation parameters
    N = int(bundle["N"])
    F = float(config.get("F", 15.0))
    dt = args.dt if args.dt is not None else float(config.get("dt", 0.01))
    T = args.T if args.T is not None else float(config.get("T", 1000.0))
    transient = args.transient if args.transient is not None else float(config.get("transient", 100.0))
    c_max = args.c_max if args.c_max is not None else float(config.get("c_max", 1.0))
    target_variable = int(config.get("target_variable", 2))
    seed = args.seed if args.seed is not None else int(config.get("seed", 42))
    threshold = float(config.get("threshold", 0.0))
    
    print(f"\nSimulation parameters:")
    print(f"  N = {N}, F = {F}")
    print(f"  dt = {dt}, T = {T}, transient = {transient}")
    print(f"  c_max = {c_max}")
    print(f"  Target variable: x_{target_variable + 1} (index {target_variable})")
    print(f"  Threshold: {threshold}")
    print(f"  Seed: {seed}")
    
    # Run controlled simulation
    print("\nRunning controlled simulation...")
    controlled_result = solve_lorenz96_controlled(
        N=N,
        F=F,
        dt=dt,
        T=T,
        transient=transient,
        policy=policy,
        c_max=c_max,
        target_variable=target_variable,
        seed=seed,
    )
    print(f"  Controlled simulation complete. Trajectory shape: {controlled_result.trajectory.shape}")
    
    # Run uncontrolled simulation
    print("\nRunning uncontrolled simulation...")
    uncontrolled_trajectory, uncontrolled_times = solve_lorenz96_uncontrolled(
        N=N,
        F=F,
        t0=0.0,
        t1=T,
        dt=dt,
        transient=transient,
        seed=seed,
    )
    uncontrolled_target = uncontrolled_trajectory[:, target_variable]
    print(f"  Uncontrolled simulation complete. Trajectory shape: {uncontrolled_trajectory.shape}")
    
    # Statistics
    print("\n" + "=" * 70)
    print("RESULTS")
    print("=" * 70)
    print(f"\nTarget variable: x_{target_variable + 1}")
    print(f"\nUncontrolled:")
    print(f"  Mean: {np.mean(uncontrolled_target):.4f}")
    print(f"  Std: {np.std(uncontrolled_target):.4f}")
    print(f"  Min: {np.min(uncontrolled_target):.4f}")
    print(f"  Max: {np.max(uncontrolled_target):.4f}")
    print(f"  P(x_{target_variable + 1} > {threshold}): {np.mean(uncontrolled_target > threshold):.4f}")
    
    print(f"\nControlled:")
    print(f"  Mean: {np.mean(controlled_result.target):.4f}")
    print(f"  Std: {np.std(controlled_result.target):.4f}")
    print(f"  Min: {np.min(controlled_result.target):.4f}")
    print(f"  Max: {np.max(controlled_result.target):.4f}")
    print(f"  P(x_{target_variable + 1} > {threshold}): {np.mean(controlled_result.target > threshold):.4f}")
    
    control_rms = np.sqrt(np.mean(np.sum(controlled_result.controls**2, axis=1)))
    print(f"\nControl effort:")
    print(f"  RMS: {control_rms:.4f}")
    print(f"  Mean ||u||²: {np.mean(np.sum(controlled_result.controls**2, axis=1)):.4f}")
    
    # Create output directory
    output_dir = Path(args.output_dir)
    output_dir.mkdir(exist_ok=True)
    print(f"\nSaving plots to: {output_dir}")
    
    # Plot 1: Time series comparison
    print("  Generating time series plot...")
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    
    plot_steps = min(10000, len(controlled_result.times))
    t_plot = controlled_result.times[:plot_steps]
    
    axes[0].plot(t_plot, uncontrolled_target[:plot_steps], linewidth=0.5, label='Uncontrolled', alpha=0.8)
    axes[0].plot(t_plot, controlled_result.target[:plot_steps], linewidth=0.5, label='Controlled', alpha=0.8)
    axes[0].axhline(y=threshold, color='r', linestyle='--', linewidth=1, alpha=0.7, label=f'Threshold = {threshold}')
    axes[0].set_ylabel(f'$x_{{{target_variable + 1}}}$')
    axes[0].set_title(f'Time Series Comparison: $x_{{{target_variable + 1}}}$')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    
    # Control signal
    control_norm = np.linalg.norm(controlled_result.controls[:plot_steps], axis=1)
    axes[1].plot(t_plot, control_norm, linewidth=0.5, color='green')
    axes[1].set_xlabel('Time')
    axes[1].set_ylabel(r'$||u||$')
    axes[1].set_title('Control Magnitude')
    axes[1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_dir / 'timeseries_comparison.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # Plot 2: Histograms
    print("  Generating histogram plot...")
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
    bins = np.linspace(
        min(np.min(uncontrolled_target), np.min(controlled_result.target)),
        max(np.max(uncontrolled_target), np.max(controlled_result.target)),
        100
    )
    
    axes[0].hist(uncontrolled_target, bins=bins, density=True, alpha=0.7, edgecolor='black', label='Uncontrolled')
    axes[0].axvline(x=threshold, color='r', linestyle='--', linewidth=2, label=f'Threshold = {threshold}')
    axes[0].set_xlabel(f'$x_{{{target_variable + 1}}}$')
    axes[0].set_ylabel('Probability Density')
    axes[0].set_title(f'Uncontrolled Distribution of $x_{{{target_variable + 1}}}$')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    
    axes[1].hist(controlled_result.target, bins=bins, density=True, alpha=0.7, edgecolor='black', label='Controlled', color='orange')
    axes[1].axvline(x=threshold, color='r', linestyle='--', linewidth=2, label=f'Threshold = {threshold}')
    axes[1].set_xlabel(f'$x_{{{target_variable + 1}}}$')
    axes[1].set_ylabel('Probability Density')
    axes[1].set_title(f'Controlled Distribution of $x_{{{target_variable + 1}}}$')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_dir / 'histogram_comparison.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # Plot 3: Phase portrait (first 3 variables)
    print("  Generating phase portrait...")
    fig = plt.figure(figsize=(14, 6))
    
    plot_3d_steps = min(5000, len(controlled_result.times))
    
    ax1 = fig.add_subplot(121, projection='3d')
    ax1.plot(
        uncontrolled_trajectory[:plot_3d_steps, 0],
        uncontrolled_trajectory[:plot_3d_steps, 1],
        uncontrolled_trajectory[:plot_3d_steps, 2],
        linewidth=0.3, alpha=0.6
    )
    ax1.set_xlabel('$x_1$')
    ax1.set_ylabel('$x_2$')
    ax1.set_zlabel('$x_3$')
    ax1.set_title('Uncontrolled Phase Portrait')
    
    ax2 = fig.add_subplot(122, projection='3d')
    ax2.plot(
        controlled_result.trajectory[:plot_3d_steps, 0],
        controlled_result.trajectory[:plot_3d_steps, 1],
        controlled_result.trajectory[:plot_3d_steps, 2],
        linewidth=0.3, alpha=0.6, color='orange'
    )
    ax2.set_xlabel('$x_1$')
    ax2.set_ylabel('$x_2$')
    ax2.set_zlabel('$x_3$')
    ax2.set_title('Controlled Phase Portrait')
    
    plt.tight_layout()
    plt.savefig(output_dir / 'phase_portrait_3d.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # Plot 4: Containment probabilities vs threshold
    print("  Computing containment probabilities...")
    
    if args.threshold_min is None:
        threshold_min = min(np.min(uncontrolled_target), np.min(controlled_result.target))
    else:
        threshold_min = args.threshold_min
    
    if args.threshold_max is None:
        threshold_max = max(np.max(uncontrolled_target), np.max(controlled_result.target))
    else:
        threshold_max = args.threshold_max
    
    thresholds = np.linspace(threshold_min, threshold_max, args.num_thresholds)
    
    uncontrolled_probs, uncontrolled_lbs = compute_containment_probabilities(
        uncontrolled_target, thresholds, args.bin_width, args.lb_maxiter, seed
    )
    controlled_probs, controlled_lbs = compute_containment_probabilities(
        controlled_result.target, thresholds, args.bin_width, args.lb_maxiter, seed
    )
    
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.plot(thresholds, uncontrolled_probs, label='Uncontrolled (True)', linewidth=2)
    ax.plot(thresholds, uncontrolled_lbs, label='Uncontrolled (LB)', linewidth=2, linestyle='--', alpha=0.7)
    ax.plot(thresholds, controlled_probs, label='Controlled (True)', linewidth=2)
    ax.plot(thresholds, controlled_lbs, label='Controlled (LB)', linewidth=2, linestyle='--', alpha=0.7)
    ax.axvline(x=threshold, color='r', linestyle=':', linewidth=2, alpha=0.5, label=f'Design threshold = {threshold}')
    ax.set_xlabel('Threshold')
    ax.set_ylabel(f'$P(x_{{{target_variable + 1}}} > \\mathrm{{threshold}})$')
    ax.set_title(f'Containment Probabilities vs Threshold')
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(output_dir / 'containment_probabilities.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # Plot 5: Control effort breakdown
    print("  Generating control effort plot...")
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    
    # Individual control components
    for dim in bundle['actuated_dims']:
        axes[0].plot(
            t_plot,
            controlled_result.controls[:plot_steps, dim],
            linewidth=0.5,
            label=f'$u_{{{dim + 1}}}$',
            alpha=0.8
        )
    axes[0].set_ylabel('Control')
    axes[0].set_title('Individual Control Components')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    
    # Sensor measurements
    for i, dim in enumerate(bundle['sensed_dims']):
        axes[1].plot(
            t_plot,
            controlled_result.sensor_trace[:plot_steps, i],
            linewidth=0.5,
            label=f'$x_{{{dim + 1}}}$ (sensor {i+1})',
            alpha=0.8
        )
    axes[1].set_xlabel('Time')
    axes[1].set_ylabel('Sensor Reading')
    axes[1].set_title('Sensor Measurements')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_dir / 'control_and_sensors.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    print("\n" + "=" * 70)
    print("Post-processing complete!")
    print(f"All plots saved to: {output_dir}")
    print("=" * 70)


if __name__ == "__main__":
    main()

