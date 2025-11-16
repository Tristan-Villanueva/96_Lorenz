"""Uncontrolled Lorenz 96 system simulation.

This module provides:
    * Lorenz 96 dynamics with constant forcing F
    * RK23-based solver using scipy's solve_ivp
    * Helper utilities for generating trajectory data
"""

from __future__ import annotations

import argparse
import os
from dataclasses import dataclass
from typing import Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp

# Apply shared plotting style if available
style_path = os.path.join(os.path.dirname(__file__), 'plot_style.mplstyle')
if os.path.exists(style_path):
    plt.style.use(style_path)


@dataclass
class L96SimulationArgs:
    """Arguments for the Lorenz 96 system simulation."""
    # System parameters
    N: int = 40  # Number of variables
    F: float = 15.0  # Forcing parameter
    
    # Initial conditions
    x0: Optional[np.ndarray] = None  # Initial state vector
    
    # Time parameters
    t0: float = 0.0
    t1: float = 1000.0
    dt: float = 0.01
    transient: float = 100.0
    
    # Random seed
    seed: int = 42


def lorenz96_dynamics(x: np.ndarray, F: float) -> np.ndarray:
    """Lorenz 96 system dynamics (uncontrolled).
    
    dx_i/dt = (x_{i+1} - x_{i-2}) * x_{i-1} - x_i + F
    
    Parameters
    ----------
    x : array of shape (N,)
        State vector
    F : float
        Forcing parameter
    
    Returns
    -------
    array of shape (N,)
        Time derivatives dx/dt
    """
    N = len(x)
    dxdt = np.zeros(N)
    
    # Vectorized computation with periodic boundary conditions
    dxdt = (np.roll(x, -1) - np.roll(x, 2)) * np.roll(x, 1) - x + F
    
    return dxdt


def solve_lorenz96_uncontrolled(
    *,
    N: int = 40,
    F: float = 15.0,
    x0: Optional[np.ndarray] = None,
    t0: float = 0.0,
    t1: float = 1000.0,
    dt: float = 0.01,
    transient: float = 100.0,
    seed: int = 42,
) -> Tuple[np.ndarray, np.ndarray]:
    """Simulate uncontrolled Lorenz 96 system.
    
    Parameters
    ----------
    N : int
        Number of state variables
    F : float
        Forcing parameter
    x0 : array-like, optional
        Initial conditions of shape (N,). If None, uses F + small perturbation.
    t0 : float
        Start time
    t1 : float
        End time (after transient)
    dt : float
        Time step for output sampling
    transient : float
        Transient time to discard
    seed : int
        Random seed for initial condition perturbation
    
    Returns
    -------
    trajectory : ndarray of shape (n_times, N)
        State trajectories
    t : ndarray
        Time array (starting from 0 after transient)
    """
    # Set up initial condition
    if x0 is None:
        rng = np.random.default_rng(seed)
        x0 = F * np.ones(N)
        x0[0] += 0.01  # Small perturbation to first variable
    else:
        x0 = np.asarray(x0, dtype=float)
        if x0.shape != (N,):
            raise ValueError(f"Initial condition must have shape ({N},), got {x0.shape}")
    
    def lorenz96_ode(t: float, x: np.ndarray) -> np.ndarray:
        """ODE right-hand side in scipy format: f(t, x)."""
        return lorenz96_dynamics(x, F)
    
    # Integrate through transient
    if transient > 0:
        sol_transient = solve_ivp(
            lorenz96_ode,
            t_span=(0, transient),
            y0=x0,
            method='RK23',
            rtol=1e-4,
            atol=1e-6,
            dense_output=False,
        )
        if not sol_transient.success:
            raise RuntimeError("Transient integration failed")
        x0 = sol_transient.y[:, -1]
    
    # Data collection phase
    num_steps = int(np.round((t1 - t0) / dt))
    t_eval = np.linspace(0.0, num_steps * dt, num_steps + 1)
    
    sol = solve_ivp(
        lorenz96_ode,
        t_span=(0, t_eval[-1]),
        y0=x0,
        method='RK23',
        t_eval=t_eval,
        rtol=1e-4,
        atol=1e-6,
        dense_output=False,
    )
    
    if not sol.success:
        raise RuntimeError("ODE solver failed during data collection")
    
    trajectory = sol.y.T  # shape: (n_times, N)
    times = sol.t
    
    return trajectory, times


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Simulate uncontrolled Lorenz 96 system."
    )
    parser.add_argument("--N", type=int, default=40, help="Number of variables")
    parser.add_argument("--F", type=float, default=15.0, help="Forcing parameter")
    parser.add_argument("--t1", type=float, default=1000.0, help="Final time after transient")
    parser.add_argument("--dt", type=float, default=0.01, help="Time step")
    parser.add_argument("--transient", type=float, default=100.0, help="Transient time to discard")
    parser.add_argument("--seed", type=int, default=42, help="Random seed")
    parser.add_argument("--output", type=str, default="lorenz96_uncontrolled.npz", help="Output file")
    parser.add_argument("--plot", action="store_true", help="Generate diagnostic plots")
    return parser.parse_args()


def main():
    args = parse_args()
    
    print("=" * 70)
    print("LORENZ 96 UNCONTROLLED SIMULATION")
    print("=" * 70)
    print(f"Number of variables: N = {args.N}")
    print(f"Forcing parameter: F = {args.F}")
    print(f"Time: [0, {args.t1}], dt = {args.dt}")
    print(f"Transient: {args.transient}")
    print("=" * 70)
    
    # Run simulation
    print("\nSimulating Lorenz 96 system...")
    trajectory, times = solve_lorenz96_uncontrolled(
        N=args.N,
        F=args.F,
        t0=0.0,
        t1=args.t1,
        dt=args.dt,
        transient=args.transient,
        seed=args.seed,
    )
    
    print(f"Simulation complete. Trajectory shape: {trajectory.shape}")
    print(f"Time span: [{times[0]:.2f}, {times[-1]:.2f}]")
    
    # Compute statistics
    x3_values = trajectory[:, 2]  # x_3 (index 2)
    print(f"\nStatistics for x_3:")
    print(f"  Mean: {np.mean(x3_values):.4f}")
    print(f"  Std: {np.std(x3_values):.4f}")
    print(f"  Min: {np.min(x3_values):.4f}")
    print(f"  Max: {np.max(x3_values):.4f}")
    print(f"  P(x_3 > 0): {np.mean(x3_values > 0):.4f}")
    
    # Save results
    np.savez(
        args.output,
        trajectory=trajectory,
        times=times,
        N=args.N,
        F=args.F,
        dt=args.dt,
        transient=args.transient,
        seed=args.seed,
    )
    print(f"\nResults saved to: {args.output}")
    
    # Generate plots if requested
    if args.plot:
        print("\nGenerating plots...")
        
        # Plot first three variables in 3D
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], linewidth=0.5)
        ax.set_xlabel(r'$x_1$')
        ax.set_ylabel(r'$x_2$')
        ax.set_zlabel(r'$x_3$')
        ax.set_title('Lorenz 96 Attractor (First 3 Variables)')
        plt.savefig('lorenz96_3d.png', dpi=300, bbox_inches='tight')
        plt.close()
        print("  Saved: lorenz96_3d.png")
        
        # Plot x_3 time series
        fig, ax = plt.subplots(figsize=(12, 4))
        ax.plot(times[:min(5000, len(times))], x3_values[:min(5000, len(times))], linewidth=0.5)
        ax.axhline(y=0, color='r', linestyle='--', linewidth=1, alpha=0.7)
        ax.set_xlabel('Time')
        ax.set_ylabel(r'$x_3$')
        ax.set_title(r'Time Series of $x_3$')
        ax.grid(True, alpha=0.3)
        plt.savefig('lorenz96_x3_timeseries.png', dpi=300, bbox_inches='tight')
        plt.close()
        print("  Saved: lorenz96_x3_timeseries.png")
        
        # Histogram of x_3
        fig, ax = plt.subplots(figsize=(8, 6))
        ax.hist(x3_values, bins=100, density=True, alpha=0.7, edgecolor='black')
        ax.axvline(x=0, color='r', linestyle='--', linewidth=2, label=r'$x_3 = 0$')
        ax.set_xlabel(r'$x_3$')
        ax.set_ylabel('Probability Density')
        ax.set_title(r'Distribution of $x_3$')
        ax.legend()
        ax.grid(True, alpha=0.3)
        plt.savefig('lorenz96_x3_histogram.png', dpi=300, bbox_inches='tight')
        plt.close()
        print("  Saved: lorenz96_x3_histogram.png")
        
        print("\nAll plots saved successfully!")


if __name__ == "__main__":
    main()

