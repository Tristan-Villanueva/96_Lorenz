"""Controlled Lorenz 96 system simulations with polynomial control policies.

This module provides:
    * Integrated optimal sensor placement and control policy optimization
    * A polynomial-basis controller (`PolynomialPolicy`) with configurable order
    * RK23-based simulator for the Lorenz 96 system with control inputs
    * Helper utilities for packaging results
    * CMA-ES optimization to maximize lower bound on P(x_target > t_threshold)
"""

from __future__ import annotations

import argparse
import multiprocessing as mp
import os
import pickle
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy.integrate import solve_ivp

try:
    import cma
except ImportError as exc:
    raise ImportError("CMA-ES utilities require the `cma` package. Install via `pip install cma`.") from exc

# Import from local directory
sys.path.insert(0, os.path.dirname(__file__))
from kl_bound_cmaes import compute_lb, ensure_thresholds_in_edges
from uncontrolled import solve_lorenz96_uncontrolled

ArrayLike = Sequence[float]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Optimize polynomial control policy for Lorenz 96 system."
    )
    parser.add_argument(
        "--actuate",
        type=int,
        nargs="+",
        default=[2],
        help="State indices to actuate (space-separated, 0-indexed). Default: 2 (x_3)",
    )
    parser.add_argument(
        "--sensor-file",
        type=str,
        default="sensors.npz",
        help="File containing optimal sensor indices from optimal_sensors.py",
    )
    parser.add_argument(
        "--sense",
        type=int,
        nargs="+",
        default=None,
        help="State indices to sense (overrides sensor-file and auto-optimization)",
    )
    parser.add_argument(
        "--auto-sensors",
        action="store_true",
        help="Automatically optimize sensor locations if not provided via --sense or --sensor-file",
    )
    parser.add_argument(
        "--num-sensors",
        type=int,
        default=2,
        help="Number of sensors for auto-optimization (default: 2)",
    )
    parser.add_argument(
        "--sensor-T",
        type=float,
        default=10000.0,
        help="Simulation time for sensor optimization data collection (default: 10000)",
    )
    parser.add_argument(
        "--sensor-maxiter",
        type=int,
        default=200,
        help="CMA-ES iterations for sensor optimization (default: 200)",
    )
    parser.add_argument(
        "--num-bins",
        type=int,
        default=100,
        help="Number of histogram bins for MI computation (default: 100)",
    )
    parser.add_argument(
        "--poly-order",
        type=int,
        default=2,
        choices=[1, 2, 3],
        help="Polynomial order for controller.",
    )
    parser.add_argument(
        "--threshold",
        type=float,
        default=0.0,
        help="Threshold t for maximizing P(x_target > t).",
    )
    parser.add_argument(
        "--target-variable",
        type=int,
        default=2,
        help="Index of target variable (default 2 for x_3, 0-indexed).",
    )
    parser.add_argument(
        "--c-max",
        type=float,
        default=1.0,
        help="Actuation capacity (control bound).",
    )
    parser.add_argument(
        "--lambda-control",
        type=float,
        default=0.0,
        help="Control penalty weight.",
    )
    parser.add_argument(
        "--bin-width",
        type=float,
        default=1.0,
        help="Histogram bin width for target distribution.",
    )
    parser.add_argument(
        "--N",
        type=int,
        default=40,
        help="Number of Lorenz 96 variables.",
    )
    parser.add_argument(
        "--F",
        type=float,
        default=15.0,
        help="Forcing parameter.",
    )
    parser.add_argument(
        "--dt",
        type=float,
        default=0.01,
        help="Time step size.",
    )
    parser.add_argument(
        "--T",
        type=float,
        default=1000.0,
        help="Total simulation time after transient.",
    )
    parser.add_argument(
        "--transient",
        type=float,
        default=100.0,
        help="Transient time to discard.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Random seed.",
    )
    parser.add_argument(
        "--sigma0",
        type=float,
        default=0.3,
        help="CMA-ES initial step size.",
    )
    parser.add_argument(
        "--maxiter",
        type=int,
        default=50,
        help="CMA-ES maximum iterations for policy optimization.",
    )
    parser.add_argument(
        "--lb-maxiter",
        type=int,
        default=150,
        help="CMA-ES maximum iterations for computing lower bounds.",
    )
    parser.add_argument(
        "--n-cpus",
        type=int,
        default=1,
        help="Number of CPUs for parallel evaluation (1 = serial).",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="policy.npz",
        help="Path to save optimized policy.",
    )
    return parser.parse_args()


# --------------------------------------------------------------------------- #
# Polynomial control policy
# --------------------------------------------------------------------------- #


@dataclass
class PolynomialPolicy:
    """Polynomial-basis controller mapping sensor readings to control actions.
    
    For each actuated coordinate, the policy evaluates a bounded control:
    u_i = c_max * tanh(Σ aᵢⱼ × (product of (αᵢ × sensed coordinates) with total degree ≤ poly_order))
    
    Parameters
    ----------
    actuated_dims:
        List of dimension indices to actuate.
    sensed_dims:
        List of dimension indices to sense.
    poly_order:
        Maximum polynomial order (1, 2, or 3).
    N:
        Total number of state variables in Lorenz 96.
    """
    
    actuated_dims: List[int]
    sensed_dims: List[int]
    poly_order: int
    N: int
    _coefficients: np.ndarray = field(init=False, repr=False)
    _alphas: np.ndarray = field(init=False, repr=False)
    _monomial_powers: List[Tuple[int, ...]] = field(init=False, repr=False)
    
    def __post_init__(self) -> None:
        if self.poly_order not in [1, 2, 3]:
            raise ValueError("`poly_order` must be 1, 2, or 3.")
        for dim in self.sensed_dims:
            if not (0 <= dim < self.N):
                raise ValueError(f"Sensed dimension {dim} must be in [0, {self.N-1}].")
        for dim in self.actuated_dims:
            if not (0 <= dim < self.N):
                raise ValueError(f"Actuated dimension {dim} must be in [0, {self.N-1}].")
        
        # Generate all monomial terms up to poly_order
        self._monomial_powers = self._generate_monomials(len(self.sensed_dims), self.poly_order)
        
        # Number of coefficients per actuated coordinate = number of monomial terms
        num_coeffs_per_dim = len(self._monomial_powers)
        num_params = len(self.actuated_dims) * num_coeffs_per_dim
        self._coefficients = np.zeros(num_params, dtype=float)
        
        # One alpha scaling parameter per actuated dimension
        self._alphas = np.ones(len(self.actuated_dims), dtype=float)
    
    @staticmethod
    def _generate_monomials(n_vars: int, max_order: int) -> List[Tuple[int, ...]]:
        """Generate all monomial powers with total degree <= max_order."""
        monomials = []
        
        def generate_recursive(current_powers: List[int], remaining_order: int, var_idx: int):
            if var_idx == n_vars:
                monomials.append(tuple(current_powers))
                return
            
            for power in range(remaining_order + 1):
                generate_recursive(
                    current_powers + [power],
                    remaining_order - power,
                    var_idx + 1
                )
        
        generate_recursive([], max_order, 0)
        return monomials
    
    @property
    def num_params(self) -> int:
        return self._coefficients.size + self._alphas.size
    
    def _evaluate_polynomial(self, sensors: np.ndarray, dim_idx: int) -> float:
        """Evaluate polynomial for a specific actuated dimension."""
        num_coeffs_per_dim = len(self._monomial_powers)
        start_idx = dim_idx * num_coeffs_per_dim
        coeffs = self._coefficients[start_idx:start_idx + num_coeffs_per_dim]
        
        # Scale sensors by alpha for this actuated dimension
        alpha = self._alphas[dim_idx]
        scaled_sensors = alpha * sensors
        
        # Evaluate all monomial terms
        result = 0.0
        for coeff, powers in zip(coeffs, self._monomial_powers):
            # Compute monomial: product of scaled_sensors[i]^powers[i]
            term = 1.0
            for sensor_val, power in zip(scaled_sensors, powers):
                if power > 0:
                    term *= sensor_val ** power
            result += coeff * term
        return result
    
    def get_control(self, sensors: np.ndarray, c_max: float) -> np.ndarray:
        """Compute bounded control actions using tanh: u = c_max * tanh(polynomial(alpha * s)).
        
        Parameters
        ----------
        sensors: array of sensed values (length = len(self.sensed_dims))
        c_max: maximum control magnitude
        
        Returns
        -------
        N-dimensional control vector with zeros for non-actuated dims.
        """
        if c_max <= 0.0:
            raise ValueError("`c_max` must be positive.")
        
        sensors = np.asarray(sensors, dtype=float)
        if sensors.shape != (len(self.sensed_dims),):
            raise ValueError(
                f"Expected sensor vector of shape ({len(self.sensed_dims)},), got {sensors.shape}."
            )
        
        control = np.zeros(self.N, dtype=float)
        for idx, dim in enumerate(self.actuated_dims):
            raw = self._evaluate_polynomial(sensors, idx)
            control[dim] = float(c_max * np.tanh(raw))  # Bounded via tanh
        
        return control
    
    # ---- Parameter vectorisation ---------------------------------------------
    
    def get_params_as_array(self) -> np.ndarray:
        """Return polynomial coefficients and alphas as a 1D array."""
        return np.concatenate([self._coefficients, self._alphas])
    
    def set_params_from_array(self, params: np.ndarray) -> None:
        """Load coefficients and alphas from a 1D parameter array."""
        params = np.asarray(params, dtype=float)
        expected_size = self._coefficients.size + self._alphas.size
        if params.size != expected_size:
            raise ValueError(
                f"Expected parameter vector of size {expected_size}, received {params.size}."
            )
        n_coeffs = self._coefficients.size
        self._coefficients = np.array(params[:n_coeffs], dtype=float, copy=True)
        self._alphas = np.array(params[n_coeffs:], dtype=float, copy=True)
    
    def copy(self) -> "PolynomialPolicy":
        """Create a deep copy of the policy."""
        clone = PolynomialPolicy(
            actuated_dims=list(self.actuated_dims),
            sensed_dims=list(self.sensed_dims),
            poly_order=self.poly_order,
            N=self.N,
        )
        clone._coefficients = np.array(self._coefficients, copy=True)
        clone._alphas = np.array(self._alphas, copy=True)
        return clone


# --------------------------------------------------------------------------- #
# Controlled Lorenz 96 simulation with RK23
# --------------------------------------------------------------------------- #


@dataclass
class ControlledSimulationResult:
    """Container for controlled Lorenz 96 simulation outputs."""
    
    times: np.ndarray
    trajectory: np.ndarray  # shape (n_times, N)
    target: np.ndarray      # Target variable values
    controls: np.ndarray    # shape (n_times, N)
    sensor_trace: np.ndarray
    metadata: Dict[str, Any]
    
    def to_dict(self) -> Dict[str, np.ndarray]:
        return {
            "times": self.times,
            "trajectory": self.trajectory,
            "target": self.target,
            "controls": self.controls,
            "sensor_trace": self.sensor_trace,
        }


def solve_lorenz96_controlled(
    *,
    N: int = 40,
    F: float = 15.0,
    dt: float,
    T: float,
    transient: float,
    policy: PolynomialPolicy,
    c_max: float,
    target_variable: int = 2,
    seed: int = 42,
    initial_condition: Optional[np.ndarray] = None,
) -> ControlledSimulationResult:
    """Simulate the Lorenz 96 system with polynomial control using scipy's adaptive integrator.
    
    Equations:
        dx_i/dt = (x_{i+1} - x_{i-2}) * x_{i-1} - x_i + F + u_i
    
    Parameters
    ----------
    N : int
        Number of Lorenz 96 variables
    F : float
        Forcing parameter
    dt : float
        Time step size for output sampling
    T : float
        Total simulation time after transient
    transient : float
        Transient time to discard
    policy : PolynomialPolicy
        Polynomial controller
    c_max : float
        Control scaling factor
    target_variable : int
        Index of target variable to track
    seed : int
        Random seed for initial condition
    initial_condition : array, optional
        Initial state vector of shape (N,)
    """
    
    if dt <= 0 or T < 0 or transient < 0:
        raise ValueError("`dt`, `T`, and `transient` must be non-negative with dt > 0.")
    
    # Initial condition
    if initial_condition is not None:
        state0 = np.asarray(initial_condition, dtype=float)
        if state0.shape != (N,):
            raise ValueError(f"`initial_condition` must have shape ({N},).")
    else:
        rng = np.random.default_rng(seed)
        state0 = F * np.ones(N)
        state0[0] += 0.01  # Small perturbation
    
    def lorenz96_controlled_rhs(t: float, s: np.ndarray) -> np.ndarray:
        """Right-hand side of controlled Lorenz 96 system."""
        # Uncontrolled dynamics
        dxdt = (np.roll(s, -1) - np.roll(s, 2)) * np.roll(s, 1) - s + F
        
        # Add control
        sensors = s[policy.sensed_dims]
        control = policy.get_control(sensors, c_max)
        dxdt += control
        
        return dxdt
    
    # Integrate through transient
    if transient > 0:
        sol_transient = solve_ivp(
            lorenz96_controlled_rhs,
            t_span=(0, transient),
            y0=state0,
            method='RK23',
            rtol=1e-4,
            atol=1e-6,
            dense_output=False,
        )
        if not sol_transient.success:
            raise RuntimeError("Transient integration failed")
        state0 = sol_transient.y[:, -1]
    
    # Data collection phase with output at requested time steps
    num_steps = int(np.round(T / dt))
    t_eval = np.linspace(0.0, num_steps * dt, num_steps + 1)
    
    sol = solve_ivp(
        lorenz96_controlled_rhs,
        t_span=(0, t_eval[-1]),
        y0=state0,
        method='RK23',
        t_eval=t_eval,
        rtol=1e-4,
        atol=1e-6,
        dense_output=False,
    )
    
    if not sol.success:
        raise RuntimeError("ODE solver failed during data collection")
    
    times = sol.t
    trajectory = sol.y.T  # shape: (num_steps+1, N)
    target = trajectory[:, target_variable]
    
    # Compute controls and sensor trace for all time points
    controls = np.zeros((len(times), N), dtype=float)
    sensor_trace = np.zeros((len(times), len(policy.sensed_dims)), dtype=float)
    
    for i in range(len(times)):
        state = trajectory[i]
        sensors = state[policy.sensed_dims]
        sensor_trace[i] = sensors
        controls[i] = policy.get_control(sensors, c_max)
    
    metadata = {
        "N": int(N),
        "F": float(F),
        "dt": float(dt),
        "T": float(T),
        "transient": float(transient),
        "c_max": float(c_max),
        "target_variable": int(target_variable),
        "seed": int(seed),
    }
    
    return ControlledSimulationResult(
        times=times,
        trajectory=trajectory,
        target=target,
        controls=controls,
        sensor_trace=sensor_trace,
        metadata=metadata,
    )


# --------------------------------------------------------------------------- #
# CMA-ES objective helpers for control policy optimisation
# --------------------------------------------------------------------------- #


@dataclass
class SimulationConfig:
    """Configuration container for Lorenz 96 simulations."""
    
    N: int = 40
    F: float = 15.0
    dt: float = 0.01
    T: float = 100.0
    transient: float = 10.0
    target_variable: int = 2
    seed: int = 42


@dataclass
class ControlObjectiveConfig:
    """Parameters governing the policy optimisation objective."""
    
    threshold: float  # t in P(x_target > t)
    lambda_control: float
    c_max: float
    bin_width: float = 1.0
    lb_seed: int = 42
    lb_maxiter: int = 150


@dataclass
class PolicyOptimizationResult:
    """Bundle of outputs from CMA-ES policy optimisation."""
    
    best_theta: np.ndarray
    best_objective: float
    best_lower_bound: float
    best_control_penalty: float
    best_true_prob: float
    history: List[Dict[str, float]]
    rollout: ControlledSimulationResult


class ControlObjectiveEvaluator:
    """Callable wrapper producing CMA-ES-friendly objectives."""
    
    def __init__(
        self,
        policy_template: PolynomialPolicy,
        sim_config: SimulationConfig,
        objective_config: ControlObjectiveConfig,
    ) -> None:
        self.policy = policy_template.copy()
        self.sim_config = sim_config
        self.objective_config = objective_config
        self._initial_theta = policy_template.get_params_as_array()
        self.policy.set_params_from_array(self._initial_theta)
        self.last_rollout: Optional[ControlledSimulationResult] = None
        self.evaluation_history: List[Dict[str, float]] = []
    
    def evaluate(self, theta: np.ndarray, capture_rollout: bool = False) -> Dict[str, Any]:
        """Run the simulation and compute objective components."""
        self.policy.set_params_from_array(theta)
        
        try:
            sim_result = solve_lorenz96_controlled(
                N=self.sim_config.N,
                F=self.sim_config.F,
                dt=self.sim_config.dt,
                T=self.sim_config.T,
                transient=self.sim_config.transient,
                policy=self.policy,
                c_max=self.objective_config.c_max,
                target_variable=self.sim_config.target_variable,
                seed=self.sim_config.seed,
            )
        except Exception as e:
            return self._failure_stats(message=f"simulation_failed: {e}", capture_rollout=capture_rollout)
        
        target_values = sim_result.target
        
        if target_values.size == 0 or not np.all(np.isfinite(target_values)):
            return self._failure_stats(message="non_finite_target", capture_rollout=capture_rollout)
        if not np.all(np.isfinite(sim_result.controls)):
            return self._failure_stats(message="non_finite_control", capture_rollout=capture_rollout)
        
        # Compute histogram and lower bound on P(target > threshold)
        target_max = float(np.max(target_values))
        target_min = float(np.min(target_values))
        
        # Extend range to include threshold if needed
        lower = min(target_min, self.objective_config.threshold)
        upper = max(target_max, self.objective_config.threshold)
        
        # Calculate number of bins based on fixed bin width
        num_bins = max(10, int(np.ceil((upper - lower) / self.objective_config.bin_width)))
        bin_edges = np.linspace(lower, lower + num_bins * self.objective_config.bin_width, num_bins + 1, dtype=float)
        
        # Ensure threshold is included as a bin edge
        thresholds = np.array([self.objective_config.threshold], dtype=float)
        bin_edges = ensure_thresholds_in_edges(bin_edges=bin_edges, thresholds=thresholds)
        num_bins = len(bin_edges) - 1
        
        try:
            lb_data = compute_lb(
                energy=target_values,
                thresholds=thresholds,
                num_bins=num_bins,
                bin_edges=bin_edges,
                seed=self.objective_config.lb_seed,
                critical_thresholds=[self.objective_config.threshold],
                lb_maxiter=self.objective_config.lb_maxiter,
            )
            lower_bound = float(lb_data["lower_bounds"][0])
        except Exception as e:
            return self._failure_stats(message=f"lb_failure: {e}", capture_rollout=capture_rollout)
        
        # Control penalty
        mean_control_sq = float(np.mean(np.sum(sim_result.controls**2, axis=1)))
        control_penalty = self.objective_config.lambda_control * mean_control_sq
        
        # True probability P(target > threshold)
        true_prob = float(np.mean(target_values > self.objective_config.threshold))
        
        # Objective: maximize true probability (for now, can switch to lower bound)
        objective_value = true_prob - control_penalty
        loss_value = -objective_value
        
        stats = {
            "loss": loss_value,
            "objective": objective_value,
            "lower_bound": lower_bound,
            "true_prob": true_prob,
            "mean_control_sq": mean_control_sq,
            "control_penalty": control_penalty,
        }
        
        self.evaluation_history.append(
            {
                "loss": loss_value,
                "objective": objective_value,
                "lower_bound": lower_bound,
                "true_prob": true_prob,
                "control_penalty": control_penalty,
            }
        )
        
        if capture_rollout:
            self.last_rollout = sim_result
            stats["rollout"] = sim_result
        
        return stats
    
    def _failure_stats(
        self,
        *,
        message: str,
        capture_rollout: bool,
    ) -> Dict[str, Any]:
        loss_value = 1e6
        
        stats = {
            "loss": loss_value,
            "objective": -1e6,
            "lower_bound": 0.0,
            "true_prob": 0.0,
            "mean_control_sq": 0.0,
            "control_penalty": 0.0,
            "failure_reason": message,
        }
        self.evaluation_history.append(
            {
                "loss": float(loss_value),
                "objective": float(stats["objective"]),
                "lower_bound": 0.0,
                "true_prob": 0.0,
                "control_penalty": 0.0,
                "failure_reason": message,
            }
        )
        if capture_rollout:
            self.last_rollout = None
            stats["rollout"] = None
        return stats
    
    def __call__(self, theta: np.ndarray) -> float:
        """CMA-ES expects a scalar loss to minimise."""
        stats = self.evaluate(theta, capture_rollout=False)
        return float(stats["loss"])
    
    def initial_theta(self) -> np.ndarray:
        return np.array(self._initial_theta, copy=True)


_OBJECTIVE_WORKER: Optional[ControlObjectiveEvaluator] = None


def _init_objective_worker(pickled_objective: bytes) -> None:
    global _OBJECTIVE_WORKER
    _OBJECTIVE_WORKER = pickle.loads(pickled_objective)


def _evaluate_objective_theta(theta: np.ndarray) -> Dict[str, Any]:
    if _OBJECTIVE_WORKER is None:
        raise RuntimeError("Objective worker not initialised.")
    theta_array = np.asarray(theta, dtype=float)
    return _OBJECTIVE_WORKER.evaluate(theta_array, capture_rollout=False)


def optimize_policy_cmaes(
    objective: ControlObjectiveEvaluator,
    *,
    sigma0: float = 0.3,
    maxiter: int = 100,
    seed: Optional[int] = None,
    popsize: Optional[int] = None,
    initial_theta: Optional[np.ndarray] = None,
    n_cpus: Optional[int] = None,
) -> PolicyOptimizationResult:
    """Run CMA-ES to optimise the polynomial policy coefficients."""
    
    theta0 = objective.initial_theta() if initial_theta is None else np.asarray(initial_theta, dtype=float)
    if theta0.shape != objective.initial_theta().shape:
        raise ValueError("Initial parameter vector has incorrect shape.")
    
    history: List[Dict[str, float]] = []
    
    def append_history(stats: Dict[str, Any], *, record_in_objective: bool) -> None:
        record = {
            "loss": float(stats["loss"]),
            "objective": float(stats["objective"]),
            "lower_bound": float(stats["lower_bound"]),
            "true_prob": float(stats.get("true_prob", 0.0)),
            "control_penalty": float(stats["control_penalty"]),
        }
        history.append(record)
        if record_in_objective:
            objective.evaluation_history.append(record)
    
    options: Dict[str, Any] = {
        "maxiter": maxiter,
        "verb_disp": 1,
    }
    if seed is not None:
        options["seed"] = seed
    if popsize is not None:
        options["popsize"] = popsize
    
    es = cma.CMAEvolutionStrategy(theta0, sigma0, options)
    
    if n_cpus is None:
        worker_count = 1
    elif n_cpus <= 0:
        worker_count = max(1, os.cpu_count() or 1)
    else:
        worker_count = n_cpus
    
    executor: Optional[ProcessPoolExecutor] = None
    pickled_objective: Optional[bytes] = None
    
    if worker_count > 1:
        ctx = mp.get_context("spawn")
        pickled_objective = pickle.dumps(objective)
        executor = ProcessPoolExecutor(
            max_workers=worker_count,
            mp_context=ctx,
            initializer=_init_objective_worker,
            initargs=(pickled_objective,),
        )
    
    try:
        for iteration in range(maxiter):
            if es.stop():
                break
            candidates = es.ask()
            if worker_count == 1:
                stats_batch = [objective.evaluate(np.asarray(theta, dtype=float), capture_rollout=False) for theta in candidates]
                record_in_objective = False
            else:
                assert executor is not None
                futures = [
                    executor.submit(_evaluate_objective_theta, np.asarray(theta, dtype=float))
                    for theta in candidates
                ]
                stats_batch = [future.result() for future in futures]
                record_in_objective = True
            losses = []
            true_probs = []
            lower_bounds = []
            for theta, stats in zip(candidates, stats_batch):
                append_history(stats, record_in_objective=record_in_objective)
                losses.append(float(stats["loss"]))
                true_probs.append(float(stats.get("true_prob", 0.0)))
                lower_bounds.append(float(stats.get("lower_bound", 0.0)))
            es.tell(candidates, losses)
            es.disp()
            # Print the best results from this generation
            best_idx = np.argmin(losses)
            best_true_prob = true_probs[best_idx]
            best_lb = lower_bounds[best_idx]
            print(f"  → Best P(x_target>{objective.objective_config.threshold:.1f}) = {best_true_prob:.6f}, LB = {best_lb:.6f}")
            if es.stop():
                break
    finally:
        if executor is not None:
            executor.shutdown()
    
    if es.result is None or es.result.xbest is None:
        raise RuntimeError("CMA-ES failed to produce a result.")
    
    best_theta = np.asarray(es.result.xbest, dtype=float)
    final_stats = objective.evaluate(best_theta, capture_rollout=True)
    rollout = final_stats["rollout"]
    
    return PolicyOptimizationResult(
        best_theta=best_theta,
        best_objective=float(final_stats["objective"]),
        best_lower_bound=float(final_stats["lower_bound"]),
        best_control_penalty=float(final_stats["control_penalty"]),
        best_true_prob=float(final_stats.get("true_prob", 0.0)),
        history=history,
        rollout=rollout,
    )


# --------------------------------------------------------------------------- #
# Sensor Optimization Functions
# --------------------------------------------------------------------------- #


def compute_mutual_information(
    target: np.ndarray,
    sensors: np.ndarray,
    num_bins: int,
) -> float:
    """Compute mutual information I(target; sensors) between scalar target and vector sensor measurements."""
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
    sensor_axes = tuple(range(1, joint_prob.ndim))
    p_target = np.sum(joint_prob, axis=sensor_axes)
    p_sensors = np.sum(joint_prob, axis=0)
    
    # Expand for broadcasting
    expanded_p_target = p_target.reshape((-1,) + (1,) * (joint_prob.ndim - 1))
    expanded_p_sensors = p_sensors.reshape((1,) + p_sensors.shape)
    
    # Compute MI
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
        trajectory: np.ndarray,
        target: np.ndarray,
        num_bins: int,
        num_sensors: int,
        N: int,
    ) -> None:
        self.trajectory = trajectory
        self.target = target
        self.num_bins = num_bins
        self.num_sensors = num_sensors
        self.N = N
        if num_sensors <= 0:
            raise ValueError("`num_sensors` must be positive.")
        if num_sensors >= N:
            raise ValueError(f"`num_sensors` ({num_sensors}) must be less than N ({N}).")
    
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
        sensors = self.trajectory[:, indices]
        mi = compute_mutual_information(self.target, sensors, self.num_bins)
        return mi
    
    def _parameters_to_indices(self, theta: np.ndarray) -> np.ndarray:
        """Convert continuous parameters to discrete sensor indices."""
        indices = np.clip(theta, 0, self.N - 1)
        indices = np.round(indices).astype(int)
        indices = self._ensure_unique_indices(indices)
        return indices
    
    def _ensure_unique_indices(self, indices: np.ndarray) -> np.ndarray:
        """Ensure all indices are unique by adjusting duplicates."""
        unique_indices = []
        used = set()
        
        for idx in indices:
            if idx in used:
                for offset in range(1, self.N):
                    candidate_low = (idx - offset) % self.N
                    if candidate_low not in used:
                        idx = candidate_low
                        break
                    candidate_high = (idx + offset) % self.N
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


def optimize_sensor_locations(
    *,
    N: int,
    F: float,
    target_variable: int,
    num_sensors: int,
    num_bins: int,
    T: float,
    dt: float,
    transient: float,
    seed: int,
    maxiter: int,
    sigma0: float = 3.0,
    popsize: int = 50,
) -> Tuple[np.ndarray, float]:
    """Optimize sensor locations to maximize mutual information with target variable.
    
    Returns
    -------
    sensor_indices : array of int
        Optimal sensor locations (0-indexed)
    mutual_information : float
        Achieved mutual information value
    """
    print("\n" + "=" * 70)
    print("STEP 1: OPTIMAL SENSOR PLACEMENT")
    print("=" * 70)
    print(f"Running uncontrolled simulation for sensor optimization...")
    print(f"  T = {T}, dt = {dt}, transient = {transient}")
    
    # Generate trajectory data
    trajectory, times = solve_lorenz96_uncontrolled(
        N=N,
        F=F,
        t0=0.0,
        t1=T,
        dt=dt,
        transient=transient,
        seed=seed,
    )
    
    target = trajectory[:, target_variable]
    print(f"  Collected {trajectory.shape[0]} samples")
    print(f"  Target: x_{target_variable + 1} (index {target_variable})")
    
    # Create objective
    obj = SensorMIObjective(
        trajectory=trajectory,
        target=target,
        num_bins=num_bins,
        num_sensors=num_sensors,
        N=N,
    )
    
    # Initial guess (uniformly spaced)
    if num_sensors == 1:
        mean0 = np.array([N / 2.0])
    else:
        spacing = N / num_sensors
        mean0 = (np.arange(num_sensors) + 0.5) * spacing
    
    lower, upper = 0.0, float(N - 1)
    
    print(f"\nOptimizing {num_sensors} sensor locations via CMA-ES...")
    print(f"  Maximum iterations: {maxiter}")
    
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
        raise RuntimeError("CMA-ES did not return a result for sensor optimization.")
    
    best_theta = es.result.xbest
    if not np.all(np.isfinite(best_theta)):
        best_theta = mean0
    
    if np.isfinite(es.result.fbest):
        best_mi = -float(es.result.fbest)
    else:
        best_mi = obj.mutual_information(best_theta)
    
    sensor_indices = obj.decode_parameters(best_theta)
    
    print(f"\nSensor optimization complete!")
    print(f"  Mutual information: {best_mi:.6f} nats")
    print(f"  Optimal sensor indices:")
    for i, idx in enumerate(sensor_indices, start=1):
        print(f"    Sensor {i}: x_{idx + 1} (index {idx})")
    print("=" * 70)
    
    return sensor_indices, best_mi


# --------------------------------------------------------------------------- #
# Command-line interface
# --------------------------------------------------------------------------- #


def main():
    args = parse_args()
    
    print("=" * 70)
    print("Lorenz 96 System: Integrated Sensor + Control Optimization")
    print("=" * 70)
    
    # Determine sensor indices
    sensed_dims = None
    sensor_mi = None
    
    if args.sense is not None:
        # Manual sensor specification (highest priority)
        sensed_dims = args.sense
        print(f"\nUsing manually specified sensor indices: {sensed_dims}")
    elif os.path.exists(args.sensor_file) and not args.auto_sensors:
        # Load from existing sensor file
        sensor_data = np.load(args.sensor_file)
        sensed_dims = list(sensor_data["sensor_indices"])
        if "mutual_information" in sensor_data:
            sensor_mi = float(sensor_data["mutual_information"])
        print(f"\nLoaded sensor indices from {args.sensor_file}: {sensed_dims}")
        if sensor_mi is not None:
            print(f"  Mutual information: {sensor_mi:.6f} nats")
    else:
        # Auto-optimize sensors
        if not args.auto_sensors:
            print(f"\nWarning: Sensor file '{args.sensor_file}' not found.")
            print("Automatically optimizing sensor locations...")
            print("(Use --sense to specify manually or --auto-sensors to suppress this warning)")
        
        sensor_indices, sensor_mi = optimize_sensor_locations(
            N=args.N,
            F=args.F,
            target_variable=args.target_variable,
            num_sensors=args.num_sensors,
            num_bins=args.num_bins,
            T=args.sensor_T,
            dt=args.dt,
            transient=args.transient,
            seed=args.seed,
            maxiter=args.sensor_maxiter,
        )
        sensed_dims = list(sensor_indices)
        
        # Save sensor results
        sensor_output = f"sensors_auto_{args.num_sensors}.npz"
        np.savez(
            sensor_output,
            sensor_indices=sensor_indices,
            mutual_information=sensor_mi,
            num_sensors=args.num_sensors,
            target_variable=args.target_variable,
            N=args.N,
            F=args.F,
            seed=args.seed,
        )
        print(f"\nSensor results saved to: {sensor_output}")
    
    actuated_dims = args.actuate
    
    # Generate automatic filename if using default
    if args.output == "policy.npz":
        c_max_str = f"cm{args.c_max}".replace(".", "p")
        actuate_str = "_".join(str(d) for d in actuated_dims)
        poly_str = str(args.poly_order)
        args.output = f"policy_{c_max_str}_act{actuate_str}_p{poly_str}.npz"
    
    print("\n" + "=" * 70)
    print("STEP 2: CONTROL POLICY OPTIMIZATION")
    print("=" * 70)
    print(f"System: N = {args.N}, F = {args.F}")
    print(f"Actuated indices: {actuated_dims}")
    print(f"Sensed indices: {sensed_dims}")
    print(f"Polynomial order: {args.poly_order}")
    print(f"Target variable: x_{args.target_variable + 1} (index {args.target_variable})")
    print(f"Threshold: {args.threshold}")
    print(f"Objective: Maximize P(x_{args.target_variable + 1} > {args.threshold})")
    print("=" * 70 + "\n")
    
    # Create policy
    policy = PolynomialPolicy(
        actuated_dims=actuated_dims,
        sensed_dims=sensed_dims,
        poly_order=args.poly_order,
        N=args.N,
    )
    print(f"Policy has {policy.num_params} parameters.\n")
    
    # Simulation config
    sim_config = SimulationConfig(
        N=args.N,
        F=args.F,
        dt=args.dt,
        T=args.T,
        transient=args.transient,
        target_variable=args.target_variable,
        seed=args.seed,
    )
    
    # Objective config
    objective_config = ControlObjectiveConfig(
        threshold=args.threshold,
        lambda_control=args.lambda_control,
        c_max=args.c_max,
        bin_width=args.bin_width,
        lb_seed=args.seed,
        lb_maxiter=args.lb_maxiter,
    )
    
    # Create objective evaluator
    evaluator = ControlObjectiveEvaluator(
        policy_template=policy,
        sim_config=sim_config,
        objective_config=objective_config,
    )
    
    # Run CMA-ES optimization
    print("Starting CMA-ES optimization...\n")
    result = optimize_policy_cmaes(
        objective=evaluator,
        sigma0=args.sigma0,
        maxiter=args.maxiter,
        seed=args.seed,
        n_cpus=args.n_cpus,
    )
    
    print("\n" + "=" * 70)
    print("Optimization complete!")
    print(f"Best objective: {result.best_objective:.6f}")
    print(f"Best lower bound: {result.best_lower_bound:.6f}")
    print(f"Best true P(x_{args.target_variable + 1} > {args.threshold}): {result.best_true_prob:.6f}")
    print(f"Control penalty: {result.best_control_penalty:.6f}")
    print("=" * 70 + "\n")
    
    # Save results
    policy.set_params_from_array(result.best_theta)
    save_data = {
        "params": result.best_theta,
        "coefficients": policy._coefficients,
        "alphas": policy._alphas,
        "actuated_dims": np.array(actuated_dims, dtype=int),
        "sensed_dims": np.array(sensed_dims, dtype=int),
        "poly_order": np.array(args.poly_order, dtype=int),
        "N": np.array(args.N, dtype=int),
        "config": {
            "N": args.N,
            "F": args.F,
            "dt": args.dt,
            "T": args.T,
            "transient": args.transient,
            "target_variable": args.target_variable,
            "seed": args.seed,
            "threshold": args.threshold,
            "c_max": args.c_max,
            "bin_width": args.bin_width,
            "lb_maxiter": args.lb_maxiter,
        },
        "optimization_results": {
            "best_objective": result.best_objective,
            "best_lower_bound": result.best_lower_bound,
            "best_true_prob": result.best_true_prob,
            "best_control_penalty": result.best_control_penalty,
        },
    }
    
    # Add sensor optimization info if available
    if sensor_mi is not None:
        save_data["sensor_mutual_information"] = sensor_mi
    
    np.savez(args.output, **save_data)
    print(f"\nPolicy saved to: {args.output}")
    print(f"Alpha scaling parameters: {policy._alphas}")
    if sensor_mi is not None:
        print(f"Sensor mutual information: {sensor_mi:.6f} nats")


if __name__ == "__main__":
    main()

