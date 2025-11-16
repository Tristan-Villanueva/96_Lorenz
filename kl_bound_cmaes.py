import numpy as np
from typing import Optional, Tuple, Dict, Any, Sequence, List

try:
    import cma
except ImportError as exc:
    raise ImportError("CMA-ES requires the 'cma' package. Install it via `pip install cma`.") from exc


def softmax(x: np.ndarray) -> np.ndarray:
    """Convert unconstrained parameters to a probability distribution."""
    x = np.asarray(x, dtype=float)
    x_shifted = x - np.max(x)
    x_shifted = np.clip(x_shifted, -500, 500)
    exp_x = np.exp(x_shifted)
    probs = exp_x / np.sum(exp_x)
    probs = np.maximum(probs, 1e-15)
    return probs / np.sum(probs)


def kl_divergence(p: np.ndarray, q: np.ndarray) -> float:
    """Compute KL divergence D(p || q) in base-2 logarithm."""
    eps = 1e-12
    p_safe = np.clip(p, eps, 1.0)
    q_safe = np.clip(q, eps, 1.0)
    return float(np.sum(p_safe * np.log2(p_safe / q_safe)))


def ensure_thresholds_in_edges(
    *,
    bin_edges: np.ndarray,
    thresholds: Sequence[float],
) -> np.ndarray:
    """Return histogram edges that include all requested threshold values."""
    edges = np.asarray(bin_edges, dtype=float)
    thresholds_arr = np.asarray(thresholds, dtype=float).ravel()
    thresholds_arr = thresholds_arr[np.isfinite(thresholds_arr)]
    if thresholds_arr.size == 0:
        return edges

    lower_required = float(np.min(thresholds_arr))
    upper_required = float(np.max(thresholds_arr))

    if lower_required < edges[0]:
        edges = np.concatenate(([lower_required], edges))
    if upper_required > edges[-1]:
        edges = np.concatenate((edges, [upper_required]))

    interior = thresholds_arr[
        (thresholds_arr > edges[0]) & (thresholds_arr < edges[-1])
    ]
    if interior.size:
        edges = np.concatenate((edges, interior))
    edges = np.unique(edges)
    edges.sort()
    return edges


def compute_empirical_pmf(
    samples: np.ndarray,
    num_bins: int,
    bin_edges: Optional[np.ndarray] = None,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Histogram-based empirical PMF."""
    if bin_edges is not None:
        counts, edges = np.histogram(samples, bins=bin_edges)
    else:
        counts, edges = np.histogram(
            samples,
            bins=num_bins,
            range=(0.0, float(np.max(samples))),
        )
    total = counts.sum()
    if total == 0:
        raise ValueError("Histogram counts sum to zero.")
    pmf = counts / total
    centers = 0.5 * (edges[:-1] + edges[1:])
    return pmf, centers, edges


def evaluate_lower_bound(p_J: np.ndarray, M_indices: np.ndarray, p_X: np.ndarray) -> Tuple[float, float, float, np.ndarray]:
    """Evaluate the lower bound using a candidate auxiliary distribution."""
    eps = 1e-12
    p_X = np.clip(p_X, eps, None)
    p_X = p_X / np.sum(p_X)

    P_X_in_M = np.sum(p_X[M_indices]) if np.any(M_indices) else 0.0
    D_JX = kl_divergence(p_J, p_X)

    if P_X_in_M >= 1.0 - 1e-12:
        lower_bound = 0.0
    else:
        denom = np.log2(1.0 / (1.0 - P_X_in_M)) if P_X_in_M < 1.0 else np.inf
        if denom <= 1e-12 or not np.isfinite(denom):
            lower_bound = 0.0
        else:
            lower_bound = 1.0 - (D_JX + np.log2(2)) / denom
            lower_bound = max(0.0, min(1.0, lower_bound))
    return lower_bound, D_JX, P_X_in_M, p_X


def optimize_lower_distribution(
    p_J: np.ndarray,
    M_indices: np.ndarray,
    sigma0: float = 0.5,#changing 
    maxiter: int = 1000,#changing 
    seed: Optional[int] = None,
) -> np.ndarray:
    """Optimize auxiliary distribution for lower bound using CMA-ES."""
    n_bins = len(p_J)

    if not np.any(M_indices):
        return np.ones(n_bins) / n_bins

    def objective(theta: np.ndarray) -> float:
        if not np.all(np.isfinite(theta)):
            return 1e20
        p_X = softmax(theta)
        P_X_in_M = float(np.sum(p_X[M_indices]))
        if P_X_in_M <= 0.001:
            return 1e10
        if P_X_in_M >= 0.999:
            P_X_in_M = 0.999

        denom = np.log2(1.0 / (1.0 - P_X_in_M))
        if denom <= 1e-10 or not np.isfinite(denom):
            return 1e6

        D_JX = kl_divergence(p_J, p_X)
        if not np.isfinite(D_JX):
            return 1e6

        lower_bound = 1.0 - (D_JX + np.log2(2)) / denom
        if not np.isfinite(lower_bound):
            return 1e6
        return -lower_bound

    # Candidate initializations
    theta_candidates: List[np.ndarray] = []

    theta_uniform = np.zeros(n_bins)
    theta_candidates.append(theta_uniform)

    biased = np.ones(n_bins)
    biased[M_indices] *= 1.5
    biased /= biased.sum()
    theta_biased = np.log(biased + 1e-12)
    theta_biased -= np.mean(theta_biased)
    theta_candidates.append(theta_biased)

    theta_data = np.log(p_J + 1e-12)
    theta_data -= np.mean(theta_data)
    theta_candidates.append(theta_data)

    best_theta = None
    best_obj = np.inf

    for theta0 in theta_candidates:
        test_obj = objective(theta0)
        if not np.isfinite(test_obj):
            continue
        try:
            es = cma.CMAEvolutionStrategy(theta0, sigma0, {
                'maxiter': maxiter,
                'verb_disp': 0,
                'verb_log': 0,
                'verb_plot': 0,
                'seed': seed,
                'popsize': min(4 + int(3 * np.log(n_bins)), 25),
            })
            es.optimize(objective, iterations=maxiter)
            if hasattr(es, 'result') and es.result is not None:
                result_theta = es.result.xbest
                result_obj = es.result.fbest
                if np.isfinite(result_obj) and np.all(np.isfinite(result_theta)):
                    if result_obj < best_obj:
                        best_obj = result_obj
                        best_theta = result_theta
        except Exception:
            continue

    if best_theta is None:
        fallback = np.ones(n_bins)
        fallback[M_indices] *= 2.0
        fallback = fallback / fallback.sum()
        return fallback

    return softmax(best_theta)


def compute_lb(
    energy: np.ndarray,
    thresholds: Sequence[float],
    num_bins: int = 100,
    bin_edges: Optional[np.ndarray] = None,
    seed: Optional[int] = 42,
    critical_thresholds: Optional[Sequence[float]] = None,
    lb_maxiter: int = 150,
) -> Dict[str, Any]:
    """
    Compute optimized lower bounds for containment probabilities P(E ∈ M_t),
    where M_t = {e > t}.
    
    Parameters
    ----------
    lb_maxiter : int
        Maximum CMA-ES iterations for optimizing the auxiliary distribution.
    """
    energy = np.asarray(energy, dtype=float)
    thresholds = np.asarray(thresholds, dtype=float)
    if energy.ndim != 1:
        raise ValueError("Energy array must be one-dimensional.")

    if critical_thresholds is None:
        thresholds_for_edges = thresholds
    else:
        thresholds_for_edges = np.asarray(critical_thresholds, dtype=float)
    finite_thresholds = thresholds_for_edges[np.isfinite(thresholds_for_edges)]

    if bin_edges is None:
        max_energy = float(np.max(energy)) if energy.size else 0.0
        upper = max(
            max_energy,
            float(np.max(finite_thresholds)) if finite_thresholds.size else max_energy,
        )
        base_edges = np.linspace(0.0, upper, num_bins + 1, dtype=float)
    else:
        base_edges = np.asarray(bin_edges, dtype=float)

    adjusted_edges = ensure_thresholds_in_edges(
        bin_edges=base_edges,
        thresholds=finite_thresholds,
    )

    pmf_bins = adjusted_edges.size - 1
    p_J, bin_centers, edges = compute_empirical_pmf(
        energy,
        num_bins=pmf_bins,
        bin_edges=adjusted_edges,
    )

    lower_bounds = np.zeros_like(thresholds)
    true_probs_discrete = np.zeros_like(thresholds)
    true_probs_continuous = np.zeros_like(thresholds)
    divergences = np.zeros_like(thresholds)
    px_mass_in_M = np.zeros_like(thresholds)

    optimized_distributions: List[np.ndarray] = []

    for idx, t in enumerate(thresholds):
        M_indices = bin_centers > t
        true_probs_discrete[idx] = float(np.sum(p_J[M_indices])) if np.any(M_indices) else 0.0
        true_probs_continuous[idx] = float(np.mean(energy > t))

        if not np.any(M_indices):
            lower_bounds[idx] = 0.0
            divergences[idx] = 0.0
            px_mass_in_M[idx] = 0.0
            optimized_distributions.append(np.ones_like(p_J) / p_J.size)
            continue

        p_X_opt = optimize_lower_distribution(
            p_J,
            M_indices,
            maxiter=lb_maxiter,
            seed=seed,
        )
        lower_bound, divergence, mass_in_M, p_X_norm = evaluate_lower_bound(p_J, M_indices, p_X_opt)
        lower_bounds[idx] = lower_bound
        divergences[idx] = divergence
        px_mass_in_M[idx] = mass_in_M
        optimized_distributions.append(p_X_norm)

    return {
        "thresholds": thresholds,
        "lower_bounds": lower_bounds,
        "true_probabilities_discrete": true_probs_discrete,
        "true_probabilities_continuous": true_probs_continuous,
        "bin_centers": bin_centers,
        "bin_edges": edges,
        "p_J": p_J,
        "optimized_distributions": optimized_distributions,
        "divergences": divergences,
        "p_X_mass_in_M": px_mass_in_M,
    }
