"""Continuously optimize one known board pose for one camera's extrinsics.

Run with a Python environment containing NumPy, SciPy, and Matplotlib:
    python content/posts/2025-02-EOL-calib/simulations/one_camera_one_board_placement.py

The physical camera stays fixed. SLSQP minimizes the normalized trace of the
camera-pose covariance by adjusting all six board-pose coordinates. No candidate
grid or Monte Carlo errors are used to choose the placement. A subsequent noise
simulation validates the original and optimized placements independently.

The same run also reproduces the optical-axis-constrained optimization and the
four mirror-symmetry checks. Use --centred-checks-only to skip the noise trials
and figures, or --output-dir to write an isolated set of reproducibility files.
"""

import argparse
import csv
import json
import os
import platform
import tempfile
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "ocob-matplotlib"))
os.environ.setdefault("XDG_CACHE_HOME", str(Path(tempfile.gettempdir()) / "ocob-cache"))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.patches import Rectangle
from matplotlib.ticker import MaxNLocator
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import scipy
from scipy.optimize import least_squares, minimize
from scipy.spatial.transform import Rotation


OUT = Path(__file__).resolve().parents[1] / "resources"
WIDTH, HEIGHT = 1280, 960
FOCAL = 800.0
PRINCIPAL = np.array([WIDTH / 2, HEIGHT / 2])
SIGMA_PIXEL = 0.25
MARGIN = 40.0
MIN_DEPTH = 0.6
MIN_GAP = 12.0
MIN_INCIDENCE = np.cos(np.deg2rad(70))
TRANSLATION_TOLERANCE = 0.005
ROTATION_TOLERANCE = np.deg2rad(0.1)
CAMERA_SCALES = np.r_[np.full(3, TRANSLATION_TOLERANCE), np.full(3, ROTATION_TOLERANCE)]
BOARD_SCALES = np.array([0.5, 0.5, 1.0, 30.0, 30.0, 30.0])
BOARD_BOUNDS = np.array([[-0.6, 0.6], [-0.4, 0.4], [1.0, 3.0], [-60, 60], [-60, 60], [-45, 45]])
INITIAL = np.array([0.25, 0.15, 2.5, 5.0, 8.0, 0.0])
LOCAL_POINTS = np.array([[x, y, 0.0] for y in np.linspace(-0.2, 0.2, 5) for x in np.linspace(-0.3, 0.3, 7)])
OUTLINE = [0, 6, 34, 28, 0]
COLORS = {"Initial": "#3974ad", "Optimized": "#d86528"}
OFF_CENTRE_OPTIONS = {"maxiter": 200, "ftol": 1e-10}
CENTRED_OPTIONS = {"maxiter": 200, "ftol": 1e-11}
FEASIBILITY_TOLERANCE = 1e-7
SYMMETRY_RTOL = 1e-10
SYMMETRY_ATOL = 1e-12


def board_geometry(s):
    """s = [x, y, z in m, Euler x/y/z in deg]; camera frame = world at truth."""
    # At zero angles, the printed front normal points toward the camera (-Z).
    rotation = Rotation.from_euler("xyz", s[3:], degrees=True).as_matrix() @ np.diag([1.0, -1.0, -1.0])
    points = LOCAL_POINTS @ rotation.T + s[:3]
    normal = rotation[:, 2]
    incidence = (-points / np.linalg.norm(points, axis=1, keepdims=True)) @ normal
    sigma = SIGMA_PIXEL / np.maximum(incidence, 0.05)
    return points, sigma, incidence


def project(points):
    return FOCAL * points[:, :2] / points[:, 2, None] + PRINCIPAL


def camera_jacobian(points, sigma):
    """Whitened Jacobian for camera right increments [translation, rotation]."""
    result = []
    for (x, y, z), noise in zip(points, sigma):
        projection = FOCAL * np.array([[1 / z, 0, -x / z**2], [0, 1 / z, -y / z**2]])
        cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
        result.append(projection @ np.hstack([-np.eye(3), cross]) / noise)
    return np.vstack(result)


def information(s):
    points, sigma, _ = board_geometry(s)
    jacobian = camera_jacobian(points, sigma)
    # Work in dimensionless camera increments; do not add artificial damping.
    _, singular, vt = np.linalg.svd(jacobian * CAMERA_SCALES, full_matrices=False)
    if singular[-1] < 1e-10:
        raise ValueError("Unobservable camera-pose direction")
    normalized_covariance = (vt.T / singular**2) @ vt
    covariance = CAMERA_SCALES[:, None] * normalized_covariance * CAMERA_SCALES[None, :]
    return float(np.trace(normalized_covariance)), covariance


def constraints(s):
    points, _, incidence = board_geometry(s)
    uv = project(points)
    grid = uv.reshape(5, 7, 2)
    gaps = np.r_[np.linalg.norm(np.diff(grid, axis=0), axis=2).ravel(), np.linalg.norm(np.diff(grid, axis=1), axis=2).ravel()]
    return np.r_[
        (uv[:, 0] - MARGIN) / WIDTH,
        (WIDTH - MARGIN - uv[:, 0]) / WIDTH,
        (uv[:, 1] - MARGIN) / HEIGHT,
        (HEIGHT - MARGIN - uv[:, 1]) / HEIGHT,
        points[:, 2] - MIN_DEPTH,
        incidence - MIN_INCIDENCE,
        gaps / MIN_GAP - 1,
    ]


def metrics(s):
    score, covariance = information(s)
    points, sigma, incidence = board_geometry(s)
    return {
        "x_m": float(s[0]), "y_m": float(s[1]), "z_m": float(s[2]),
        "rx_deg": float(s[3]), "ry_deg": float(s[4]), "rz_deg": float(s[5]),
        "score": score,
        "predicted_translation_rms_mm": float(1000 * np.sqrt(np.trace(covariance[:3, :3]))),
        "predicted_rotation_rms_deg": float(np.rad2deg(np.sqrt(np.trace(covariance[3:, 3:])))),
        "mean_sigma_px": float(np.mean(sigma)),
        "max_viewing_angle_deg": float(np.rad2deg(np.arccos(np.min(incidence)))),
        "minimum_constraint": float(np.min(constraints(s))),
    }


def optimize():
    history = [INITIAL.copy()]

    def callback(scaled):
        history.append(scaled * BOARD_SCALES)

    result = minimize(
        lambda scaled: information(scaled * BOARD_SCALES)[0],
        INITIAL / BOARD_SCALES,
        method="SLSQP",
        bounds=BOARD_BOUNDS / BOARD_SCALES[:, None],
        constraints={"type": "ineq", "fun": lambda scaled: constraints(scaled * BOARD_SCALES)},
        callback=callback,
        options=OFF_CENTRE_OPTIONS.copy(),
    )
    solution = result.x * BOARD_SCALES
    if not result.success or np.min(constraints(solution)) < -1e-7:
        raise RuntimeError(f"Board placement optimization failed: {result.message}")
    if not np.allclose(history[-1], solution):
        history.append(solution)
    return solution, np.array(history), result


def optimize_centred(reference_pose):
    """Fix x_B=y_B=0 and optimize z_B and three angles from the reference pose."""
    scales = BOARD_SCALES[2:]

    def expand(scaled):
        return np.r_[0.0, 0.0, scaled * scales]

    initial = expand(reference_pose[2:] / scales)
    history = [initial.copy()]
    result = minimize(
        lambda scaled: information(expand(scaled))[0],
        reference_pose[2:] / scales,
        method="SLSQP",
        bounds=BOARD_BOUNDS[2:] / scales[:, None],
        constraints={"type": "ineq", "fun": lambda scaled: constraints(expand(scaled))},
        callback=lambda scaled: history.append(expand(scaled)),
        options=CENTRED_OPTIONS.copy(),
    )
    solution = expand(result.x)
    if not result.success:
        raise RuntimeError(f"Centred optimization failed: {result.message}")
    validate_feasibility(solution)
    np.testing.assert_array_equal(solution[:2], [0.0, 0.0])
    if not np.array_equal(history[-1], solution):
        history.append(solution.copy())
    return solution, np.array(history), result


def validate_feasibility(state):
    """Check nonlinear constraints AND the explicit translation/angle bounds."""
    if not np.all(np.isfinite(state)):
        raise ValueError("Non-finite board pose")
    slack = np.r_[constraints(state), state - BOARD_BOUNDS[:, 0],
                  BOARD_BOUNDS[:, 1] - state]
    if np.min(slack) < -FEASIBILITY_TOLERANCE:
        raise ValueError(f"Infeasible board pose: minimum slack {np.min(slack)}")


def mirrored_poses(state):
    """Reflect positions AND orientations for this fixed-axis xyz convention.

    Reflecting x changes signs of x_B, beta, gamma; reflecting y changes signs
    of y_B, alpha, gamma. Reflecting both composes those two operations.
    The symmetric board grid is reflected by reversing its corresponding axis.
    """
    signs = {
        "Original": [1, 1, 1, 1, 1, 1],
        "Reflect x": [-1, 1, 1, 1, -1, -1],
        "Reflect y": [1, -1, 1, -1, 1, -1],
        "Reflect x and y": [-1, -1, 1, -1, -1, 1],
    }
    return {label: state * np.array(sign) for label, sign in signs.items()}


def mirror_symmetry_checks(state):
    reference = metrics(state)
    rows = []
    for label, mirrored in mirrored_poses(state).items():
        validate_feasibility(mirrored)
        validate_jacobian(mirrored)
        row = {"placement": label, **metrics(mirrored)}
        for key in ("score", "predicted_translation_rms_mm", "predicted_rotation_rms_deg"):
            np.testing.assert_allclose(row[key], reference[key],
                                       rtol=SYMMETRY_RTOL, atol=SYMMETRY_ATOL)
        rows.append({**row,
                     "score_difference_from_original": row["score"] - reference["score"],
                     "feasible": True, "symmetry_check_passed": True})
    return rows


def reproduce_centred_explanation(solution, free_result, output_dir):
    """Export all data behind the article's centred and mirrored-pose claims."""
    validate_feasibility(solution)
    centred, history, result = optimize_centred(solution)
    validate_jacobian(centred)
    comparison = [
        {"placement": "Centred", **metrics(centred), "optimizer_iterations": int(result.nit)},
        {"placement": "Off-centre", **metrics(solution), "optimizer_iterations": int(free_result.nit)},
    ]
    mirrors = mirror_symmetry_checks(solution)
    score_reduction = 100 * (1 - comparison[1]["score"] / comparison[0]["score"])
    report = {
        "versions": {"python": platform.python_version(), "numpy": np.__version__,
                     "scipy": scipy.__version__, "matplotlib": matplotlib.__version__},
        "platform": {"system": platform.system(), "machine": platform.machine()},
        "pose_order": ["x_m", "y_m", "z_m", "rx_deg", "ry_deg", "rz_deg"],
        "method": "SLSQP",
        "derivatives": "Numerical derivatives with SciPy's default finite-difference step",
        "off_centre_options": OFF_CENTRE_OPTIONS,
        "centred_options": CENTRED_OPTIONS,
        "original_start_pose": INITIAL.tolist(),
        "centred_start_pose": history[0].tolist(),
        "board_variable_scales": BOARD_SCALES.tolist(),
        "camera_tolerance_scales_m_rad": CAMERA_SCALES.tolist(),
        "feasibility_tolerance": FEASIBILITY_TOLERANCE,
        "symmetry_rtol": SYMMETRY_RTOL,
        "symmetry_atol": SYMMETRY_ATOL,
        "off_centre_optimizer_success": bool(free_result.success),
        "off_centre_optimizer_status": free_result.message,
        "centred_optimizer_success": bool(result.success),
        "centred_optimizer_status": result.message,
        "comparison": comparison,
        "off_centre_score_reduction_percent": float(score_reduction),
        "mirror_checks": mirrors,
        "validation": "Predicted covariance only; these diagnostics use no random data or Monte Carlo selection",
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(output_dir / "ocob_centred_optimization_history.csv",
              [{"iteration": i, **metrics(state)} for i, state in enumerate(history)])
    write_csv(output_dir / "ocob_centred_comparison.csv", comparison)
    write_csv(output_dir / "ocob_mirror_symmetry_checks.csv", mirrors)
    with (output_dir / "ocob_centred_reproducibility.json").open("w") as handle:
        json.dump(report, handle, indent=2, allow_nan=False)
        handle.write("\n")
    return report


def residual(camera, points, measured, sigma):
    # A camera-local-to-world pose (R, c) projects R^T (X-c).
    camera_points = (points - camera[:3]) @ Rotation.from_rotvec(camera[3:]).as_matrix()
    return ((project(camera_points) - measured) / sigma[:, None]).ravel()


def validate_jacobian(s):
    points, sigma, _ = board_geometry(s)
    uv = project(points)
    epsilon = 1e-6
    identity = np.eye(6) * epsilon
    finite = np.column_stack([(residual(d, points, uv, sigma) - residual(-d, points, uv, sigma)) / (2 * epsilon) for d in identity])
    np.testing.assert_allclose(camera_jacobian(points, sigma), finite, rtol=1e-6, atol=1e-6)


def monte_carlo(s, trials, seed):
    points, sigma, _ = board_geometry(s)
    truth = project(points)
    rng = np.random.default_rng(seed)
    errors, reprojection = [], []
    for _ in range(trials):
        measured = truth + sigma[:, None] * rng.standard_normal(truth.shape)
        # Model a rough previous camera estimate; no global planar-branch search.
        initial_camera = rng.standard_normal(6) * np.r_[np.full(3, 0.002), np.full(3, np.deg2rad(0.2))]
        result = least_squares(residual, initial_camera, args=(points, measured, sigma), method="lm", ftol=1e-10, xtol=1e-10, gtol=1e-10, max_nfev=500)
        if not result.success:
            raise RuntimeError(f"Camera refinement failed: {result.message}")
        fitted = (points - result.x[:3]) @ Rotation.from_rotvec(result.x[3:]).as_matrix()
        if np.min(fitted[:, 2]) <= 0:
            raise RuntimeError("Camera refinement left the positive-depth branch")
        errors.append(result.x)
        reprojection.append(np.sqrt(np.mean((project(fitted) - measured) ** 2)))
    errors = np.array(errors)
    return {
        "trials": trials,
        "empirical_translation_rms_mm": float(1000 * np.sqrt(np.mean(np.sum(errors[:, :3] ** 2, axis=1)))),
        "empirical_rotation_rms_deg": float(np.rad2deg(np.sqrt(np.mean(np.sum(errors[:, 3:] ** 2, axis=1))))),
        "mean_reprojection_rmse_px_per_coordinate": float(np.mean(reprojection)),
    }, errors


def write_csv(path, rows):
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plot_adjustment(states, history):
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False})
    fig = plt.figure(figsize=(13.0, 5.4), layout="constrained")
    scene = fig.add_subplot(121, projection="3d")
    camera = fig.add_subplot(122)
    scene.plot(history[:, 0], history[:, 1], history[:, 2], color="0.55", ls="--", lw=1.3, alpha=0.9)
    scene.scatter(history[1:-1, 0], history[1:-1, 1], history[1:-1, 2], color="0.55", s=8, label="Numerical optimization path")
    scene.scatter(0, 0, 0, color="#262626", marker="^", s=100, label="Fixed camera")
    scene.quiver(0, 0, 0, 0, 0, 0.45, color="#262626", arrow_length_ratio=0.2)
    scene.text(0.04, 0, 0.04, "Camera")
    for label, state in states.items():
        points, _, _ = board_geometry(state)
        color = COLORS[label]
        outline = points[OUTLINE]
        scene.add_collection3d(Poly3DCollection([outline[:-1]], facecolors=color, alpha=0.23, edgecolors=color, linewidths=1.7))
        scene.scatter(points[:, 0], points[:, 1], points[:, 2], color=color, s=9)
        scene.scatter(*state[:3], color=color, s=50)
        scene.text(state[0] + 0.04, state[1] + 0.04, state[2] + 0.08, label, color=color, weight="bold")
        uv = project(points)
        camera.plot(uv[OUTLINE, 0], uv[OUTLINE, 1], color=color, lw=2.0, label=label)
        camera.scatter(uv[:, 0], uv[:, 1], color=color, s=18)
    scene.set(xlabel="Right x [m]", ylabel="Down y [m]", zlabel="Forward z [m]", title="Move and rotate the same board")
    scene.xaxis.set_major_locator(MaxNLocator(4))
    scene.yaxis.set_major_locator(MaxNLocator(4))
    scene.set_xlim(-0.85, 0.65)
    scene.set_ylim(-0.35, 0.70)
    scene.set_zlim(0, 2.85)
    scene.set_box_aspect((1.5, 1.05, 2.85))
    scene.view_init(elev=20, azim=-65)
    scene.legend(loc="upper left", fontsize=8, frameon=False)
    camera.add_patch(Rectangle((MARGIN, MARGIN), WIDTH - 2 * MARGIN, HEIGHT - 2 * MARGIN, fill=False, ls="--", lw=1, edgecolor="0.65"))
    camera.axvline(PRINCIPAL[0], color="0.90", lw=0.8)
    camera.axhline(PRINCIPAL[1], color="0.90", lw=0.8)
    camera.text(WIDTH - 65, 70, "40 px image margin", ha="right", color="0.45", fontsize=9)
    camera.set(xlim=(0, WIDTH), ylim=(HEIGHT, 0), xlabel="Image u [px]", ylabel="Image v [px]", title="Predicted corner locations in the fixed camera")
    camera.set_aspect("equal")
    camera.legend(loc="upper left", frameon=True, facecolor="white", framealpha=0.95)
    fig.suptitle("One camera, one board: continuous pose optimization", fontsize=15, weight="bold")
    fig.savefig(OUT / "ocob_board_adjustment.png", dpi=180, bbox_inches="tight", pad_inches=0.16)
    plt.close(fig)


def plot_optimization(history, summary):
    fig, axes = plt.subplots(1, 3, figsize=(14.5, 4.6), layout="constrained")
    scores = np.array([information(s)[0] for s in history])
    infeasible = np.array([np.min(constraints(s)) < -1e-6 for s in history])
    steps = np.arange(len(history))
    axes[0].semilogy(steps, scores, color="#266c65", lw=2, marker=".", ms=5, label="SLSQP iterate")
    axes[0].scatter(steps[infeasible], scores[infeasible], marker="x", color="#ad4540", s=40, label="Outside constraints")
    axes[0].annotate(f"{scores[0]:.2f}", (0, scores[0]), xytext=(8, 4), textcoords="offset points")
    axes[0].annotate(f"{scores[-1]:.3f}", (steps[-1], scores[-1]), xytext=(-45, 12), textcoords="offset points")
    axes[0].set(title="Optimizer reduces camera uncertainty", xlabel="Outer optimization iteration", ylabel="Normalized covariance trace U (log scale)", ylim=(0.32, 90))
    axes[0].legend(loc="upper right", fontsize=8, frameon=False)
    categories = np.arange(2)
    for ax, quantity, unit in zip(axes[1:], ["translation", "rotation"], ["mm", "deg"]):
        suffix = f"{quantity}_rms_{unit}"
        predicted = [s[f"predicted_{suffix}"] for s in summary]
        empirical = [s[f"empirical_{suffix}"] for s in summary]
        bars1 = ax.bar(categories - 0.18, predicted, width=0.36, color="#597f9f", label="Predicted from J")
        bars2 = ax.bar(categories + 0.18, empirical, width=0.36, color="#d86528", label=f"{summary[0]['trials']} noisy fits")
        ax.bar_label(bars1, fmt="%.3f" if unit == "deg" else "%.2f", fontsize=9, padding=4)
        ax.bar_label(bars2, fmt="%.3f" if unit == "deg" else "%.2f", fontsize=9, padding=4)
        ax.set_xticks(categories, ["Initial", "Optimized"])
        ax.set(title=f"Camera {quantity} precision", ylabel=f"RMS magnitude of 3D {quantity} error [{unit}]")
        ax.set_ylim(0, max(predicted + empirical) * 1.3)
        ax.legend(loc="upper right", fontsize=8, frameon=False)
    for ax in axes:
        ax.grid(axis="y", alpha=0.18)
        ax.set_axisbelow(True)
    fig.suptitle("Optimize the placement first; validate camera estimation afterward", fontsize=14, weight="bold")
    fig.savefig(OUT / "ocob_optimization_trace.png", dpi=180)
    plt.close(fig)


def plot_camera_views(history):
    """Draw selected actual iterates, not newly evaluated candidate placements."""
    # Include early motion, an infeasible excursion and recovery, and convergence.
    selected = sorted({k for k in (0, 1, 2, 7, 14, 21, 22, 28, len(history) - 1)
                       if 0 <= k < len(history)})
    rows = []
    fig, axes = plt.subplots(3, 3, figsize=(13.5, 11.8))
    fig.subplots_adjust(left=0.065, right=0.985, bottom=0.085, top=0.88,
                        wspace=0.20, hspace=0.40)
    for ax, k in zip(axes.flat, selected):
        state = history[k]
        row = {"iteration": k, **metrics(state)}
        feasible = row["minimum_constraint"] >= -1e-6
        rows.append({**row, "feasible_at_1e_minus6": feasible})
        color = "#266c65" if feasible else "#ad4540"
        if k == 0:
            color = COLORS["Initial"]
        elif k == len(history) - 1:
            color = COLORS["Optimized"]
        points, _, _ = board_geometry(state)
        uv = project(points)
        grid = uv.reshape(5, 7, 2)
        cells, shades = [], []
        for r in range(4):
            for c in range(6):
                cells.append([grid[r, c], grid[r, c + 1],
                              grid[r + 1, c + 1], grid[r + 1, c]])
                shades.append("#353b43" if (r + c) % 2 == 0 else "#f5f5f1")
        # The drawn patch is exactly the 0.60 x 0.40 m corner-grid footprint.
        # No extra board border or synthetic observations are introduced.
        ax.add_collection(PolyCollection(cells, facecolors=shades,
                                         edgecolors="none", zorder=3))
        ax.plot(uv[OUTLINE, 0], uv[OUTLINE, 1], color=color, lw=2, zorder=4)
        ax.scatter(uv[:, 0], uv[:, 1], s=8, color=color, zorder=5)
        ax.add_patch(Rectangle((MARGIN, MARGIN), WIDTH - 2 * MARGIN,
                               HEIGHT - 2 * MARGIN, fill=False, ls="--",
                               lw=0.9, edgecolor="0.62", zorder=2))
        ax.axvline(PRINCIPAL[0], color="0.88", lw=0.7, zorder=1)
        ax.axhline(PRINCIPAL[1], color="0.88", lw=0.7, zorder=1)
        ax.set(xlim=(0, WIDTH), ylim=(HEIGHT, 0),
               xticks=[0, 640, 1280], yticks=[0, 480, 960])
        ax.set_aspect("equal")
        ax.tick_params(labelsize=9, length=3)
        for spine in ax.spines.values():
            spine.set_visible(True)
            spine.set_color("#b86c68" if not feasible else "0.55")
        ax.set_facecolor("#fff7f6" if not feasible else "white")
        stage = " (initial)" if k == 0 else " (final)" if k == len(history) - 1 else ""
        ax.set_title(f"Iteration {k}{stage}  |  U = {row['score']:.3f}",
                     fontsize=11, weight="bold", color=color, pad=8)
        ax.text(0.965, 0.955, "Feasible" if feasible else "Outside constraints",
                transform=ax.transAxes, ha="right", va="top", color=color,
                fontsize=9, bbox={"facecolor": "white", "alpha": 0.92,
                                 "edgecolor": "none", "pad": 2})
        if np.any((uv[:, 0] < 0) | (uv[:, 0] > WIDTH) |
                  (uv[:, 1] < 0) | (uv[:, 1] > HEIGHT)):
            ax.text(0.035, 0.08, "Board cropped by image edge",
                    transform=ax.transAxes, color=color, fontsize=8.5,
                    bbox={"facecolor": "white", "edgecolor": "none", "pad": 2})
    for ax in list(axes.flat)[len(selected):]:
        ax.set_visible(False)
    for ax in axes[-1, :]:
        ax.set_xlabel("Image u [px]", fontsize=10)
    for ax in axes[:, 0]:
        ax.set_ylabel("Image v [px]", fontsize=10)
    fig.suptitle("Board in the camera view during continuous optimization",
                 y=0.972, fontsize=17, weight="bold")
    fig.text(0.5, 0.938,
             "Selected snapshots of ONE SLSQP run, read left to right and top to bottom",
             ha="center", fontsize=11, color="0.3")
    fig.text(0.5, 0.035,
             "All panels: 1280 x 960 px  |  Dashed boundary: 40 px image margin  |  U: camera uncertainty objective\n"
             "Red panels are infeasible numerical iterates, not commanded physical board placements.",
             ha="center", va="center", fontsize=10, color="0.3", linespacing=1.6)
    fig.savefig(OUT / "ocob_camera_view_iterations.png", dpi=200)
    plt.close(fig)
    write_csv(OUT / "ocob_camera_view_iterations.csv", rows)
    return selected


def main():
    global OUT
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", type=int, default=500)
    parser.add_argument("--centred-checks-only", action="store_true",
                        help="Reproduce the centred comparison and symmetry checks without noise trials or figures")
    parser.add_argument("--output-dir", type=Path, default=OUT,
                        help="Directory for generated CSV, JSON, and figure files")
    args = parser.parse_args()
    if args.trials < 1:
        parser.error("--trials must be positive")
    OUT = args.output_dir.resolve()
    solution, history, result = optimize()
    validate_jacobian(INITIAL)
    validate_jacobian(solution)
    OUT.mkdir(parents=True, exist_ok=True)
    diagnostics = reproduce_centred_explanation(solution, result, OUT)
    write_csv(OUT / "ocob_optimization_history.csv", [{"iteration": i, **metrics(state)} for i, state in enumerate(history)])
    if args.centred_checks_only:
        print(json.dumps(diagnostics, indent=2, allow_nan=False))
        return
    states = {"Initial": INITIAL, "Optimized": solution}
    summary = []
    for label, state in states.items():
        row = {"placement": label, **metrics(state)}
        validation, errors = monte_carlo(state, args.trials, seed=20261001)
        row.update(validation)
        summary.append(row)
        write_csv(OUT / f"ocob_{label.lower()}_trials.csv", [dict(zip(["tx_m", "ty_m", "tz_m", "rx_rad", "ry_rad", "rz_rad"], e)) for e in errors])
    write_csv(OUT / "ocob_placement_summary.csv", summary)
    plot_adjustment(states, history)
    plot_optimization(history, summary)
    selected = plot_camera_views(history)
    print(json.dumps({"optimizer": "SLSQP", "iterations": int(result.nit), "status": result.message, "camera_view_iterations": selected, "summary": summary, "centred_diagnostics": diagnostics}, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
