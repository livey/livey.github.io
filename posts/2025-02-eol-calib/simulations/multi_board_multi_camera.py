#!/usr/bin/env python3
"""Multi-board / multi-camera calibration simulation.

This script generates the figures and result tables used by the post section
"Simulation: Multi-Board and Multi-Camera Calibration". It compares:

1. Centralized sparse bundle adjustment over camera and board poses.
2. Point-level ADMM with camera-local world-point copies.

The notation follows the post:
  T^B_b: board b local-to-world pose
  T^C_c: camera c local-to-world pose
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import os

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "resources"
OUT.mkdir(parents=True, exist_ok=True)

RNG = np.random.default_rng(7)


@dataclass(frozen=True)
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


@dataclass
class Observation:
    c: int
    b: int
    i: int
    uv: np.ndarray


@dataclass
class Pose:
    R: np.ndarray
    t: np.ndarray


def normalize(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    if n < 1e-12:
        raise ValueError("Cannot normalize near-zero vector")
    return v / n


def look_at(camera_center: np.ndarray, target: np.ndarray) -> Pose:
    """Return camera local-to-world pose whose +Z axis looks at target."""
    world_up = np.array([0.0, 0.0, 1.0])
    z_axis = normalize(target - camera_center)
    x_axis = np.cross(z_axis, world_up)
    if np.linalg.norm(x_axis) < 1e-8:
        x_axis = np.array([1.0, 0.0, 0.0])
    x_axis = normalize(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    R = np.column_stack([x_axis, y_axis, z_axis])
    return Pose(R=R, t=camera_center)


def board_pose(center: np.ndarray, normal: np.ndarray) -> Pose:
    """Return board local-to-world pose with local z-axis as board normal."""
    world_up = np.array([0.0, 0.0, 1.0])
    z_axis = normalize(normal)
    x_axis = np.cross(world_up, z_axis)
    if np.linalg.norm(x_axis) < 1e-8:
        x_axis = np.array([1.0, 0.0, 0.0])
    x_axis = normalize(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    R = np.column_stack([x_axis, y_axis, z_axis])
    return Pose(R=R, t=center)


def make_board_points(nx: int = 7, ny: int = 5, spacing: float = 0.28) -> np.ndarray:
    xs = (np.arange(nx) - (nx - 1) / 2.0) * spacing
    ys = (np.arange(ny) - (ny - 1) / 2.0) * spacing
    points = []
    for y in ys:
        for x in xs:
            points.append([x, y, 0.0])
    return np.asarray(points, dtype=float)


def transform_points(pose: Pose, q: np.ndarray) -> np.ndarray:
    return q @ pose.R.T + pose.t


def project_point(pose_c: Pose, Xw: np.ndarray, K: Intrinsics) -> tuple[np.ndarray, np.ndarray]:
    x = pose_c.R.T @ (Xw - pose_c.t)
    uv = np.array([K.fx * x[0] / x[2] + K.cx, K.fy * x[1] / x[2] + K.cy])
    return uv, x


def pack_poses(cameras: list[Pose], boards: list[Pose]) -> np.ndarray:
    parts = []
    for pose in cameras + boards:
        parts.append(Rotation.from_matrix(pose.R).as_rotvec())
        parts.append(pose.t)
    return np.concatenate(parts)


def unpack_poses(x: np.ndarray, n_cameras: int, n_boards: int) -> tuple[list[Pose], list[Pose]]:
    poses: list[Pose] = []
    k = 0
    for _ in range(n_cameras + n_boards):
        r = x[k : k + 3]
        t = x[k + 3 : k + 6]
        k += 6
        poses.append(Pose(Rotation.from_rotvec(r).as_matrix(), t.copy()))
    return poses[:n_cameras], poses[n_cameras:]


def perturb_pose(pose: Pose, rot_deg: float, trans_m: float) -> Pose:
    dr = Rotation.from_rotvec(np.deg2rad(rot_deg) * RNG.normal(size=3)).as_matrix()
    dt = trans_m * RNG.normal(size=3)
    return Pose(R=dr @ pose.R, t=pose.t + dt)


def weighted_kabsch(q: np.ndarray, y: np.ndarray, w: np.ndarray) -> Pose:
    w = np.asarray(w, dtype=float)
    wsum = np.sum(w)
    q_bar = np.sum(q * w[:, None], axis=0) / wsum
    y_bar = np.sum(y * w[:, None], axis=0) / wsum
    H = ((q - q_bar) * w[:, None]).T @ (y - y_bar)
    U, _, Vt = np.linalg.svd(H)
    V = Vt.T
    S = np.eye(3)
    S[2, 2] = np.linalg.det(V @ U.T)
    R = V @ S @ U.T
    t = y_bar - R @ q_bar
    return Pose(R=R, t=t)


def rotation_error_deg(estimate: Pose, truth: Pose) -> float:
    delta = truth.R.T @ estimate.R
    return float(np.rad2deg(Rotation.from_matrix(delta).magnitude()))


def translation_error(estimate: Pose, truth: Pose) -> float:
    return float(np.linalg.norm(estimate.t - truth.t))


def reprojection_errors(
    cameras: list[Pose],
    boards: list[Pose],
    q: np.ndarray,
    observations: list[Observation],
    K: Intrinsics,
) -> np.ndarray:
    errs = []
    for obs in observations:
        X = boards[obs.b].R @ q[obs.i] + boards[obs.b].t
        uv, _ = project_point(cameras[obs.c], X, K)
        errs.append(uv - obs.uv)
    return np.asarray(errs)


def camera_subproblem_sparsity(n_edges: int):
    sparsity = lil_matrix((5 * n_edges, 6 + 3 * n_edges), dtype=int)
    for i in range(n_edges):
        row = 5 * i
        point_col = 6 + 3 * i
        sparsity[row : row + 2, 0:6] = 1
        sparsity[row : row + 2, point_col : point_col + 3] = 1
        sparsity[row + 2 : row + 5, point_col : point_col + 3] = 1
    return sparsity.tocsr()


def centralized_sparsity(
    n_cameras: int,
    n_boards: int,
    observations: list[Observation],
    n_points: int,
):
    n_rows = 2 * len(observations) + 3 * n_boards * n_points
    n_cols = 6 * (n_cameras + n_boards)
    sparsity = lil_matrix((n_rows, n_cols), dtype=int)

    for k, obs in enumerate(observations):
        row = 2 * k
        camera_col = 6 * obs.c
        board_col = 6 * (n_cameras + obs.b)
        sparsity[row : row + 2, camera_col : camera_col + 6] = 1
        sparsity[row : row + 2, board_col : board_col + 6] = 1

    row0 = 2 * len(observations)
    for b in range(n_boards):
        board_col = 6 * (n_cameras + b)
        for i in range(n_points):
            row = row0 + 3 * (b * n_points + i)
            sparsity[row : row + 3, board_col : board_col + 6] = 1

    return sparsity.tocsr()


def pose_error_table(
    label: str,
    estimates: list[Pose],
    truth: list[Pose],
    prefix: str,
) -> pd.DataFrame:
    rows = []
    for idx, (est, gt) in enumerate(zip(estimates, truth)):
        rows.append(
            {
                "method": label,
                "id": f"{prefix}{idx}",
                "rotation_deg": rotation_error_deg(est, gt),
                "translation_m": translation_error(est, gt),
            }
        )
    return pd.DataFrame(rows)


def simulate_data() -> dict:
    K = Intrinsics(fx=720.0, fy=720.0, cx=650.0, cy=500.0, width=1300, height=1000)
    q = make_board_points()

    cameras_true = [
        look_at(np.array([-3.3, -4.3, 1.9]), np.array([-0.45, -0.15, 0.95])),
        look_at(np.array([3.3, -4.2, 1.95]), np.array([0.45, -0.15, 0.95])),
        look_at(np.array([-3.2, 4.3, 1.85]), np.array([-0.6, 0.75, 0.95])),
        look_at(np.array([3.2, 4.2, 1.9]), np.array([0.6, 0.75, 0.95])),
    ]

    boards_true = [
        board_pose(np.array([0.0, -1.15, 0.95]), np.array([0.0, -1.0, 0.25])),
        board_pose(np.array([-1.05, 0.55, 1.05]), np.array([-1.0, 0.04, 0.25])),
        board_pose(np.array([1.05, 0.55, 1.00]), np.array([1.0, 0.04, 0.22])),
    ]

    sigma_px = 0.75
    sigma_z = 0.012
    observations: list[Observation] = []
    visibility = np.zeros((len(cameras_true), len(boards_true)), dtype=int)

    for c, camera in enumerate(cameras_true):
        for b, board in enumerate(boards_true):
            n = board.R[:, 2]
            count = 0
            for i, qi in enumerate(q):
                X = board.R @ qi + board.t
                uv, xc = project_point(camera, X, K)
                view_dir = normalize(camera.t - X)
                incidence = float(n @ view_dir)
                visible = (
                    xc[2] > 0.6
                    and 35.0 < uv[0] < K.width - 35.0
                    and 35.0 < uv[1] < K.height - 35.0
                    and incidence > 0.02
                    and RNG.random() > 0.05
                )
                if visible:
                    uv_noisy = uv + RNG.normal(scale=sigma_px, size=2)
                    observations.append(Observation(c=c, b=b, i=i, uv=uv_noisy))
                    count += 1
            visibility[c, b] = count

    z_meas = []
    for board in boards_true:
        X = transform_points(board, q)
        z_meas.append(X + RNG.normal(scale=sigma_z, size=X.shape))

    boards_init = [weighted_kabsch(q, z, np.ones(len(q))) for z in z_meas]
    cameras_init = [perturb_pose(p, rot_deg=2.5, trans_m=0.14) for p in cameras_true]

    return {
        "K": K,
        "q": q,
        "cameras_true": cameras_true,
        "boards_true": boards_true,
        "cameras_init": cameras_init,
        "boards_init": boards_init,
        "observations": observations,
        "visibility": visibility,
        "z_meas": z_meas,
        "sigma_px": sigma_px,
        "sigma_z": sigma_z,
    }


def centralized_ba(data: dict) -> dict:
    K = data["K"]
    q = data["q"]
    observations = data["observations"]
    z_meas = data["z_meas"]
    sigma_px = data["sigma_px"]
    sigma_z = data["sigma_z"]
    n_cameras = len(data["cameras_true"])
    n_boards = len(data["boards_true"])

    def residual(x: np.ndarray) -> np.ndarray:
        cameras, boards = unpack_poses(x, n_cameras, n_boards)
        res = []
        for obs in observations:
            X = boards[obs.b].R @ q[obs.i] + boards[obs.b].t
            uv, xc = project_point(cameras[obs.c], X, K)
            if xc[2] <= 0.1:
                res.extend([1e3, 1e3])
            else:
                res.extend((uv - obs.uv) / sigma_px)
        for b, board in enumerate(boards):
            X = transform_points(board, q)
            res.extend(((X - z_meas[b]) / sigma_z).ravel())
        return np.asarray(res)

    x0 = pack_poses(data["cameras_init"], data["boards_init"])
    sparsity = centralized_sparsity(n_cameras, n_boards, observations, len(q))
    result = least_squares(
        residual,
        x0,
        method="trf",
        x_scale="jac",
        jac_sparsity=sparsity,
        tr_solver="lsmr",
        loss="linear",
        max_nfev=160,
        ftol=1e-10,
        xtol=1e-10,
        gtol=1e-10,
    )
    cameras, boards = unpack_poses(result.x, n_cameras, n_boards)
    return {"cameras": cameras, "boards": boards, "result": result}


def admm_solver(data: dict, iterations: int = 22, mu: float = 6500.0) -> dict:
    K = data["K"]
    q = data["q"]
    observations: list[Observation] = data["observations"]
    z_meas = data["z_meas"]
    sigma_px = data["sigma_px"]
    sigma_z = data["sigma_z"]
    n_cameras = len(data["cameras_true"])
    n_boards = len(data["boards_true"])

    cameras = [Pose(p.R.copy(), p.t.copy()) for p in data["cameras_init"]]
    boards = [Pose(p.R.copy(), p.t.copy()) for p in data["boards_init"]]

    obs_by_camera = [[] for _ in range(n_cameras)]
    obs_by_board = [[] for _ in range(n_boards)]
    for k, obs in enumerate(observations):
        obs_by_camera[obs.c].append(k)
        obs_by_board[obs.b].append(k)
    sparsity_by_camera = {
        c: camera_subproblem_sparsity(len(keys))
        for c, keys in enumerate(obs_by_camera)
        if keys
    }

    Y = np.zeros((len(observations), 3))
    lam = np.zeros_like(Y)
    for k, obs in enumerate(observations):
        Y[k] = boards[obs.b].R @ q[obs.i] + boards[obs.b].t

    history = []

    for it in range(iterations + 1):
        errs = reprojection_errors(cameras, boards, q, observations, K)
        primal = []
        for k, obs in enumerate(observations):
            Xb = boards[obs.b].R @ q[obs.i] + boards[obs.b].t
            primal.append(Y[k] - Xb)
        history.append(
            {
                "iteration": it,
                "rmse_px": float(np.sqrt(np.mean(np.sum(errs**2, axis=1)))),
                "primal_m": float(np.sqrt(np.mean(np.sum(np.asarray(primal) ** 2, axis=1)))),
                "camera_rot_deg": float(
                    np.mean(
                        [
                            rotation_error_deg(cameras[i], data["cameras_true"][i])
                            for i in range(n_cameras)
                        ]
                    )
                ),
                "camera_trans_m": float(
                    np.mean(
                        [
                            translation_error(cameras[i], data["cameras_true"][i])
                            for i in range(n_cameras)
                        ]
                    )
                ),
            }
        )
        if it == iterations:
            break

        for c in range(n_cameras):
            keys = obs_by_camera[c]
            if not keys:
                continue
            pose0 = cameras[c]
            x0 = np.concatenate(
                [Rotation.from_matrix(pose0.R).as_rotvec(), pose0.t, Y[keys].ravel()]
            )

            def camera_residual(x: np.ndarray, camera_id: int = c, edge_keys: list[int] = keys) -> np.ndarray:
                R = Rotation.from_rotvec(x[:3]).as_matrix()
                t = x[3:6]
                pose = Pose(R, t)
                Ys = x[6:].reshape((-1, 3))
                res = []
                for local_idx, edge_id in enumerate(edge_keys):
                    obs = observations[edge_id]
                    uv, xc = project_point(pose, Ys[local_idx], K)
                    if xc[2] <= 0.1:
                        res.extend([1e3, 1e3])
                    else:
                        res.extend((uv - obs.uv) / sigma_px)
                    Xb = boards[obs.b].R @ q[obs.i] + boards[obs.b].t
                    res.extend(np.sqrt(mu) * (Ys[local_idx] - Xb + lam[edge_id]))
                return np.asarray(res)

            sol = least_squares(
                camera_residual,
                x0,
                method="trf",
                x_scale="jac",
                jac_sparsity=sparsity_by_camera[c],
                tr_solver="lsmr",
                max_nfev=45,
                ftol=5e-8,
                xtol=5e-8,
                gtol=5e-8,
            )
            cameras[c] = Pose(Rotation.from_rotvec(sol.x[:3]).as_matrix(), sol.x[3:6].copy())
            Y[keys] = sol.x[6:].reshape((-1, 3))

        for b in range(n_boards):
            q_targets = []
            y_targets = []
            weights = []
            for edge_id in obs_by_board[b]:
                obs = observations[edge_id]
                q_targets.append(q[obs.i])
                y_targets.append(Y[edge_id] + lam[edge_id])
                weights.append(mu)
            for i in range(len(q)):
                q_targets.append(q[i])
                y_targets.append(z_meas[b][i])
                weights.append(1.0 / sigma_z**2)
            boards[b] = weighted_kabsch(
                np.asarray(q_targets), np.asarray(y_targets), np.asarray(weights)
            )

        for k, obs in enumerate(observations):
            Xb = boards[obs.b].R @ q[obs.i] + boards[obs.b].t
            lam[k] += Y[k] - Xb

    return {"cameras": cameras, "boards": boards, "Y": Y, "lambda": lam, "history": pd.DataFrame(history)}


def solver_summary(data: dict, central: dict, admm: dict) -> tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame]:
    q = data["q"]
    observations = data["observations"]
    K = data["K"]
    camera_truth = data["cameras_true"]
    board_truth = data["boards_true"]
    methods = [
        ("Initial", data["cameras_init"], data["boards_init"]),
        ("Centralized BA", central["cameras"], central["boards"]),
        ("Point-level ADMM", admm["cameras"], admm["boards"]),
    ]

    camera_rows = []
    board_rows = []
    summary_rows = []
    for name, cameras, boards in methods:
        c_df = pose_error_table(name, cameras, camera_truth, "C")
        b_df = pose_error_table(name, boards, board_truth, "B")
        camera_rows.append(c_df)
        board_rows.append(b_df)
        errs = reprojection_errors(cameras, boards, q, observations, K)
        summary_rows.append(
            {
                "method": name,
                "reprojection_rmse_px": np.sqrt(np.mean(np.sum(errs**2, axis=1))),
                "mean_camera_rotation_deg": c_df["rotation_deg"].mean(),
                "mean_camera_translation_m": c_df["translation_m"].mean(),
                "mean_board_rotation_deg": b_df["rotation_deg"].mean(),
                "mean_board_translation_m": b_df["translation_m"].mean(),
            }
        )
    return pd.concat(camera_rows), pd.concat(board_rows), pd.DataFrame(summary_rows)


def plot_scene(data: dict, path: Path) -> None:
    K = data["K"]
    q = data["q"]
    cameras = data["cameras_true"]
    boards = data["boards_true"]
    observations = data["observations"]

    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection="3d")
    colors = ["tab:blue", "tab:orange", "tab:green"]

    for b, board in enumerate(boards):
        X = transform_points(board, q)
        ax.scatter(X[:, 0], X[:, 1], X[:, 2], s=18, color=colors[b], label=f"board {b}")
        center = board.t
        normal = board.R[:, 2]
        ax.quiver(center[0], center[1], center[2], normal[0], normal[1], normal[2], length=0.45, color=colors[b])

    for c, camera in enumerate(cameras):
        ax.scatter(camera.t[0], camera.t[1], camera.t[2], marker="^", s=80, color="black")
        ax.text(camera.t[0], camera.t[1], camera.t[2] + 0.12, f"C{c}", color="black")
        corners = np.array([[0, 0], [K.width, 0], [K.width, K.height], [0, K.height]])
        rays = []
        for u, v in corners:
            ray = np.array([(u - K.cx) / K.fx, (v - K.cy) / K.fy, 1.0])
            rays.append(camera.t + camera.R @ (0.65 * ray / np.linalg.norm(ray)))
        rays = np.asarray(rays)
        for r in rays:
            ax.plot([camera.t[0], r[0]], [camera.t[1], r[1]], [camera.t[2], r[2]], color="0.35", lw=0.8)
        for j in range(4):
            a, b2 = rays[j], rays[(j + 1) % 4]
            ax.plot([a[0], b2[0]], [a[1], b2[1]], [a[2], b2[2]], color="0.35", lw=0.8)

    edge_counts = {}
    for obs in observations:
        edge_counts[(obs.c, obs.b)] = edge_counts.get((obs.c, obs.b), 0) + 1
    for (c, b), count in edge_counts.items():
        if count < 5:
            continue
        cam = cameras[c].t
        board = boards[b].t
        ax.plot([cam[0], board[0]], [cam[1], board[1]], [cam[2], board[2]], color=colors[b], alpha=0.18)

    ax.set_xlabel("world x [m]")
    ax.set_ylabel("world y [m]")
    ax.set_zlabel("world z [m]")
    ax.set_title("Simulated multi-camera / multi-board observation graph")
    ax.view_init(elev=23, azim=-48)
    ax.legend(loc="upper left")
    ax.set_box_aspect((1.2, 1.2, 0.55))
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_observations(data: dict, central: dict, path: Path) -> None:
    K = data["K"]
    q = data["q"]
    observations = data["observations"]
    colors = ["tab:blue", "tab:orange", "tab:green"]

    fig, axes = plt.subplots(2, 2, figsize=(10, 7), sharex=True, sharey=True)
    axes = axes.ravel()
    for c, ax in enumerate(axes):
        ax.set_title(f"camera {c}")
        ax.set_xlim(0, K.width)
        ax.set_ylim(K.height, 0)
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.25)
        for b in range(len(data["boards_true"])):
            pts = np.array([obs.uv for obs in observations if obs.c == c and obs.b == b])
            if len(pts):
                ax.scatter(pts[:, 0], pts[:, 1], s=10, color=colors[b], label=f"B{b}", alpha=0.75)
        for obs in observations:
            if obs.c != c:
                continue
            X = central["boards"][obs.b].R @ q[obs.i] + central["boards"][obs.b].t
            uv, _ = project_point(central["cameras"][obs.c], X, K)
            ax.plot([obs.uv[0], uv[0]], [obs.uv[1], uv[1]], color="black", lw=0.35, alpha=0.35)
        if c == 0:
            ax.legend(loc="lower right", fontsize=8)
    fig.suptitle("Noisy observations and centralized BA residual vectors")
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_visibility(data: dict, path: Path) -> pd.DataFrame:
    vis = data["visibility"]
    df = pd.DataFrame(vis, index=[f"C{i}" for i in range(vis.shape[0])], columns=[f"B{j}" for j in range(vis.shape[1])])
    fig, ax = plt.subplots(figsize=(6.2, 3.8))
    im = ax.imshow(vis, cmap="Blues")
    for i in range(vis.shape[0]):
        for j in range(vis.shape[1]):
            ax.text(j, i, str(vis[i, j]), ha="center", va="center", color="black")
    ax.set_xticks(np.arange(vis.shape[1]), labels=df.columns)
    ax.set_yticks(np.arange(vis.shape[0]), labels=df.index)
    ax.set_xlabel("board")
    ax.set_ylabel("camera")
    ax.set_title("Observed marker count per camera-board pair")
    fig.colorbar(im, ax=ax, shrink=0.8, label="observed points")
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return df


def plot_convergence(admm: dict, summary: pd.DataFrame, path: Path) -> None:
    hist = admm["history"]
    central_rmse = float(summary.loc[summary["method"] == "Centralized BA", "reprojection_rmse_px"].iloc[0])
    fig, axes = plt.subplots(1, 3, figsize=(13, 3.6))
    axes[0].plot(hist["iteration"], hist["rmse_px"], marker="o", ms=3)
    axes[0].axhline(central_rmse, ls="--", color="black", lw=1, label="centralized BA")
    axes[0].set_title("reprojection RMSE")
    axes[0].set_xlabel("ADMM iteration")
    axes[0].set_ylabel("px")
    axes[0].legend(fontsize=8)

    axes[1].plot(hist["iteration"], hist["primal_m"], marker="o", ms=3, color="tab:orange")
    axes[1].set_title("ADMM primal residual")
    axes[1].set_xlabel("ADMM iteration")
    axes[1].set_ylabel("m")

    axes[2].plot(hist["iteration"], hist["camera_rot_deg"], label="rot deg", marker="o", ms=3)
    axes[2].plot(hist["iteration"], hist["camera_trans_m"], label="trans m", marker="o", ms=3)
    axes[2].set_title("mean camera pose error")
    axes[2].set_xlabel("ADMM iteration")
    axes[2].legend(fontsize=8)
    for ax in axes:
        ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_error_bars(camera_df: pd.DataFrame, board_df: pd.DataFrame, path: Path) -> None:
    methods = ["Initial", "Centralized BA", "Point-level ADMM"]
    fig, axes = plt.subplots(2, 2, figsize=(11, 7))
    for ax, df, metric, title in [
        (axes[0, 0], camera_df, "rotation_deg", "camera rotation error"),
        (axes[0, 1], camera_df, "translation_m", "camera translation error"),
        (axes[1, 0], board_df, "rotation_deg", "board rotation error"),
        (axes[1, 1], board_df, "translation_m", "board translation error"),
    ]:
        means = [df[df["method"] == m][metric].mean() for m in methods]
        stds = [df[df["method"] == m][metric].std(ddof=0) for m in methods]
        ax.bar(np.arange(len(methods)), means, yerr=stds, capsize=4, color=["0.65", "tab:blue", "tab:green"])
        ax.set_xticks(np.arange(len(methods)), labels=methods, rotation=12, ha="right")
        ax.set_title(title)
        ax.grid(True, axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_residual_hist(data: dict, central: dict, admm: dict, path: Path) -> None:
    methods = [
        ("Initial", data["cameras_init"], data["boards_init"], "0.55"),
        ("Centralized BA", central["cameras"], central["boards"], "tab:blue"),
        ("Point-level ADMM", admm["cameras"], admm["boards"], "tab:green"),
    ]
    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    for name, cameras, boards, color in methods:
        errs = reprojection_errors(cameras, boards, data["q"], data["observations"], data["K"])
        norms = np.linalg.norm(errs, axis=1)
        ax.hist(norms, bins=28, alpha=0.45, density=True, label=name, color=color)
    ax.set_xlabel("reprojection residual norm [px]")
    ax.set_ylabel("density")
    ax.set_title("Residual distribution")
    ax.grid(True, alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def write_outputs(data: dict, central: dict, admm: dict) -> None:
    camera_df, board_df, summary_df = solver_summary(data, central, admm)
    visibility_df = plot_visibility(data, OUT / "multi_calib_visibility.png")

    plot_scene(data, OUT / "multi_calib_scene.png")
    plot_observations(data, central, OUT / "multi_calib_observations.png")
    plot_convergence(admm, summary_df, OUT / "multi_calib_convergence.png")
    plot_error_bars(camera_df, board_df, OUT / "multi_calib_pose_errors.png")
    plot_residual_hist(data, central, admm, OUT / "multi_calib_residual_hist.png")

    summary_df.to_csv(OUT / "multi_calib_solver_summary.csv", index=False)
    camera_df.to_csv(OUT / "multi_calib_camera_errors.csv", index=False)
    board_df.to_csv(OUT / "multi_calib_board_errors.csv", index=False)
    visibility_df.to_csv(OUT / "multi_calib_visibility.csv")
    admm["history"].to_csv(OUT / "multi_calib_admm_history.csv", index=False)

    def fmt(df: pd.DataFrame, digits: int = 4) -> str:
        return df.to_markdown(index=False, floatfmt=f".{digits}f")

    summary_display = summary_df.copy()
    camera_display = camera_df.copy()
    board_display = board_df.copy()
    for df in [summary_display, camera_display, board_display]:
        for col in df.select_dtypes(include=[float]).columns:
            df[col] = df[col].astype(float)

    md = "\n".join(
        [
            "#### Solver Summary",
            "",
            fmt(summary_display),
            "",
            "#### Per-Camera Pose Errors",
            "",
            fmt(camera_display),
            "",
            "#### Per-Board Pose Errors",
            "",
            fmt(board_display),
            "",
            "#### Visibility Counts",
            "",
            visibility_df.to_markdown(),
            "",
        ]
    )
    (OUT / "multi_calib_results.md").write_text(md, encoding="utf-8")


def main() -> None:
    data = simulate_data()
    if len(data["observations"]) < 80:
        raise RuntimeError(f"Too few observations generated: {len(data['observations'])}")
    central = centralized_ba(data)
    admm = admm_solver(data)
    write_outputs(data, central, admm)
    print(f"observations: {len(data['observations'])}")
    print((OUT / "multi_calib_results.md").read_text(encoding="utf-8"))


if __name__ == "__main__":
    main()
