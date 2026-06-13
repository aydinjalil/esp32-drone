#!/usr/bin/env python3
"""Magnetometer hard- and soft-iron calibration + verification.

Fits an ellipsoid to raw magnetometer samples and derives:

  * hard-iron offset  b  (3-vector, the ellipsoid centre)
  * soft-iron matrix  A  (3x3, maps the ellipsoid onto a sphere)

so that a calibrated sample is:

      m_cal = A @ (m_raw - b)

The full 3x3 A corrects a *tilted* ellipsoid, which a per-axis scale (diagonal)
model cannot. The script also evaluates any existing diagonal constants so you
can see, in one place, how good your current calibration is and how much the
full fit improves it.

Verification metric
-------------------
A perfectly calibrated magnetometer reports a constant field magnitude |m| in
every orientation -- the calibrated cloud is a sphere centred on the origin. We
report the coefficient of variation  CV = 100 * std(|m|) / mean(|m|).
Lower is better; good calibrations are typically a few percent.

Usage
-----
    python mag_calibrate.py PATH/TO/mag_log.csv [--plot]

CSV format: header "mx,my,mz" followed by raw rows (the format written by
mag_calibration.py / produced by mag_capture_raw.ino).
"""

import argparse
import sys

import numpy as np


# Existing DIAGONAL constants currently in imu_debug.ino, for side-by-side
# comparison. Update these if you change the firmware defaults.
CURRENT_OFFSET = np.array([-3.97, 17.33, 64.50])
CURRENT_SCALE = np.array([1.06, 1.17, 0.83])


def load_csv(path):
    """Load raw mx,my,mz rows, skipping a header and any malformed lines."""
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split(",")
            if len(parts) != 3:
                continue
            try:
                rows.append([float(p) for p in parts])
            except ValueError:
                continue  # header or junk line
    data = np.asarray(rows, dtype=float)
    if data.ndim != 2 or data.shape[0] < 100:
        raise SystemExit(
            f"Need at least ~100 valid samples for a stable fit; got "
            f"{0 if data.size == 0 else data.shape[0]}. Capture more data."
        )
    return data


def fit_ellipsoid(xyz):
    """Algebraic least-squares ellipsoid fit.

    Fits  a x^2 + b y^2 + c z^2 + 2d xy + 2e xz + 2f yz + 2g x + 2h y + 2i z = 1
    and returns (offset b_vec, soft-iron matrix A, radius).

    A maps raw samples onto a sphere of `radius` (the mean ellipsoid semi-axis),
    keeping calibrated values in physical uT-scale rather than unit-normalised.
    """
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    D = np.column_stack([
        x * x, y * y, z * z,
        2 * x * y, 2 * x * z, 2 * y * z,
        2 * x, 2 * y, 2 * z,
    ])
    ones = np.ones_like(x)
    # Least-squares solution to D @ v = 1
    v, *_ = np.linalg.lstsq(D, ones, rcond=None)
    a, b, c, d, e, f, g, h, i = v

    # Quadratic form Q and linear term u
    Q = np.array([
        [a, d, e],
        [d, b, f],
        [e, f, c],
    ])
    u = np.array([g, h, i])

    # Ellipsoid centre (hard-iron offset): Q @ center = -u
    center = np.linalg.solve(Q, -u)

    # Shift to centred form: (x-c)^T Q (x-c) = k
    k = 1.0 + center @ Q @ center
    M = Q / k  # so (x-c)^T M (x-c) = 1

    # Eigendecomposition of the (symmetric) M. Eigenvalues must be > 0 for a
    # proper ellipsoid; semi-axis length along eigvec i is 1/sqrt(lambda_i).
    eigvals, eigvecs = np.linalg.eigh(M)
    if np.any(eigvals <= 0):
        raise SystemExit(
            "Ellipsoid fit failed (non-positive eigenvalues). This usually "
            "means poor orientation coverage in the capture. Recapture with "
            "more complete figure-8 / full-rotation motion."
        )

    semi_axes = 1.0 / np.sqrt(eigvals)
    radius = float(np.mean(semi_axes))

    # Symmetric square root of M, scaled to map onto a sphere of `radius`:
    #   A = radius * V diag(sqrt(lambda)) V^T   ->   |A (x-c)| = radius
    A = radius * (eigvecs @ np.diag(np.sqrt(eigvals)) @ eigvecs.T)

    return center, A, radius, semi_axes


def cv_percent(mags):
    """Coefficient of variation of magnitudes, in percent."""
    return 100.0 * np.std(mags) / np.mean(mags)


def report(name, mags):
    cv = cv_percent(mags)
    print(
        f"  {name:<28} mean|m|={np.mean(mags):8.2f}  "
        f"std={np.std(mags):6.2f}  CV={cv:5.2f}%"
    )
    return cv


def fmt_c_array3(v):
    return "{ %sf, %sf, %sf }" % tuple(f"{x:.4f}" for x in v)


def emit_firmware(center, A):
    print("\n=== Paste into imu_debug.ino (native sensor frame) ===\n")
    print(f"const float MAG_B[3] = {fmt_c_array3(center)};")
    print("const float MAG_A[3][3] = {")
    for row in A:
        print(f"  {fmt_c_array3(row)},")
    print("};")


def maybe_plot(raw, cal_full, center):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:  # noqa: BLE001
        print(f"\n[plot skipped: {exc}]")
        return

    planes = [(0, 1, "XY"), (1, 2, "YZ"), (0, 2, "XZ")]
    fig, axes = plt.subplots(2, 3, figsize=(13, 8))
    for col, (i, j, label) in enumerate(planes):
        ax0 = axes[0, col]
        ax0.scatter(raw[:, i], raw[:, j], s=3, c="#cc4444", alpha=0.4)
        ax0.scatter([center[i]], [center[j]], c="black", marker="+", s=120)
        ax0.set_title(f"raw {label}")
        ax0.set_aspect("equal", "box")
        ax0.grid(True, alpha=0.3)

        ax1 = axes[1, col]
        ax1.scatter(cal_full[:, i], cal_full[:, j], s=3, c="#3388cc", alpha=0.4)
        ax1.axhline(0, color="gray", lw=0.5)
        ax1.axvline(0, color="gray", lw=0.5)
        ax1.set_title(f"calibrated {label}")
        ax1.set_aspect("equal", "box")
        ax1.grid(True, alpha=0.3)

    fig.suptitle("Magnetometer calibration: raw (top) vs full 3x3 fit (bottom)")
    fig.tight_layout()
    out = "mag_calibration_check.png"
    fig.savefig(out, dpi=110)
    print(f"\n[plot saved to {out}]")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", help="raw mx,my,mz CSV (e.g. mag_log.csv)")
    ap.add_argument("--plot", action="store_true",
                    help="save a before/after scatter PNG")
    args = ap.parse_args()

    # Apple's Accelerate BLAS can emit spurious divide/overflow/invalid
    # RuntimeWarnings from matmul on some shapes; results are unaffected.
    np.seterr(divide="ignore", over="ignore", invalid="ignore")

    raw = load_csv(args.csv)
    print(f"Loaded {raw.shape[0]} samples from {args.csv}\n")

    # --- Full 3x3 ellipsoid fit ---
    center, A, radius, semi_axes = fit_ellipsoid(raw)
    cal_full = (A @ (raw - center).T).T

    # --- Existing diagonal constants, for comparison ---
    cal_diag = (raw - CURRENT_OFFSET) * CURRENT_SCALE

    print("Field-magnitude consistency (lower CV = better; sphere = constant |m|):")
    cv_raw = report("raw (uncalibrated)", np.linalg.norm(raw, axis=1))
    cv_diag = report("current diagonal constants", np.linalg.norm(cal_diag, axis=1))
    cv_full = report("full 3x3 fit (this run)", np.linalg.norm(cal_full, axis=1))

    print("\nEllipsoid shape:")
    print(f"  centre (hard iron)        = "
          f"[{center[0]:.2f}, {center[1]:.2f}, {center[2]:.2f}]")
    print(f"  semi-axes                 = "
          f"[{semi_axes[0]:.2f}, {semi_axes[1]:.2f}, {semi_axes[2]:.2f}]  "
          f"(target radius {radius:.2f})")
    axis_ratio = semi_axes.max() / semi_axes.min()
    print(f"  max/min axis ratio        = {axis_ratio:.2f}  "
          f"(1.0 = perfect sphere; >1.2 means soft iron matters)")
    off_diag = np.abs(A - np.diag(np.diag(A))).max()
    diag_mean = np.abs(np.diag(A)).mean()
    print(f"  soft-iron off-diagonal    = {off_diag:.4f}  "
          f"({100 * off_diag / diag_mean:.1f}% of diagonal; "
          f"large = tilted ellipsoid a diagonal model can't fix)")

    print("\nVerdict:")
    if cv_full < cv_diag:
        print(f"  Full 3x3 fit improves consistency: "
              f"CV {cv_diag:.2f}% -> {cv_full:.2f}%.")
    else:
        print(f"  Full fit did not beat current constants "
              f"(CV {cv_full:.2f}% vs {cv_diag:.2f}%); current cal may already "
              f"be fine, or capture coverage is poor.")
    if cv_full > 5.0:
        print("  WARNING: CV > 5% even after fitting -- suspect incomplete "
              "orientation coverage or magnetic interference during capture.")

    emit_firmware(center, A)

    if args.plot:
        maybe_plot(raw, cal_full, center)


if __name__ == "__main__":
    sys.exit(main())
