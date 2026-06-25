#!/usr/bin/env python3
"""Ellipse-fit calibration for the HALMET wind vane (sin/cos centering).

Feed it the serial log captured while motoring the boat in one slow full circle.
It reads the same lines the 1 Hz console already prints, e.g.

    I (12345) halmet-wind: tick 42  sin=4.231V cos=3.880V  AWA=... AWS=...

fits an axis-aligned ellipse to the (sin, cos) cloud, and prints the four numbers
to type into the web UI (http://halmet-wind.local/):

    Sin centering (Linear):  slope = 1/amp_sin,  offset = -Vmid_sin/amp_sin
    Cos centering (Linear):  slope = 1/amp_cos,  offset = -Vmid_cos/amp_cos

Pure standard library — no numpy, runs on any Python 3. The model is axis-aligned
(no cross term) because the ST60+ sin and cos channels are independent; a large
'tilt' warning means real cross-coupling that the per-channel Linear can't fix.

Usage:
    python3 ellipse_fit.py [logfile]        # default: /tmp/halmet_mon.log
"""
import re
import sys

POINT = re.compile(r"sin=(-?\d+(?:\.\d+)?)V\s+cos=(-?\d+(?:\.\d+)?)V")


def read_points(path):
    pts = []
    with open(path) as f:
        for line in f:
            m = POINT.search(line)
            if m:
                pts.append((float(m.group(1)), float(m.group(2))))
    return pts


def solve(M):
    """Gauss-Jordan on an n x (n+1) augmented matrix; returns the solution."""
    n = len(M)
    for i in range(n):
        p = max(range(i, n), key=lambda r: abs(M[r][i]))
        M[i], M[p] = M[p], M[i]
        if abs(M[i][i]) < 1e-12:
            raise ValueError("singular system - not enough angular spread")
        piv = M[i][i]
        M[i] = [v / piv for v in M[i]]
        for r in range(n):
            if r != i and M[r][i]:
                f = M[r][i]
                M[r] = [a - f * b for a, b in zip(M[r], M[i])]
    return [M[i][n] for i in range(n)]


def fit_axis_aligned(pts):
    """Least-squares fit of  a*x^2 + c*y^2 + d*x + e*y = 1 ; return ellipse params.

    The raw sin/cos sit near 4 V, so the x^2 terms swamp the x terms and the
    normal matrix is badly conditioned. Mean-centering the data first keeps the
    basis well-scaled; the recovered center is translated back at the end.
    """
    n = len(pts)
    xm = sum(p[0] for p in pts) / n
    ym = sum(p[1] for p in pts) / n
    N = [[0.0] * 5 for _ in range(4)]  # 4x4 normal matrix + RHS column
    for px, py in pts:
        x, y = px - xm, py - ym
        b = (x * x, y * y, x, y)
        for i in range(4):
            for j in range(4):
                N[i][j] += b[i] * b[j]
            N[i][4] += b[i]
    a, c, d, e = solve(N)
    if a <= 0 or c <= 0:
        raise ValueError("fit is not an ellipse (boat did not turn enough)")
    cx, cy = -d / (2 * a), -e / (2 * c)  # center in mean-centered coords
    k = 1.0 + a * cx * cx + c * cy * cy  # RHS after completing the square
    return cx + xm, cy + ym, (k / a) ** 0.5, (k / c) ** 0.5


def tilt_warning(pts, x0, y0):
    """Correlation of centered points; ~0 for axis-aligned, large => cross-coupling."""
    sx = sxx = syy = sxy = 0.0
    for x, y in pts:
        u, v = x - x0, y - y0
        sxx += u * u
        syy += v * v
        sxy += u * v
        sx += 1
    denom = (sxx * syy) ** 0.5
    return abs(sxy / denom) if denom else 0.0


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/halmet_mon.log"
    pts = read_points(path)
    if len(pts) < 20:
        print(f"Only {len(pts)} (sin,cos) points in {path}.")
        print("Need a slow full circle (~60+ points at 1 Hz). Log more and retry.")
        return 1
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    print(f"{len(pts)} points | sin {min(xs):.3f}..{max(xs):.3f} V | "
          f"cos {min(ys):.3f}..{max(ys):.3f} V")
    try:
        x0, y0, ax, ay = fit_axis_aligned([list(p) for p in pts])
    except ValueError as ex:
        print(f"Fit failed: {ex}")
        print("Most likely the boat did not turn through a full circle - re-log.")
        return 1

    r = tilt_warning(pts, x0, y0)
    print()
    print(f"  Vmid_sin = {x0:.4f} V   amplitude_sin = {ax:.4f} V")
    print(f"  Vmid_cos = {y0:.4f} V   amplitude_cos = {ay:.4f} V")
    print(f"  ellipticity (amp_cos/amp_sin) = {ay / ax:.3f}  (1.000 = circle)")
    if r > 0.20:
        print(f"  ! tilt/correlation {r:.2f} is high - sin/cos may be cross-coupled;")
        print(f"    per-channel centering can't fully fix a rotated ellipse.")
    print()
    print("Type into the web UI (http://halmet-wind.local/):")
    print(f"  Sin centering : slope = {1 / ax:.4f}   offset = {-x0 / ax:.4f}")
    print(f"  Cos centering : slope = {1 / ay:.4f}   offset = {-y0 / ay:.4f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
