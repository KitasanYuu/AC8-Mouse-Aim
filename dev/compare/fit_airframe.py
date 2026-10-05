"""Fit the whole-aircraft (flight path and energy) model from telemetry recordings.

    python dev/compare/fit_airframe.py recordings/flight-*.jsonl

Prints the parameters the comparison simulator uses (dev/compare/airframe.h):
  - flight path lag: the velocity turns toward the nose with time constant tau
    (angle of attack ~ tau * pitch rate), plus a steady offset;
  - energy: dV/dt + g sin(climb) = thrust(throttle, brake) - drag * V^2 - turn drag * (nz-1)^2;
  - pitch authority vs speed: the best pitch rate full stick reaches at each speed.
Least squares in pure Python (no numpy needed).
"""
import json, math, sys

def load(path):
    frames = []
    for line in open(path, encoding="utf-8"):
        if not line.strip(): continue
        d = json.loads(line)
        if d.get("type") or d.get("gt", -1) < 0: continue   # contacts / boss packets
        frames.append(d)
    return frames

def solve(rows, ys):
    """Ordinary least squares via normal equations (small systems)."""
    n = len(rows[0])
    a = [[0.0] * n for _ in range(n)]; b = [0.0] * n
    for r, y in zip(rows, ys):
        for i in range(n):
            b[i] += r[i] * y
            for j in range(n): a[i][j] += r[i] * r[j]
    for i in range(n):                      # Gauss-Jordan
        p = max(range(i, n), key=lambda k: abs(a[k][i])); a[i], a[p] = a[p], a[i]; b[i], b[p] = b[p], b[i]
        for k in range(n):
            if k != i and a[i][i]:
                f = a[k][i] / a[i][i]
                for j in range(n): a[k][j] -= f * a[i][j]
                b[k] -= f * b[i]
    return [b[i] / a[i][i] for i in range(n)]

def stick(f):
    return f["stick"] if f.get("ctl", 1) == 1 and not f.get("kb") else f.get("raw", f["stick"])

frames = []
for path in sys.argv[1:]:
    fr = load(path)
    g0 = fr[0]["gt"] if fr else 0
    frames += [f for f in fr if f["gt"] - g0 > 3]   # skip the spawn transient

# 1. flight path lag: AoA (deg) = a0 + tau * pitch rate (deg/s); sideslip likewise with yaw rate
rows, ys, rows_b, ys_b = [], [], [], []
for f in frames:
    if f["d"][12] < 150: continue
    rows.append([1.0, f["rate"][0]]); ys.append(f["d"][13])
    rows_b.append([1.0, f["rate"][1]]); ys_b.append(f["d"][14])
a0, tau = solve(rows, ys)
b0, tau_b = solve(rows_b, ys_b)
res = [y - (a0 + tau * r[1]) for r, y in zip(rows, ys)]
print(f"AoA = {a0:.2f} deg + {tau:.3f} s * pitch rate   (rms residual {math.sqrt(sum(x*x for x in res)/len(res)):.2f} deg, n={len(rows)})")
print(f"sideslip = {b0:.2f} deg + {tau_b:.3f} s * yaw rate")

# 2. energy: dV/dt + g*sin(climb) = c0 + c1*throttle + c2*brake + c3*V^2 + c4*(nz-1)^2 + c5*highG
# Differences over ~0.5 s windows (per-frame speed differences are noisy), averaging the
# regressors over the window; windows with a jump (respawn, teleport) are dropped.
rows, ys = [], []
j = 0
for i, a in enumerate(frames):
    while j < len(frames) and frames[j]["gt"] < a["gt"] + 0.5: j += 1
    if j >= len(frames): break
    b = frames[j]; dt = b["gt"] - a["gt"]
    if not (0.45 < dt < 0.6) or j - i < 10: continue
    win = frames[i:j]
    if min(f["d"][12] for f in win) < 150: continue
    if any(abs(p["d"][12] - q["d"][12]) > 15 for p, q in zip(win, win[1:])): continue
    n = len(win)
    thr = sum(f["in"][0] for f in win) / n; brk = sum(f["in"][1] for f in win) / n
    hg = sum(1.0 for f in win if f["in"][0] > .5 and f["in"][1] > .5) / n
    climb = sum(f["vel"][2] / max(math.sqrt(sum(x * x for x in f["vel"])), 1) for f in win) / n
    v2 = sum((f["d"][12] / 100) ** 2 for f in win) / n
    nz2 = sum((f["d"][15] - 1) ** 2 for f in win) / n / 100
    rows.append([1.0, max(thr - hg, 0), max(brk - hg, 0), hg, v2, nz2])
    ys.append((b["d"][12] - a["d"][12]) / dt + 9.81 * climb)
rows, ys = rows[::5], ys[::5]   # overlapping windows: thin them
c = solve(rows, ys)
res = [y - sum(ci * xi for ci, xi in zip(c, r)) for r, y in zip(rows, ys)]
cut = sorted(abs(x) for x in res)[int(len(res) * .95)]
keep = [k for k, x in enumerate(res) if abs(x) <= cut]
rows, ys = [rows[k] for k in keep], [ys[k] for k in keep]
c = solve(rows, ys)
names = ["base", "throttle", "brake", "high-G", "V^2 (per (100 m/s)^2)", "(nz-1)^2 (per 100 g^2)"]
print("dV/dt + g sin(climb) =")
for n_, v in zip(names, c): print(f"    {v:9.3f}  {n_}")
res = [y - sum(ci * xi for ci, xi in zip(c, r)) for r, y in zip(rows, ys)]
print(f"  rms residual {math.sqrt(sum(x*x for x in res)/len(res)):.1f} m/s^2, n={len(rows)}")
# cruise speeds implied (level, 1 g)
for label, thr, brk, hg in (("idle", 0, 0, 0), ("throttle", 1, 0, 0), ("brake", 0, 1, 0)):
    accel = c[0] + c[1] * thr + c[2] * brk + c[3] * hg
    v2 = -accel / c[4] if c[4] < 0 and accel > 0 else float("nan")
    print(f"  level 1 g equilibrium speed ({label}): {100 * math.sqrt(v2) if v2 == v2 else float('nan'):.0f} m/s")

# 3. pitch authority vs speed: best sustained pitch rate under full pull, by speed band
bands = {}
for f in frames:
    s = stick(f)
    if s[0] < 0.9: continue
    hg = f["in"][0] > .5 and f["in"][1] > .5
    band = int(f["d"][12] // 100) * 100
    bands.setdefault((band, hg), []).append(f["rate"][0])
print("full-pull pitch rate by speed (p90 deg/s):")
for (band, hg), qs in sorted(bands.items()):
    if len(qs) < 30: continue
    qs.sort()
    print(f"  {band:4d}-{band + 99:4d} m/s {'high-G' if hg else 'normal':7s}: p50 {qs[len(qs)//2]:5.1f}  p90 {qs[int(len(qs)*.9)]:5.1f}  (n={len(qs)})")
