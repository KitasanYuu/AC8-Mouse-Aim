"""Fit an aircraft's by-speed tables for the comparison bench from telemetry recordings.

    python dev/compare/fit_speed_tables.py [--aircraft adfx02] recordings/flight-*.jsonl

Per 100 m/s band (0..800): the full pull rate (p90 of the nose's pitch rate with the stick
fully back, throttle and brake not both held), the same in a high-G turn (both held), the full
roll rate (p90 with the roll stick fully over), and the full-throttle acceleration
dV/dt + g sin(climb) (median, stick near neutral, no brake). Rates come from attitude
differences (the "rate" field's axes are not reliable). Only frames flown in the named
aircraft (the "self" of the contacts packets) are used when --aircraft is given.
"""
import argparse, json, math

ap = argparse.ArgumentParser()
ap.add_argument("files", nargs="+")
ap.add_argument("--aircraft", default="")
args = ap.parse_args()
rad = math.pi / 180

def basis(p, y, r):
    p, y, r = p * rad, y * rad, r * rad
    f = (math.cos(p) * math.cos(y), math.cos(p) * math.sin(y), math.sin(p))
    right = (-math.sin(y), math.cos(y), 0.0)
    up = (f[1] * right[2] - f[2] * right[1], f[2] * right[0] - f[0] * right[2], f[0] * right[1] - f[1] * right[0])
    rr = tuple(right[i] * math.cos(r) - up[i] * math.sin(r) for i in range(3))
    uu = tuple(up[i] * math.cos(r) + right[i] * math.sin(r) for i in range(3))
    return f, rr, uu

dot = lambda a, b: a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
bands = lambda: [[] for _ in range(9)]
pull, pull_hg, roll, thrust = bands(), bands(), bands(), bands()
for path in args.files:
    frames, me = [], None
    for line in open(path, encoding="utf-8"):
        if not line.strip(): continue
        d = json.loads(line)
        if d.get("type") == "contacts": me = d.get("self", me); continue
        if d.get("type") or d.get("gt", -1) < 0: continue
        if args.aircraft and (not me or args.aircraft not in me): frames.append(None); continue
        frames.append(d)
    first = next((f["gt"] for f in frames if f), 0)
    for a, b in zip(frames, frames[1:]):
        if not a or not b or b["gt"] - first < 3: continue
        dt = b["gt"] - a["gt"]
        if not 0.005 < dt < 0.1 or abs(a["att"][0]) > 80 or a.get("ctl", 1) != 1: continue
        v = a["d"][12]
        if v < 50: continue
        k = min(int(v / 100), 8)
        fa, ra, ua = basis(*a["att"]); fb, rb, ub = basis(*b["att"])
        q = dot([(fb[i] - fa[i]) / dt for i in range(3)], ua) / rad      # nose toward the canopy
        p = dot([(rb[i] - ra[i]) / dt for i in range(3)], ua) / rad      # right wing toward the canopy
        st, th, br = a["stick"], a["in"][0], a["in"][1]
        if st[0] > 0.95:
            (pull_hg if th > 0.5 and br > 0.5 else pull)[k].append(q)
        if abs(st[1]) > 0.95: roll[k].append(abs(p))
    # acceleration over 0.5 s at full throttle, little stick, no brake
    good = [f for f in frames if f and f["gt"] - first > 3 and f.get("ctl", 1) == 1]
    j = 0
    for i, a in enumerate(good):
        while j < len(good) and good[j]["gt"] < a["gt"] + 0.5: j += 1
        if j >= len(good): break
        b = good[j]; dt = b["gt"] - a["gt"]
        if dt > 0.7: continue
        seg = good[i:j + 1]
        if any(f["in"][0] < 0.95 or f["in"][1] > 0.05 or abs(f["stick"][0]) > 0.2 for f in seg): continue
        v0, v1 = a["d"][12], b["d"][12]
        if v0 < 50: continue
        vel = a["vel"]; climb = math.asin(max(-1, min(1, vel[2] / max(math.hypot(*vel), 1))))
        thrust[min(int((v0 + v1) / 200), 8)].append((v1 - v0) / dt + 9.81 * math.sin(climb))

def q(v, p): return sorted(v)[int(len(v) * p)] if len(v) >= 20 else float("nan")
print("band(m/s)  pull p90   high-G p90   roll p90   thrust med   (samples pull/roll/thrust)")
for k in range(9):
    print(f"{k * 100:4d}-{k * 100 + 99:<4d} {q(pull[k], .9):8.1f} {q(pull_hg[k], .9):12.1f} {q(roll[k], .9):10.1f} {q(thrust[k], .5):12.1f}"
          f"   ({len(pull[k])}/{len(roll[k])}/{len(thrust[k])})")
