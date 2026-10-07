"""Step responses from a flight with the mouse flight off (F8) and full gamepad stick steps.

    python dev/compare/fit_response.py recordings/flight-<...>.jsonl

The game turns the aircraft at a limited angular acceleration, not by a first-order lag: on a
full stick the rate climbs nearly in a straight line for about a second (MiG-21bis pitch ~30
deg/s^2), and on release it climbs on a little more before it falls. So for each press (the
game's input from under 0.1 to over 0.9 within 0.15 s, held 0.5 s at least) and each release
that follows it:
  press    the lag (to 5% of the steady rate), the climb (deg/s^2, fitted between 20% and 80% of
           the steady rate) and the steady rate while held (its last 0.25 s);
  release  the further climb (s to the peak after release) and the fall (deg/s^2, between 80%
           and 20% of the peak, or the peak and where it stops), while the player's own stick
           (the gamepad, "pad") stays centred and the game's input with it: the game's autopilot
           (levelling, pressed by the player) moves the game's input with the stick centred, and
           the measure stops there.
The body rate comes from the attitude frame to frame (unfiltered; the logged rate is low-passed
at 12/s). Grouped by speed band; medians.
"""
import json, math, sys, collections

path = sys.argv[1]
rows = []
for line in open(path, encoding="utf-8"):
    if '"type"' in line[:12]: continue
    try: d = json.loads(line)
    except ValueError: continue
    if d.get("gt", -1) < 0 or d.get("ctl", 1) != 0: continue
    rows.append(d)
rad = math.pi / 180
def basis(p, y, r):
    p, y, r = p * rad, y * rad, r * rad
    f = (math.cos(p) * math.cos(y), math.cos(p) * math.sin(y), math.sin(p))
    r0 = (-math.sin(y), math.cos(y), 0.0)
    u0 = (-math.sin(p) * math.cos(y), -math.sin(p) * math.sin(y), math.cos(p))
    rr = tuple(r0[i] * math.cos(r) + u0[i] * math.sin(r) for i in range(3))
    uu = tuple(u0[i] * math.cos(r) - r0[i] * math.sin(r) for i in range(3))
    return f, rr, uu
def cross(a, b): return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
def dot(a, b): return sum(x * y for x, y in zip(a, b))
t, rate, U, V = [], {"pitch": [], "roll": []}, {"pitch": [], "roll": []}, []
PAD = {"pitch": (1, -1), "roll": (0, 1)}   # gamepad axis and sign of each game input (logged)
Pd = {"pitch": [], "roll": []}
prev = None
for d in rows:
    b = basis(*d["att"])
    if prev is not None:
        dt = d["gt"] - prev[0]
        if 0.001 < dt < 0.1:
            o = prev[1]
            w = [sum(x) * 0.5 / dt / rad for x in zip(cross(o[0], b[0]), cross(o[1], b[1]), cross(o[2], b[2]))]
            t.append(d["gt"]); rate["pitch"].append(-dot(w, b[1])); rate["roll"].append(-dot(w, b[0]))
            U["pitch"].append(d["raw"][0]); U["roll"].append(d["raw"][1]); V.append(math.sqrt(sum(x * x for x in d["vel"])))
            pad = d.get("pad", [0] * 6)
            for k, (j, sg) in PAD.items(): Pd[k].append(pad[j] * sg)
    prev = (d["gt"], b)
def smooth(x, n=2): return [sum(x[max(0, i - n):i + n + 1]) / len(x[max(0, i - n):i + n + 1]) for i in range(len(x))]
for k in rate: rate[k] = smooth(rate[k])
# the rate's sign against the input's, per axis (the attitude's conventions)
for k in rate:
    c = sum(r * u for r, u in zip(rate[k], U[k]))
    if c < 0: rate[k] = [-r for r in rate[k]]
def band(v): return "<250 m/s" if v < 250 else "250-400" if v < 400 else "400+"
def slope(ts, rs):
    n = len(ts)
    if n < 3: return None
    mt, mr = sum(ts) / n, sum(rs) / n
    den = sum((x - mt) ** 2 for x in ts)
    return sum((x - mt) * (y - mr) for x, y in zip(ts, rs)) / den if den > 0 else None
res = collections.defaultdict(list)
for axis in ("pitch", "roll"):
    r, u, pd = rate[axis], U[axis], Pd[axis]
    i = 1
    while i < len(t):
        if abs(u[i - 1]) < 0.1:
            j = i
            while j < len(t) and t[j] - t[i] < 0.15 and abs(u[j]) < 0.9: j += 1
            if j < len(t) and abs(u[j]) >= 0.9:
                sign = 1 if u[j] > 0 else -1
                h = j
                while h < len(t) and u[h] * sign >= 0.9: h += 1
                if t[h - 1] - t[i] >= 0.5:
                    s = [r[q] * sign for q in range(i, h)]
                    tail = [s[q] for q in range(len(s)) if t[h - 1] - t[i + q] < 0.25]
                    steady = sum(tail) / len(tail)
                    rec = {"v": V[i], "steady": steady}
                    lag = next((t[i + q] - t[i] for q in range(len(s)) if s[q] >= 0.05 * steady), None)
                    mid = [(t[i + q], s[q]) for q in range(len(s)) if 0.2 * steady <= s[q] <= 0.8 * steady]
                    rec["lag"] = lag
                    rec["climb"] = slope([a for a, _ in mid], [b for _, b in mid]) if steady > 5 else None
                    # the release: the player's stick back to centre within 0.2 s; then until the
                    # stick moves again, or the game's input does with it centred (the autopilot),
                    # the game's input allowed 0.1 s to follow the stick
                    if h < len(t):
                        m = h
                        while m < len(t) and t[m] - t[h] < 0.2 and abs(pd[m]) > 0.1: m += 1
                        if m < len(t) and abs(pd[m]) <= 0.1:
                            q = h; seg = []
                            while q < len(t) and t[q] - t[h] < 4 and (q <= m or abs(pd[q]) <= 0.15) and \
                                    (t[q] - t[m] < 0.1 or abs(u[q]) <= 0.15):
                                seg.append((t[q] - t[h], r[q] * sign)); q += 1
                            if len(seg) > 10:
                                pk = max(range(len(seg)), key=lambda z: seg[z][1])
                                peak = seg[pk][1]
                                rec["to_peak"] = seg[pk][0]
                                fall = [(a, b) for a, b in seg[pk:] if 0.2 * peak <= b <= 0.8 * peak]
                                if len(fall) < 4: fall = [(a, b) for a, b in seg[pk:] if b <= 0.9 * peak]   # cut short by the autopilot
                                f = slope([a for a, _ in fall], [b for _, b in fall]) if peak > 5 else None
                                rec["fall"] = -f if f is not None else None
                    res[(axis, band(V[i]))].append(rec)
                    i = h
                    continue
        i += 1
med = lambda xs: (sorted(xs)[len(xs) // 2] if xs else float("nan"))
print(f"{path}: {len(rows)} frames with the mouse flight off")
for (axis, b), recs in sorted(res.items()):
    g = lambda k: [x[k] for x in recs if x.get(k) is not None]
    print(f"{axis:5} {b:9} n={len(recs):2d}  steady {med(g('steady')):6.1f} deg/s | press: lag {med(g('lag')):.2f} s, climb {med(g('climb')):6.1f} deg/s^2"
          f" | release (n {len(g('fall'))}): on for {med(g('to_peak')):.2f} s, fall {med(g('fall')):6.1f} deg/s^2")
