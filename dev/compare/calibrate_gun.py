"""Calibrate the bench's gun hits on the player's own gun kills.

    python dev/compare/calibrate_gun.py recordings/flight-*.jsonl

Every enemy the player brought down with the gun alone (no missile pressed in the 8 s before;
gun held with the nose within 15 deg of it and it within 2 km) is replayed through the bench's
round model (harness.cpp, Pilot): rounds every frame the gun is held, from the player's recorded
position at the aircraft's velocity plus 1389 m/s along the nose, falling at 9.8 m/s^2, 1.08 s
of life, counted by the closest they come to the enemy's recorded path, the share of a spread
(0.35 deg) within the enemy's size plus the burst. Damage: 100 a second of hits, 150% on
bombers and 85% on fighters, as the bench. Brought down when the game deselected it (it does so
as it goes down) or its wreck froze, whichever first.

If the model is right, the damage it counts by the moment of the kill is the enemy's health. It
prints, for a range of extra radius on the bench's (fighters 6 m, bombers 20 m, plus 5 m burst),
the median of counted damage / health over the kills: the radius where that is 1 fits the game.
"""
import bisect, collections, glob, json, math, re, sys

# --loose: no selection required (the player's selection judged from the gun on it alone)
LOOSE = "--loose" in sys.argv
files = [f for f in sys.argv[1:] if not f.startswith("--")] or sorted(glob.glob("recordings/flight-*.jsonl"))
MUZZLE, LIFE, FALL, SPREAD = 1389.0, 1.08, 9.8, math.radians(0.35)
HEALTH = [("tu95", 150), ("t160", 125), ("b01b", 130), ("su57", 115), ("f22a", 110), ("f35c", 70), ("su35", 65),
          ("typn", 60), ("m29a", 60), ("f04e", 60), ("su25", 60), ("mr2k", 55), ("f16c", 55), ("a06e", 55), ("m21b", 46), ("mi24", 35)]
def health(cls): return next((h for k, h in HEALTH if k in cls), 60)
def large(cls): return any(k in cls for k in ("tu95", "t160", "b01b"))
def nose(att):
    p, y = math.radians(att[0]), math.radians(att[1])
    return (math.cos(p) * math.cos(y), math.cos(p) * math.sin(y), math.sin(p))

kills = []
for path in files:
    frames, snaps = [], []
    for line in open(path, encoding="utf-8"):
        if not line.strip(): continue
        try: d = json.loads(line)
        except ValueError: continue
        if d.get("type") == "contacts": snaps.append(d)
        elif d.get("type") == "contacts_more" and snaps and snaps[-1]["gt"] == d["gt"]: snaps[-1]["c"] += d["c"]
        elif not d.get("type") and d.get("gt", -1) >= 0: frames.append(d)
    if not frames or "mb" not in frames[-1] or not snaps: continue
    gts = [f["gt"] for f in frames]
    tracks = collections.defaultdict(list); gone = {}; sel_at = []   # (time, where the selected object was)
    for s in snaps:
        for g in s.get("gone", []): gone.setdefault(g, s["gt"])
        sc = next((c for c in s["c"] if c[0] == s.get("selected")), None)
        if sc: sel_at.append((s["gt"], sc[2:5]))
        for c in s["c"]:
            if len(c) > 10 and c[10] == 1 and not c[9]: tracks[(c[0], c[1])].append((s["gt"], c[2], c[3], c[4]))
    for (addr, cls), smp in tracks.items():
        if re.search(r"ladn|uav|Tewy|boss|ChildWing|_gsh|LaserPod|_RG|Ally", cls, re.I) or len(smp) < 10: continue
        m = len(smp) - 1
        while m > 0 and math.dist(smp[m][1:4], smp[m - 1][1:4]) < 0.5: m -= 1
        frozen = m < len(smp) - 5
        if not (frozen or addr in gone): continue
        t_end = smp[m][0]
        # the player's own: what it had selected was this enemy or a part of it (within 100 m: a
        # Tu-95's wings are locked on, not its body), until within 10 s of the wreck freezing
        ts0 = [x[0] for x in smp]
        def where(t):
            k = min(max(bisect.bisect_left(ts0, t), 0), len(smp) - 1); return smp[k][1:4]
        chosen = [t for t, p in sel_at if smp[0][0] <= t <= t_end + 0.5 and math.dist(p, where(t)) < 100]
        if (not chosen or t_end - chosen[-1] > 10) and not LOOSE: continue
        t_kill = min(t_end, chosen[-1] + 0.2) if chosen and t_end - chosen[-1] <= 10 else t_end
        i0, i1 = bisect.bisect_left(gts, t_kill - 15), bisect.bisect_left(gts, t_kill + 0.05)
        if any(frames[k].get("mb", 0) & 2 for k in range(bisect.bisect_left(gts, t_kill - 8), i1)): continue
        ts = [x[0] for x in smp]
        def at(t):
            k = min(max(bisect.bisect_left(ts, t), 1), len(smp) - 1); a, b = smp[k - 1], smp[k]
            u = max(0.0, min(1.0, (t - a[0]) / max(b[0] - a[0], 1e-6)))
            return [a[j] + (b[j] - a[j]) * u for j in (1, 2, 3)]
        rounds = []   # (t0, dt, p0, v, spread)
        on = 0.0
        for k in range(i0, i1 - 1):
            f = frames[k]
            if not f.get("mb", 0) & 1: continue
            e = at(f["gt"]); to = [e[j] - f["pos"][j] for j in range(3)]; r = math.sqrt(sum(x * x for x in to))
            n = nose(f["att"])
            off = math.degrees(math.acos(max(-1, min(1, sum(n[j] * to[j] for j in range(3)) / max(r, 1)))))
            if off > 15 or r > 2000: continue
            dt = frames[k + 1]["gt"] - f["gt"]
            on += dt
            rounds.append((f["gt"], dt, f["pos"], [f["vel"][j] + n[j] * MUZZLE for j in range(3)], r * math.tan(SPREAD), r))
        if on < 0.1: continue
        # each round's closest approach, arriving before the kill
        best = []
        for t0, dt, p0, v, spread, r0 in rounds:
            d_min, t = 1e9, t0
            while t - t0 <= LIFE and t <= t_kill:
                age = t - t0
                pos = [p0[0] + v[0] * age, p0[1] + v[1] * age, p0[2] + v[2] * age - 0.5 * FALL * age * age]
                d = math.dist(pos, at(t))
                if d > d_min: break
                d_min = d; t += 0.01
            if d_min < 1e8: best.append((dt, d_min, spread, r0))
        kills.append({"cls": cls, "on": on, "rounds": best, "file": path[-22:-6], "t": t_kill - frames[0]["gt"]})

print(f"{len(kills)} gun kills: " + ", ".join(f"{k}: {v}" for k, v in collections.Counter(
    "bomber" if large(k["cls"]) else "fighter" for k in kills).items()))
def counted(k, extra, cone=0.0):
    # extra: metres on the radius; cone: degrees, a radius growing with the range (an aim assist)
    rate = 100 * (1.5 if large(k["cls"]) else 0.85)
    total = 0.0
    for dt, d, s, r0 in k["rounds"]:
        R = (20 if large(k["cls"]) else 6) + 5 + extra + r0 * math.tan(math.radians(cone))
        total += dt * (R * R / (R * R + 2 * s * s)) * math.exp(-d * d / (R * R + 2 * s * s))
    return total * rate / health(k["cls"])
for kind in ("fighter", "bomber"):
    ks = [k for k in kills if large(k["cls"]) == (kind == "bomber")]
    if not ks: continue
    on = sorted(k["on"] for k in ks); dist = sorted(d for k in ks for _, d, _, _ in k["rounds"])
    ang = sorted(math.degrees(math.atan2(d, r0)) for k in ks for _, d, _, r0 in k["rounds"])
    print(f"{kind}: {len(ks)} kills, gun held on it median {on[len(on)//2]:.2f} s; rounds' closest approach median {dist[len(dist)//2]:.0f} m, quartiles {dist[len(dist)//4]:.0f}-{dist[3*len(dist)//4]:.0f} m; as an angle at the range fired from median {ang[len(ang)//2]:.1f} deg, quartiles {ang[len(ang)//4]:.1f}-{ang[3*len(ang)//4]:.1f}")
    for cone in (1, 2, 3, 4, 5, 6, 8):
        r = sorted(counted(k, 0, cone) for k in ks)
        print(f"   cone {cone} deg: counted damage / health, median {r[len(r)//2]:.2f} (quartiles {r[len(r)//4]:.2f}-{r[3*len(r)//4]:.2f})")
    for extra in (0, 5, 10, 15, 20, 30, 40, 60):
        r = sorted(counted(k, extra) for k in ks)
        print(f"   extra radius {extra:3d} m: counted damage / health, median {r[len(r)//2]:.2f} (quartiles {r[len(r)//4]:.2f}-{r[3*len(r)//4]:.2f})")
