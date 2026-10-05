"""Cut gun-attack segments out of telemetry recordings for the comparison bench.

    python dev/compare/extract_tracks.py recordings/flight-*.jsonl [--max 8] [--seconds 14]
        [--at 20261005-005521:212]   always include the attack starting at that recording
                                     time (seconds from the recording's start, as the
                                     telemetry page's clock shows it)

A segment: the same hostile aircraft selected and within 1.8 km for the whole window,
the mod flying (not manual). Each becomes dev/compare/tracks/<recording>_<time>.txt:
the player's starting state, the enemy's recorded path, and the player's own recorded
path (what the controller in game actually did), all relative to the segment start.
The bench (harness.cpp) flies every controller from that state after that enemy.
"""
import argparse, bisect, json, math, os

ap = argparse.ArgumentParser()
ap.add_argument("files", nargs="+")
ap.add_argument("--max", type=int, default=8)
ap.add_argument("--seconds", type=float, default=14.0)
ap.add_argument("--range", type=float, default=1800.0)
ap.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "tracks"))
ap.add_argument("--at", action="append", default=[])
args = ap.parse_args()
os.makedirs(args.out, exist_ok=True)

def heading_change(path):
    """Mean turn rate (deg/s) of a list of (t, x, y, z)."""
    turns, total = 0.0, 0.0
    for a, b, c in zip(path, path[1:], path[2:]):
        v1 = [b[k] - a[k] for k in (1, 2, 3)]; v2 = [c[k] - b[k] for k in (1, 2, 3)]
        n1 = math.sqrt(sum(x * x for x in v1)); n2 = math.sqrt(sum(x * x for x in v2))
        if n1 < 1e-3 or n2 < 1e-3: continue
        cosv = max(-1, min(1, sum(x * y for x, y in zip(v1, v2)) / n1 / n2))
        turns += math.degrees(math.acos(cosv)); total += c[0] - b[0]
    return turns / total if total else 0

candidates = []
for path in args.files:
    frames, snaps = [], []
    for line in open(path, encoding="utf-8"):
        if not line.strip(): continue
        d = json.loads(line)
        if d.get("type") == "contacts": snaps.append(d)
        elif d.get("type") == "contacts_more" and snaps and snaps[-1]["gt"] == d["gt"]: snaps[-1]["c"] += d["c"]   # the rest of a big sample
        elif not d.get("type") and d.get("gt", -1) >= 0: frames.append(d)   # not the boss packets
    if not frames or not snaps: continue
    # enemies: classes the game lists as lock-on candidates (c[11] >= 1, since 2026-10-05: allies
    # cannot be locked), or that the player selected (c[10], older recordings)
    hostile = {c[1] for s in snaps for c in s["c"] if (len(c) > 11 and c[11] >= 1) or (len(c) > 10 and c[10] == 1)}
    gts = [f["gt"] for f in frames]
    def frame_at(t):
        i = bisect.bisect_left(gts, t)
        return frames[min(i, len(frames) - 1)]
    name = os.path.splitext(os.path.basename(path))[0].replace("flight-", "")
    g0 = frames[0]["gt"]
    # requested attacks: snapshot index -> True (checks relaxed: same target selected only)
    forced = {}
    for at in args.at:
        rec, _, when = at.partition(":")
        if rec == name:
            start = g0 + float(when)
            forced[next(i for i, s in enumerate(snaps) if s["gt"] >= start)] = True
    order = sorted(forced) + [i for i in range(len(snaps)) if i not in forced]
    used = []   # (start, end) game times already cut
    for k in order:
        s0 = snaps[k]; sel = s0.get("selected")
        strict = k not in forced
        if strict and any(a - 0.5 <= s0["gt"] <= b for a, b in used): continue
        def contact(s):
            for c in s["c"]:
                if c[0] == sel and not c[9]: return c
            return None
        c0 = contact(s0)
        if not sel or c0 is None or (strict and c0[1] not in hostile):
            continue
        f0 = frame_at(s0["gt"])
        ok, enemy, j = True, [], k
        while j < len(snaps) and snaps[j]["gt"] - s0["gt"] <= args.seconds:
            s = snaps[j]; c = contact(s); f = frame_at(s["gt"])
            if s.get("selected") != sel or c is None or abs(f["gt"] - s["gt"]) > 0.1:
                ok = False; break
            rng = math.dist(c[2:5], f["pos"])
            if strict and (rng > args.range or f.get("ctl", 1) != 1 or f.get("manual")):
                ok = False; break
            enemy.append((s["gt"] - s0["gt"], c[2], c[3], c[4], c[5], c[6], c[7]))
            j += 1
        long_enough = enemy and enemy[-1][0] >= (args.seconds - 0.2 if strict else 6)
        if (ok or not strict) and long_enough:
            t_end = s0["gt"] + enemy[-1][0]
            if strict and any(a < t_end and s0["gt"] < b for a, b in used): continue
            used.append((s0["gt"], t_end))
            own = [(f["gt"] - s0["gt"], *f["pos"], *f["att"], *f["aim"][:2], f["d"][12], *f["rate"], *f["stick"]) for f in frames
                   if s0["gt"] <= f["gt"] <= t_end]
            candidates.append({"name": f"{name}_{s0['gt'] - g0:.0f}", "file": name, "gt": s0["gt"] - g0, "cls": c0[1], "forced": not strict,
                               "player": (*f0["pos"], *f0["att"], f0["d"][12]), "enemy": enemy, "own": own,
                               "turn": heading_change(enemy), "range": math.dist(c0[2:5], f0["pos"])})

# Requested attacks, then the hardest: enemies that turn the most.
candidates.sort(key=lambda c: (not c["forced"], -c["turn"]))
chosen = candidates[:max(args.max, sum(c["forced"] for c in candidates))]
# Files already there are kept (dev/compare/scenes.txt picks the ones the bench uses).
for c in chosen:
    with open(os.path.join(args.out, c["name"] + ".txt"), "w", encoding="utf-8", newline="\n") as out:
        out.write(f"# recorded gun attack: flight-{c['file']} at {c['gt']:.1f} s into the recording, enemy {c['cls']}, "
                  f"start range {c['range']:.0f} m, enemy turns {c['turn']:.0f} deg/s on average\n")
        out.write("# player x y z pitch yaw roll speed (start)\n")
        out.write("player " + " ".join(f"{v:.3f}" for v in c["player"]) + "\n")
        out.write("# enemy t x y z pitch yaw roll (~10 Hz)\n")
        for e in c["enemy"]: out.write("enemy " + " ".join(f"{v:.3f}" for v in e) + "\n")
        out.write("# own t x y z pitch yaw roll aim_pitch aim_yaw speed q r p stick_pitch stick_roll stick_yaw (as flown in game)\n")
        for i, o in enumerate(c["own"]):
            out.write("own " + " ".join(f"{v:.3f}" for v in o) + "\n")
    print(f"{c['name']}: {c['cls']}, range {c['range']:.0f} m, enemy turn {c['turn']:.0f} deg/s")
print(f"{len(candidates)} segments found, {len(chosen)} written to {args.out}")
