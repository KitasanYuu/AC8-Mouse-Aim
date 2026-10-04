"""Cut whole air battles (minutes, every enemy aircraft) out of telemetry recordings for
the comparison bench's continuous-combat scenarios.

    python dev/compare/extract_battles.py recordings/flight-*.jsonl [--max 6]
        [--window 20261005-010417:500-690]   a battle by hand (seconds into the recording)

Without --window, battles are found automatically: stretches with an enemy aircraft
within 3 km of the player (gaps under 15 s joined), 60-200 s long; those with elites
(Shadow squadrons, named aces) first, then the busiest. Each becomes
dev/compare/battles/<recording>_<start>.txt:
  player  the player's state at the start
  enemy   one line per enemy aircraft: index, class, first and last time alive, whether it
          was shot down, whether it is an elite, and whether the player shot it down
          (selected within 10 s before it went down; missiles count too)
  e       that enemy's recorded samples (~10 Hz), relative times
  own     the player's own flight as recorded, with the selected enemy's index
Enemies fly their recorded paths: they do not react to the simulated aircraft.
"""
import argparse, bisect, json, math, os, re

ap = argparse.ArgumentParser()
ap.add_argument("files", nargs="+")
ap.add_argument("--max", type=int, default=6)
ap.add_argument("--window", action="append", default=[])
ap.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "battles"))
args = ap.parse_args()
os.makedirs(args.out, exist_ok=True)
elite = lambda cls: bool(re.search("Shadow|Named", cls))

battles = []
for path in args.files:
    frames, snaps = [], []
    for line in open(path, encoding="utf-8"):
        if not line.strip(): continue
        d = json.loads(line)
        if d.get("type") == "contacts": snaps.append(d)
        elif d.get("gt", -1) >= 0: frames.append(d)
    if not frames or not snaps: continue
    name = os.path.splitext(os.path.basename(path))[0].replace("flight-", "")
    g0 = frames[0]["gt"]
    hostile = {c[1] for s in snaps for c in s["c"] if len(c) > 10 and c[10] == 1}
    gts = [f["gt"] for f in frames]
    def frame_at(t):
        return frames[min(bisect.bisect_left(gts, t), len(frames) - 1)]
    def flying(c): return not c[9] and abs(c[2]) + abs(c[3]) + abs(c[4]) > 1

    windows = []
    for w in args.window:
        rec, _, span = w.partition(":")
        if rec == name:
            a, b = (float(x) for x in span.split("-"))
            windows.append((g0 + a, g0 + b, True))
    if not windows:
        # enemy within 3 km of the player, per snapshot
        busy = []
        for s in snaps:
            f = frame_at(s["gt"])
            near = [c for c in s["c"] if c[1] in hostile and flying(c) and math.dist(c[2:5], f["pos"]) < 3000]
            if near and f.get("ctl", 1) == 1: busy.append(s["gt"])
        runs, start, last = [], None, None
        for t in busy:
            if start is None: start = last = t
            elif t - last > 15: runs.append((start, last)); start = last = t
            else: last = t
        if start is not None: runs.append((start, last))
        for a, b in runs:
            while b - a >= 60:
                windows.append((max(a - 5, g0), min(a + 200, b + 5), False))
                a += 200

    for t0, t1, forced in windows:
        enemies = {}
        for s in snaps:
            if not t0 <= s["gt"] <= t1: continue
            f = frame_at(s["gt"])
            for c in s["c"]:
                if c[1] not in hostile or not flying(c): continue
                e = enemies.setdefault(c[0], {"cls": c[1], "samples": []})
                e["samples"].append((s["gt"] - t0, c[2], c[3], c[4], c[5], c[6], c[7]))
                e["last_range"] = math.dist(c[2:5], f["pos"])
            for gone in s.get("gone", []):
                if gone in enemies: enemies[gone]["gone"] = True
        enemies = {k: v for k, v in enemies.items() if len(v["samples"]) >= 5}
        if not enemies: continue
        ids = list(enemies)
        span = t1 - t0
        # When the player had each enemy selected (relative to the window).
        sel_times_of = {}
        for s in snaps:
            if t0 <= s["gt"] <= t1 and s.get("selected") in enemies:
                sel_times_of.setdefault(s["selected"], []).append(s["gt"] - t0)
        for eid, e in enemies.items():
            smp = e["samples"]
            # A shot-down aircraft keeps its actor for a while: the wreck flies on for a few
            # seconds, then stops dead for ~12 s before it is removed (logged). The frozen
            # tail is dropped; the game deselects it as it goes down.
            moving = len(smp) - 1
            while moving > 0 and math.dist(smp[moving][1:4], smp[moving - 1][1:4]) < 0.5: moving -= 1
            frozen = moving < len(smp) - 5
            del smp[moving + 1:]
            ended = smp[-1][0] < span - 1
            e["down"] = e.get("gone", False) or frozen or (ended and e["last_range"] < 3000)
            sel = [x for x in sel_times_of.get(eid, []) if x <= smp[-1][0] + 0.5]
            e["by_player"] = bool(e["down"] and sel and smp[-1][0] - sel[-1] < 10)
            if e["by_player"] and sel[-1] < smp[-1][0]:
                # deselected as it went down: that is the kill
                e["samples"] = [x for x in smp if x[0] <= sel[-1] + 0.2] or smp[:1]
        own = []
        for f in frames:
            if not t0 <= f["gt"] <= t1: continue
            own.append(f)
        sel_at = {}
        for s in snaps:
            if t0 <= s["gt"] <= t1: sel_at[round(s["gt"], 1)] = s.get("selected")
        sel_times = sorted(sel_at)
        def selected(t):
            i = bisect.bisect_right(sel_times, round(t, 1)) - 1
            sid = sel_at[sel_times[i]] if i >= 0 else None
            return ids.index(sid) if sid in ids else -1
        f0 = own[0]
        n_elite = sum(elite(e["cls"]) for e in enemies.values())
        battles.append({"name": f"{name}_{t0 - g0:.0f}", "file": name, "t0": t0 - g0, "t1": t1 - g0, "forced": forced,
                        "elite": n_elite, "enemies": [enemies[i] for i in ids], "ids": ids,
                        "player": (*f0["pos"], *f0["att"], f0["d"][12]),
                        "own": [(f["gt"] - t0, *f["pos"], *f["att"], *f["aim"][:2], f["d"][12], *f["rate"], *f["stick"],
                                 selected(f["gt"]), f.get("ctl", 1) if not f.get("manual") else 0) for f in own],
                        "busy": sum(len(e["samples"]) for e in enemies.values())})

battles.sort(key=lambda b: (not b["forced"], -b["elite"], -b["busy"]))
chosen = battles[:max(args.max, sum(b["forced"] for b in battles))]
for old in os.listdir(args.out):
    if old.endswith(".txt"): os.remove(os.path.join(args.out, old))
for b in chosen:
    with open(os.path.join(args.out, b["name"] + ".txt"), "w", encoding="utf-8", newline="\n") as out:
        classes = {}
        for e in b["enemies"]: classes[e["cls"]] = classes.get(e["cls"], 0) + 1
        out.write(f"# battle: flight-{b['file']} {b['t0']:.0f}-{b['t1']:.0f} s into the recording, {len(b['enemies'])} enemy aircraft "
                  f"({b['elite']} elite): " + ", ".join(f"{k} x{v}" for k, v in classes.items()) + "\n")
        out.write("# player x y z pitch yaw roll speed (start)\n")
        out.write("player " + " ".join(f"{v:.3f}" for v in b["player"]) + "\n")
        out.write("# enemy index class first_t last_t shot_down elite by_player / e index t x y z pitch yaw roll\n")
        for i, e in enumerate(b["enemies"]):
            out.write(f"enemy {i} {e['cls']} {e['samples'][0][0]:.2f} {e['samples'][-1][0]:.2f} {int(e['down'])} {int(elite(e['cls']))} {int(e['by_player'])}\n")
            for smp in e["samples"]: out.write(f"e {i} " + " ".join(f"{v:.2f}" for v in smp) + "\n")
        out.write("# own t x y z pitch yaw roll aim_pitch aim_yaw speed q r p stick_pitch stick_roll stick_yaw selected_enemy mod_flying\n")
        next_t = 0.0
        for o in b["own"]:   # 20 Hz
            if o[0] >= next_t:
                next_t = o[0] + 0.05
                out.write("own " + " ".join(f"{v:.2f}" for v in o) + "\n")
    print(f"{b['name']}: {b['t1'] - b['t0']:.0f} s, {len(b['enemies'])} enemies ({b['elite']} elite), "
          f"{sum(e['down'] for e in b['enemies'])} shot down, {sum(e['by_player'] for e in b['enemies'])} by the player")
print(f"{len(battles)} battles found, {len(chosen)} written to {args.out}")
