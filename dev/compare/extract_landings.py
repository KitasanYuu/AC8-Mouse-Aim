"""Cut landings out of a telemetry recording for the comparison bench.

    python dev/compare/extract_landings.py recordings/flight-*.jsonl [--before 25]

A landing: the aircraft coming down onto the ground (below 6 m) from flight, the mod flying.
Each becomes dev/compare/landings/<recording>_<touchdown time>.txt with
  - the runway, fitted from every ground roll in the recording (takeoffs and landings): its
    centre, heading, half length and half width, and the height of the aircraft's origin when
    on the ground;
  - the state `--before` seconds ahead of touchdown (or just after a respawn, if later);
  - the recorded touchdown (sink, bank, pitch, speed, heading), the standard a landing is held to;
  - the player's aim, throttle, brake, speed and position over the approach, as recorded.
The bench (harness.cpp) flies every controller from that state along the player's path.
"""
import argparse, json, math, os

ap = argparse.ArgumentParser()
ap.add_argument("files", nargs="+")
ap.add_argument("--before", type=float, default=25.0)
ap.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "landings"))
args = ap.parse_args()
os.makedirs(args.out, exist_ok=True)

for path in args.files:
    frames = []
    for line in open(path, encoding="utf-8"):
        if not line.strip(): continue
        d = json.loads(line)
        if d.get("type") or d.get("gt", -1) < 0: continue
        frames.append(d)
    if not frames: continue
    name = os.path.splitext(os.path.basename(path))[0].replace("flight-", "")
    g0 = frames[0]["gt"]
    def jump(a, b):   # a respawn or a move to the hangar: faster than anything flies (3 km/s)
        return math.dist(a["pos"], b["pos"]) > 3000 * max(b["gt"] - a["gt"], 0.05)
    # ground rolls: on the ground (origin below 6 m), moving, no jump
    rolls = [b for a, b in zip(frames, frames[1:]) if b["pos"][2] < 6 and 1 < b["d"][12] < 150 and not jump(a, b)]
    if len(rolls) < 50: print(f"{name}: no runway found"); continue
    # runway heading: the yaw of the rolls, folded to one direction (landings come either way)
    s = c = 0.0
    for f in rolls:
        a = math.radians(f["att"][1]) * 2; s += math.sin(a); c += math.cos(a)
    heading = math.degrees(math.atan2(s, c)) / 2
    ux, uy = math.cos(math.radians(heading)), math.sin(math.radians(heading))
    cx = sum(f["pos"][0] for f in rolls) / len(rolls); cy = sum(f["pos"][1] for f in rolls) / len(rolls)
    along = [(f["pos"][0] - cx) * ux + (f["pos"][1] - cy) * uy for f in rolls]
    side = [-(f["pos"][0] - cx) * uy + (f["pos"][1] - cy) * ux for f in rolls]
    lo, hi = min(along) - 300, max(along) + 300   # beyond the rolls seen, a little runway either end
    cx += ux * (lo + hi) / 2; cy += uy * (lo + hi) / 2
    half_length = (hi - lo) / 2
    half_width = max(30.0, max(abs(x) for x in side) + 10)
    ground = sorted(f["pos"][2] for f in rolls)[len(rolls) // 2]
    # touchdowns: the origin coming down through 1 m above its height on the ground, flying
    # (over 40 m/s), the mod flying. The game stops the aircraft on the ground in that frame:
    # the touchdown is the frame before it (sinking 14.5 m/s at 4.8 m, then still, logged)
    contact = ground + 1.0
    touch = []
    for i in range(1, len(frames)):
        a, b = frames[i - 1], frames[i]
        if b["pos"][2] <= contact < a["pos"][2] and a["d"][12] > 40 and not jump(a, b) and a.get("ctl", 1) == 1:
            if not touch or a["gt"] - touch[-1][0] > 20: touch.append((a["gt"], a))
    for td, ft in touch:
        t0 = td - args.before
        for a, b in zip(frames, frames[1:]):   # after the last respawn before it
            if t0 - 1.0 <= b["gt"] < td and jump(a, b): t0 = max(t0, b["gt"] + 1.0)   # (the first second after one reads wrong)
        seg = [f for f in frames if t0 <= f["gt"] <= td + 3]
        f0 = seg[0]
        out = os.path.join(args.out, f"{name}_{td - g0:.0f}.txt")
        with open(out, "w", encoding="utf-8", newline="\n") as o:
            o.write(f"# landing: flight-{name} {t0 - g0:.1f}-{td - g0:.1f} s into the recording, touchdown at {td - t0:.1f} s\n")
            o.write("# runway centre_x centre_y heading half_length half_width ground_z (the aircraft's origin on the ground; touchdown 1 m above)\n")
            o.write(f"runway {cx:.1f} {cy:.1f} {heading:.2f} {half_length:.0f} {half_width:.0f} {ground:.1f}\n")
            o.write("# start x y z pitch yaw roll speed\n")
            o.write("start " + " ".join(f"{v:.3f}" for v in (*f0["pos"], *f0["att"], f0["d"][12])) + "\n")
            o.write("# touch t sink bank pitch speed heading (as recorded)\n")
            o.write(f"touch {td - t0:.2f} {-ft['vel'][2]:.2f} {ft['att'][2]:.2f} {ft['att'][0]:.2f} {ft['d'][12]:.1f} {ft['att'][1]:.2f}\n")
            o.write("# r t aim_pitch aim_yaw throttle brake speed x y z (20 Hz)\n")
            next_t = t0
            for f in seg:
                if f["gt"] < next_t: continue
                next_t = f["gt"] + 0.05
                o.write(f"r {f['gt'] - t0:.2f} {f['aim'][0]:.2f} {f['aim'][1]:.2f} {f['in'][0]:.2f} {f['in'][1]:.2f} {f['d'][12]:.1f} "
                        f"{f['pos'][0]:.1f} {f['pos'][1]:.1f} {f['pos'][2]:.1f}\n")
        print(f"{os.path.basename(out)}: {td - t0:.1f} s, from {f0['pos'][2]:.0f} m at {f0['d'][12]:.0f} m/s; touchdown sink {-ft['vel'][2]:.1f} m/s, "
              f"bank {ft['att'][2]:.0f}, pitch {ft['att'][0]:.1f}; runway heading {heading:.1f}, {2 * half_length:.0f} x {2 * half_width:.0f} m")
