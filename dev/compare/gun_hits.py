"""The player's gun as the game counts it, from recordings with hit events (contacts.lua "hit").

    python dev/compare/gun_hits.py recordings/flight-<...>.jsonl [...]

Every gun hit the game reports (LiveGameObject.OnHitByGun) carries the victim, the attacker and
the victim's health after it (LiveGameObject.HealthInternal: 10 x the table's health, a Tu-95
1500). For the player's own hits it prints, by range band (the player's position to the victim's
at the hit, from the 10 Hz contacts):
  - the time the gun was held (mouse button 1) with the selected target in the band, the hits
    in it and the hits a second;
  - the damage a hit did (the drop in health from the victim's previous hit or sample);
and the kills: a victim whose health ran out on the player's hit (the game's DESTROYED), apart
from those that went with no hit of the player's near (a falling bomber's wreck brings others
down: the player's account, 2026-10-06).
"""
import bisect, collections, json, math, sys

BANDS = [(0, 400), (400, 700), (700, 1000), (1000, 1300), (1300, 1700), (1700, 2200)]


def band(r):
    return next((f"{a}-{b}" for a, b in BANDS if a <= r < b), None)


for path in sys.argv[1:]:
    frames, snaps, hits = [], [], []
    for line in open(path, encoding="utf-8"):
        if not line.strip(): continue
        try: d = json.loads(line)
        except ValueError: continue
        t = d.get("type")
        if t == "contacts": snaps.append(d)
        elif t == "contacts_more" and snaps and snaps[-1]["gt"] == d["gt"]: snaps[-1]["c"] += d["c"]
        elif t == "hit": hits.append(d)
        elif not t and d.get("gt", -1) >= 0: frames.append(d)
    me = snaps[0]["self"] if snaps else "?"
    gts = [f["gt"] for f in frames]
    # where each object was (and its health) at each sample
    track = collections.defaultdict(list)   # id -> [(t, (x, y, z), health)]
    for s in snaps:
        for c in s["c"]:
            track[c[0]].append((s["gt"], tuple(c[2:5]), c[12] if len(c) > 12 else -1))
    def at(i, t):
        tr = track.get(i)
        if not tr: return None
        k = bisect.bisect_left([x[0] for x in tr], t)
        return tr[min(k, len(tr) - 1)]
    def my_pos(t):
        k = min(bisect.bisect_left(gts, t), len(frames) - 1)
        return frames[k]["pos"]
    mine = [h for h in hits if h["ac"] == me]
    print(f"{path}: {len(hits)} hits reported, {len(mine)} by the player ({me})")
    # damage per hit: the drop from the victim's previous known health
    last_h = {}
    per_band = collections.defaultdict(lambda: {"hits": 0, "dmg": []})
    kills = []
    for h in sorted(hits, key=lambda x: x["gt"]):
        v = h["v"]
        prev = last_h.get(v)
        if prev is None:
            s = at(v, h["gt"] - 0.05)
            prev = s[2] if s and s[2] >= 0 else None
        last_h[v] = h["h"]
        if h["ac"] != me: continue
        s = at(v, h["gt"])
        r = math.dist(my_pos(h["gt"]), s[1]) if s else None
        b = band(r) if r is not None else None
        if b:
            per_band[b]["hits"] += 1
            if prev is not None and prev >= h["h"]: per_band[b]["dmg"].append((prev - h["h"]) / 10)
        if h["h"] <= 0 and (prev is None or prev > 0): kills.append((h["gt"], h["vc"], r))
    # the time the gun was held on the selected target, by band
    held = collections.Counter()
    sel = [(s["gt"], s.get("selected", 0)) for s in snaps]
    st = [x[0] for x in sel]
    for k in range(len(frames) - 1):
        f = frames[k]
        if not f.get("mb", 0) & 1: continue
        j = bisect.bisect_left(st, f["gt"])
        if j >= len(sel): continue
        s = at(sel[j][1], f["gt"])
        if not s: continue
        b = band(math.dist(f["pos"], s[1]))
        if b: held[b] += frames[k + 1]["gt"] - f["gt"]
    print("  range m     gun held (s)  hits  hits/s held   damage a hit (median, quartiles)")
    for a, bb in BANDS:
        b = f"{a}-{bb}"; p = per_band[b]; dm = sorted(p["dmg"])
        q = (f"{dm[len(dm)//2]:.1f} ({dm[len(dm)//4]:.1f}-{dm[3*len(dm)//4]:.1f})" if dm else "-")
        print(f"  {b:10} {held[b]:8.1f}     {p['hits']:5d}   {p['hits'] / held[b] if held[b] else 0:6.1f}        {q}")
    print("  kills by the player's gun (health out on the player's hit):")
    for t, vc, r in kills:
        print(f"    {t:7.1f}  {vc:34}  {r:6.0f} m" if r is not None else f"    {t:7.1f}  {vc}")
    # one kill an aircraft: a Tu-95's body, guns and wings run out together
    kinds = collections.Counter()
    seen_t = []
    for t, vc, r in kills:
        fam = "bomber" if any(k in vc for k in ("tu95", "t160", "b01b")) else "fighter"
        if any(abs(t - t2) < 0.5 and fam == f2 for t2, f2 in seen_t): continue
        seen_t.append((t, fam)); kinds[fam] += 1
    print(f"  as aircraft: {dict(kinds)}")
    # the straight-line miss behind each hit: of the rounds fired along the nose in the 1.6 s
    # before (1389 m/s plus the aircraft's velocity, falling 9.8 m/s^2), the closest any came
    # to the victim's recorded path by the time of the hit
    MUZZLE, LIFE, FALL = 1389.0, 1.6, 9.8
    def nose(att):
        p, y = math.radians(att[0]), math.radians(att[1])
        return (math.cos(p) * math.cos(y), math.cos(p) * math.sin(y), math.sin(p))
    miss = collections.defaultdict(list)   # (family, band) -> [(metres, degrees)]
    for h in mine:
        s = at(h["v"], h["gt"])
        if not s: continue
        tr = track[h["v"]]; tt = [x[0] for x in tr]
        def vpos(t):
            k = min(max(bisect.bisect_left(tt, t), 1), len(tr) - 1); a, b = tr[k - 1], tr[k]
            u = max(0.0, min(1.0, (t - a[0]) / max(b[0] - a[0], 1e-6)))
            return [a[1][j] + (b[1][j] - a[1][j]) * u for j in range(3)]
        best = None
        k1 = bisect.bisect_right(gts, h["gt"]); k0 = bisect.bisect_left(gts, h["gt"] - LIFE)
        for k in range(k0, k1):
            f = frames[k]
            if not f.get("mb", 0) & 1: continue
            n = nose(f["att"]); v = [f["vel"][j] + n[j] * MUZZLE for j in range(3)]
            age = h["gt"] - f["gt"]
            # closest approach of this round up to the hit time
            for a in (age * x / 10 for x in range(1, 11)):
                p = [f["pos"][j] + v[j] * a for j in range(3)]; p[2] -= 0.5 * FALL * a * a
                dd = math.dist(p, vpos(f["gt"] + a))
                if best is None or dd < best[0]: best = (dd, math.dist(f["pos"], vpos(f["gt"] + a)))
        if best is None: continue
        fam = "bomber" if any(k in h["vc"] for k in ("tu95", "t160", "b01b")) else "fighter"
        b = band(best[1])
        if b: miss[(fam, b)].append((best[0], math.degrees(math.atan2(best[0], best[1]))))
    print("  straight-line miss behind each hit (the closest round along the nose):")
    for fam in ("bomber", "fighter"):
        for a, bb in BANDS:
            xs = miss.get((fam, f"{a}-{bb}"), [])
            if len(xs) < 5: continue
            m = sorted(x[0] for x in xs); g = sorted(x[1] for x in xs)
            print(f"    {fam:7} {a:4d}-{bb:<4d} m  hits {len(xs):4d}  miss median {m[len(m)//2]:5.0f} m (p90 {m[int(len(m)*0.9)]:5.0f})  as an angle median {g[len(g)//2]:4.1f} deg (p90 {g[int(len(g)*0.9)]:4.1f})")
    # down with no hit of the player's in the 3 s before: wrecks, missiles, others
    mine_t = collections.defaultdict(list)
    for h in mine: mine_t[h["v"]].append(h["gt"])
    other = 0
    for i, tr in track.items():
        hs = [x for x in tr if x[2] >= 0]
        if len(hs) < 2 or hs[0][2] <= 0 or hs[-1][2] > 0: continue
        t_out = next(x[0] for x in hs if x[2] <= 0)
        if not any(t_out - 3 <= t <= t_out + 0.2 for t in mine_t.get(i, [])): other += 1
    print(f"  went down with no gun hit of the player's in the 3 s before: {other}")
