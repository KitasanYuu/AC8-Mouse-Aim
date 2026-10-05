"""Player aircraft model from a mesh-probe snapshot (F11 in game, mesh_probe.lua).

The probe reads, from the running game, the size of the aircraft's skeletal mesh (its asset
bounds) and the positions of the skeleton's bones: the hinges of the control surfaces (leading
-edge flaps, ailerons, rudders, elevators), the intakes, the canopy and the nozzles. This script
turns those into the few numbers models.js needs to lay out a conventional fighter
(window.AC8_PROBED[<id>] in models/probed_<id>.js); models.js builds the triangles.

    python dev/compare/build_model.py <mesh-probe-*.json> [--id su35]

Only measured numbers are written; the shapes between them (sweeps, chords, cross-sections)
are the template's. No mesh vertex is read or written.
"""
import argparse
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def body_mesh(snapshot):
    """The aircraft's main skeletal mesh: the component with bones whose bounds are largest
    (the cockpit mesh shares them; the body is preferred by name)."""
    best = None
    for c in snapshot.get("components") or []:
        if not isinstance(c, dict) or not c.get("bones") or not c.get("asset_bounds"):
            continue
        name = (c.get("name") or "").split(".")[-1]
        score = (("Body" in name) * 10 + 1) * max(c["asset_bounds"]["extent"])
        if best is None or score > best[0]:
            best = (score, c)
    return best[1] if best else None


def bones_of(component):
    out = {}
    for b in component["bones"]:
        t = (b.get("component") or {}).get("translation")
        if t:
            out[b["name"]] = [round(v / 100, 3) for v in t]   # cm -> m, X forward, Y right, Z up
    return out


def pick(bones, pattern):
    return [v for k, v in bones.items() if re.search(pattern, k, re.I)]


def mean(points):
    return [round(sum(p[i] for p in points) / len(points), 3) for i in range(3)] if points else None


def outer(points):
    """A left/right pair as one point on the right side (y >= 0)."""
    if not points:
        return None
    p = mean([[x, abs(y), z] for x, y, z in points])
    return p


def le_line(points):
    right = sorted({(round(x, 3), round(abs(y), 3), round(z, 3)) for x, y, z in points}, key=lambda p: p[1])
    if len(right) < 2 or right[-1][1] - right[0][1] < 1.0:
        return None
    return [list(right[0]), list(right[-1])]


def params(snapshot, model_id):
    comp = body_mesh(snapshot)
    if not comp:
        sys.exit("no skeletal mesh with bones and bounds in this snapshot")
    b = comp["asset_bounds"]
    ox, oy, oz = (v / 100 for v in b["origin"])
    ex, ey, ez = (v / 100 for v in b["extent"])
    bones = bones_of(comp)
    canopy = pick(bones, r"canopy")
    out = {
        "id": model_id,
        "source": "mesh probe: " + (snapshot.get("actor") or "?").split(" ")[0],
        "mesh": (comp.get("mesh") or "").split(" ")[-1],
        # the mesh's box: nose and tail, half span, bottom and top (m)
        "nose": round(ox + ex, 2), "tail": round(ox - ex, 2), "half_span": round(ey, 2),
        "bottom": round(oz - ez, 2), "top": round(oz + ez, 2),
        # measured points, right side
        "canopy": mean(canopy),
        "canopy_x": [min(p[0] for p in canopy), max(p[0] for p in canopy)] if canopy else None,
        "canopy_half_width": round(max(abs(p[1]) for p in canopy), 2) if canopy else None,
        "intake": outer(pick(bones, r"airintake_0?1$")) or outer(pick(bones, r"intake")),
        "intake_back": outer(pick(bones, r"airintake_0?2$")),
        "le_flap": outer(pick(bones, r"frontflap|leadingedge|slat")),
        # two or more leading-edge hinges a side: the innermost and outermost give the edge itself
        "le_line": le_line(pick(bones, r"[lr]_(frontflap|leadingedge|slat)")),
        "aileron": outer(pick(bones, r"aileron|flaperon")),
        "flap": outer(pick(bones, r"[lr]_flap_0?1$")),
        # fins: the rudder hinge's lower and upper points when the skeleton has both (cant)
        "rudder_top": outer(pick(bones, r"rudder_0?2$")),
        "elevator": outer(pick(bones, r"elevator|stabilator|htail")),
        "rudder": outer(pick(bones, r"rudder_0?1$")) or outer(pick(bones, r"rudder")),
        "canard": outer(pick(bones, r"canard")),
        "nozzle": outer(pick(bones, r"[lr]_nozzle")) or outer(pick(bones, r"nozzle")),
        "bones": len(bones),
    }
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("snapshot")
    ap.add_argument("--id", help="model id (default: from the class name, e.g. su35)")
    args = ap.parse_args()
    snap = json.load(open(args.snapshot, encoding="utf-8"))
    comp = body_mesh(snap)
    named = " ".join([(snap.get("actor") or ""), (comp or {}).get("mesh") or ""])
    found = re.search(r"P{1,2}\d+_([a-z0-9]+)", named, re.I)
    model_id = args.id or (found.group(1).lower() if found else "player")
    p = params(snap, model_id)
    path = os.path.join(HERE, "models", f"probed_{model_id}.js")
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(f"// {p['source']} ({p['mesh']}), measured by dev/compare/build_model.py:\n")
        f.write("// the mesh's box and the skeleton's hinge points; models.js lays a fighter out on them.\n")
        f.write(f"(window.AC8_PROBED = window.AC8_PROBED || {{}})[{json.dumps(model_id)}] = ")
        f.write(json.dumps(p, ensure_ascii=False, indent=1))
        f.write(";\n")
    print(path)
    print(json.dumps(p, ensure_ascii=False))


if __name__ == "__main__":
    main()
