// Rough aircraft models for the review page, drawn from published dimensions and reference
// pictures (none taken from the game's files). Body axes in metres: x forward, y right, z up,
// the origin near the middle of the airframe. Every part is a convex plate or a body of
// revolution-like stations; the page lights and sorts the triangles itself.
//
//   AC8_MODELS.forClass(className)  -> model for an enemy class (BP_OP1020_m29a_..., ladn, ...)
//   AC8_MODELS.byId[id]             -> model; model.tris(sweep) gives [{v: [p, p, p], dark}]
//   model.length                    -> metres, for the level of detail
(function () {
'use strict';
const rad = Math.PI / 180;

// A body through stations [x, half width, height above, depth below], eight points around.
function body(stations, o = {}) {
  const y0 = o.y || 0, z0 = o.z || 0, n = 8, tris = [];
  const ring = s => {
    const pts = [];
    for (let k = 0; k < n; k++) {
      const a = Math.PI / 2 - k * 2 * Math.PI / n, c = Math.cos(a), si = Math.sin(a);
      pts.push([s[0], y0 + s[1] * c, z0 + (si >= 0 ? s[2] : s[3]) * si]);
    }
    return pts;
  };
  const rings = stations.map(ring);
  for (let i = 1; i < rings.length; i++)
    for (let k = 0; k < n; k++) {
      const a = rings[i - 1][k], b = rings[i - 1][(k + 1) % n], c = rings[i][(k + 1) % n], d = rings[i][k];
      add(tris, [a, b, c], o.dark); add(tris, [a, c, d], o.dark);
    }
  if (o.end) { const r = rings[rings.length - 1]; for (let k = 1; k < n - 1; k++) add(tris, [r[0], r[k], r[k + 1]], true); }   // nozzle
  return tris;
}
// A flat convex plate; points [x, y] (at height o.z) or [x, y, z].
function plate(points, o = {}) {
  const p = points.map(q => q.length === 3 ? q : [q[0], q[1], o.z || 0]), tris = [];
  for (let k = 1; k < p.length - 1; k++) add(tris, [p[0], p[k], p[k + 1]], o.dark);
  return tris;
}
function add(tris, v, dark) {
  const e1 = [v[1][0] - v[0][0], v[1][1] - v[0][1], v[1][2] - v[0][2]], e2 = [v[2][0] - v[0][0], v[2][1] - v[0][1], v[2][2] - v[0][2]];
  const area = Math.hypot(e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]);
  if (area > 1e-6) tris.push({ v, dark: !!dark });
}
// The part and its mirror image across the centre line.
const both = tris => tris.concat(tris.map(t => ({ v: t.v.map(p => [p[0], -p[1], p[2]]), dark: t.dark })));
const scale = (tris, sx, sy, sz) => tris.map(t => ({ v: t.v.map(p => [p[0] * sx, p[1] * sy, p[2] * sz]), dark: t.dark }));
const fixed = (length, tris) => ({ length, tris: () => tris });

// Su-35 (21.9 m long, 15.3 m span, 5.9 m high): blended body, nacelles apart, twin upright fins.
function flanker() {
  return [].concat(
    body([[11.0, 0, 0, 0], [10.0, .35, .35, .35], [8.5, .6, .6, .55], [6.5, .8, .85, .7], [4.5, .95, 1.0, .7], [2, 1.1, .75, .55],
          [-2, 1.1, .6, .45], [-6, .9, .55, .4], [-9.5, .5, .4, .3], [-11.0, .25, .25, .2]]),
    body([[7.6, 0, 0, 0], [6.6, .38, .45, .05], [4.6, .42, .55, .05], [3.0, .2, .25, .05], [2.2, 0, 0, 0]], { z: .75, dark: true }),
    both(body([[4.0, .62, .45, .55], [1.5, .75, .55, .6], [-5, .75, .55, .55], [-8.8, .62, .5, .5], [-10.6, .55, .45, .45]], { y: 1.55, z: -.55, end: true })),
    both(plate([[7.2, .9], [3.0, 2.4], [-5.8, 2.4], [-6.2, 1.0]], { z: .1 })),          // leading-edge root extension
    both(plate([[3.0, 2.4], [-4.6, 7.65], [-6.1, 7.65], [-5.8, 2.4]], { z: .1 })),      // wing
    both(plate([[-6.8, 2.3], [-9.6, 4.95], [-10.7, 4.95], [-10.9, 2.3]], { z: -.2 })),  // stabilator
    both(plate([[-4.6, 2.15, .6], [-8.4, 2.15, 4.9], [-9.6, 2.15, 4.9], [-9.6, 2.15, .5]])));   // fin
}
// MiG-29 (17.3 m, 11.4 m span): the same layout, smaller, the fins canted out.
function fulcrum() {
  const t = scale(flanker(), .79, .75, .8);
  return t.map(tri => ({ v: tri.v.map(p => {
    if (p[0] < -3.5 && Math.abs(p[1]) > 1.55 && Math.abs(p[1]) < 1.75 && p[2] > .3) {   // fin points: lean out 8 deg
      const s = Math.sign(p[1]); return [p[0], p[1] + s * Math.sin(8 * rad) * (p[2] - .4), p[2]];
    }
    return p;
  }), dark: tri.dark }));
}
// Light single-engine fighter, for the weak airframe (15 m, 9.5 m span).
function light() {
  return [].concat(
    body([[7.5, 0, 0, 0], [6.5, .35, .35, .35], [4.5, .6, .65, .55], [2.5, .75, .9, .7], [0, .85, .8, .75], [-4, .8, .7, .7], [-6.5, .55, .55, .5], [-7.5, .45, .45, .45]], { end: true }),
    body([[5.2, 0, 0, 0], [4.4, .35, .4, .05], [2.6, .38, .45, .05], [1.2, 0, 0, 0]], { z: .55, dark: true }),
    body([[2.8, .55, .35, .35], [0, .6, .4, .4], [-2.5, .4, .3, .3]], { z: -.7 }),
    both(plate([[4.5, .6], [1.8, 1.3], [-3.6, 1.3], [-3.6, .6]])),
    both(plate([[1.8, 1.3], [-1.6, 4.75], [-3.2, 4.75], [-3.6, 1.3]])),
    both(plate([[-4.8, .7], [-6.6, 2.8], [-7.6, 2.8], [-7.4, .7]])),
    plate([[-3.0, 0, .8], [-6.0, 0, 4.2], [-7.0, 0, 4.2], [-7.2, 0, .7]]));
}
// Mirage 2000 (14.4 m, 9.1 m span): tailless delta, half-cone intakes, one fin.
function mirage() {
  return [].concat(
    body([[7.2, 0, 0, 0], [6.0, .3, .3, .3], [4.2, .55, .6, .5], [2.5, .65, .75, .6], [0, .75, .75, .65], [-4, .7, .7, .6], [-6.4, .55, .55, .5], [-7.2, .45, .45, .45]], { end: true }),
    body([[4.8, 0, 0, 0], [3.9, .33, .42, .05], [2.0, .35, .45, .05], [0.6, 0, 0, 0]], { z: .55, dark: true }),
    both(body([[2.8, .35, .45, .45], [1, .45, .55, .5], [-2.5, .3, .4, .4]], { y: .95, z: -.05 })),
    both(plate([[2.6, .7], [-4.8, 4.55], [-5.6, 4.55], [-5.9, .7]], { z: -.2 })),
    plate([[-2.2, 0, .7], [-5.5, 0, 3.9], [-6.5, 0, 3.9], [-6.8, 0, .6]]));
}
// MiG-21 (14.7 m, 7.15 m span): a tube with a nose intake, small delta, swept fin and tail.
function fishbed() {
  return [].concat(
    body([[7.9, 0, 0, 0], [7.3, .3, .3, .3]], { dark: true }),
    body([[7.4, .45, .45, .45], [6.5, .6, .6, .6], [4, .68, .75, .65], [0, .7, .7, .7], [-4, .68, .68, .68], [-6.5, .55, .55, .55], [-7.3, .5, .5, .5]], { end: true }),
    body([[4.6, 0, 0, 0], [3.8, .3, .35, .05], [2.2, .28, .4, .05], [0.5, 0, 0, 0]], { z: .6, dark: true }),
    both(plate([[0.8, .65], [-3.6, 3.58], [-4.2, 3.58], [-4.4, .65]], { z: -.1 })),
    both(plate([[-4.8, .6], [-6.4, 2.6], [-6.9, 2.6], [-6.9, .6]])),
    plate([[-2.0, 0, .6], [-5.6, 0, 3.3], [-6.5, 0, 3.3], [-7.0, 0, .5]]));
}
// A-6E (16.6 m, 16.15 m span): broad nose, side-by-side canopy, straight wing, pod engines.
function intruder() {
  return [].concat(
    body([[8.3, 0, 0, 0], [7.8, .6, .55, .55], [6.8, 1.0, .9, .8], [5, 1.2, 1.1, .9], [2, 1.2, 1.0, .9], [-2, 1.0, .8, .8], [-5, .6, .6, .6], [-8.3, .3, .4, .3]]),
    body([[6.2, 0, 0, 0], [5.6, .9, .5, .05], [3.4, .85, .5, .05], [2.2, 0, 0, 0]], { z: .9, dark: true }),
    both(body([[3.2, .5, .45, .45], [0, .55, .5, .5], [-3.5, .45, .4, .4]], { y: 1.25, z: -.8, end: true })),
    both(plate([[1.8, 1.0], [-2.0, 8.07], [-3.6, 8.07], [-3.0, 1.0]], { z: .1 })),
    both(plate([[-5.4, .4], [-7.0, 3.3], [-8.0, 3.3], [-7.9, .4]], { z: .3 })),
    plate([[-3.8, 0, .8], [-6.9, 0, 4.1], [-7.9, 0, 4.1], [-8.2, 0, .6]]));
}
// Drone (about 5 m): a small flying wing with twin fins.
function drone() {
  return [].concat(
    body([[2.5, 0, 0, 0], [1.5, .4, .35, .3], [-1.5, .4, .3, .3], [-2.5, .25, .2, .2]], { end: true }),
    both(plate([[1.2, .3], [-1.2, 3], [-1.8, 3], [-1.6, .3]])),
    both(plate([[-1.2, .4, .2], [-2.2, .9, 1.1], [-2.6, .9, 1.1], [-2.5, .4, .2]])));
}
// LADON (official: 128 m long, 18.7 m high; 87.6 m span swept, 148.6 m spread). Swept back it is
// essentially an XB-70: a long slender nose with canards behind the cockpit, one large delta
// with a straight trailing edge, six engines side by side in a box under the delta with the
// nozzles in a row, twin fins on the trailing edge, and the outer parts of the delta folding
// down. The outer wings are long narrow wings on pivots, each joined to the delta's leading
// edge by a small isosceles fairing: spread they stand out swept 30 deg; swept they turn back
// along the leading edge, completing the big delta, the outer part folded down.
function engineBox(x0, x1, w0, w1, zt, zb, ramp) {   // intake box under the wing, w0 wide in front, w1 at the back
  const P = (x, y, z) => [x, y, z], wr = w0 + (w1 - w0) * ramp / (x0 - x1);
  return [].concat(
    plate([P(x0, -w0, zt), P(x0, w0, zt), P(x0 - ramp, wr, zb), P(x0 - ramp, -wr, zb)]),   // intake ramp
    plate([P(x0 - ramp, -wr, zb), P(x0 - ramp, wr, zb), P(x1, w1, zb), P(x1, -w1, zb)]),   // bottom
    both(plate([P(x0, w0, zt), P(x1, w1, zt), P(x1, w1, zb), P(x0 - ramp, wr, zb)])),      // sides
    plate([P(x1, -w1, zt), P(x1, w1, zt), P(x1, w1, zb), P(x1, -w1, zb)], { dark: true }));  // rear face
}
function ladonFixed() {
  const nozzles = [];
  for (const y of [3, 9, 15]) nozzles.push(...both(body([[-57.5, 2.5, 2.5, 2.5], [-60, 2.3, 2.3, 2.3]], { y, z: -3.6, end: true })));   // half the trailing edge a side
  return [].concat(
    // nose tip at 68 m, nozzles end at -60 m: 128 m overall
    body([[68, 0, 0, 0], [64.8, .9, .8, .8], [60.5, 1.8, 1.6, 1.4], [55.2, 2.6, 2.3, 2.0], [47.7, 3.1, 2.7, 2.4], [37, 3.4, 2.8, 2.6], [21, 3.8, 3.0, 2.8], [5, 4.2, 3.0, 3.0]]),
    body([[59.5, 0, 0, 0], [57.9, 1.4, .6, .05], [55.8, 1.5, .6, .05], [54.7, 0, 0, 0]], { z: 1.6, dark: true }),
    body([[12, 3.8, 3.0, 3.0], [0, 4.2, 3.0, 2.0], [-30, 4.2, 2.6, 1.6], [-50, 3.6, 2.0, 1.2], [-58, 2.6, 1.4, 1.0]]),   // spine over the box
    // the fixed wing (user's sketch): an isosceles triangle with the pivot at its outer
    // corner, its rear side running back in to 9.6 m; from that corner a triangle flaring out
    // to the trailing-edge corner (its leading edge stays under the swept wing: no gap)
    both(plate([[18.6, 3.5], [0.4, 13.6], [-13, 9.6], [-13, 3.5]])),
    both(plate([[-13, 9.6], [-58, REAR_CORNER], [-58, 3.5], [-13, 3.5]])),
    engineBox(-8, -57.5, 9, 18, -.5, -6.2, 9),                                             // six engines in one box, widening aft
    nozzles,
    both(plate([[44, 2.5], [38, 9], [35.5, 9], [35, 2.5]], { z: .5 })),                  // canards
    both(plate([[-38, 15, .3], [-53, 16.3, 12.4], [-58.5, 16.3, 12.4], [-58.5, 15, .3]])));   // fins, in line with the outer engines; 18.7 m fin tip to box bottom
}
// The outer wing, rigid, drawn spread: its root along the front triangle's rear side, the
// pivot at that triangle's outer corner, 26 deg leading-edge sweep, tip at 74.3 m. Swept, it
// turns back about the pivot until the tip is at 43.8 m, its leading edge carrying on the
// triangle's, lying over the rear triangle; outboard of a fold line that is parallel to the
// engines when swept, at the rear triangle's corner (35.2 m out), its outer part is raised 12 deg
// when spread and folded down 35.4 deg when swept (both measured from the skeleton).
const PIVOT = [0.4, 13.6];
const WING = [[0.4, 13.6], [-29.2, 74.3], [-36, 73.6], [-9.4, 10.6]];   // root on the outer part of the triangle's rear side, 26 deg sweep; chord ~11 m root, ~7 m tip
const REAR_CORNER = 35.2;   // on the fold line, under the swept wing's inner edge: only the outer wing folds
function panelAt(theta) {   // turned back by theta about the pivot
  const c = Math.cos(theta), s = Math.sin(theta);
  return WING.map(([x, y]) => {
    const dx = x - PIVOT[0], dy = y - PIVOT[1];
    return [PIVOT[0] + dx * c - dy * s, PIVOT[1] + dx * s + dy * c, 0];
  });
}
const SWEEP_BACK = (() => {
  let lo = 0, hi = Math.PI / 2;
  for (let k = 0; k < 40; k++) { const m = (lo + hi) / 2; panelAt(m)[1][1] > 43.8 ? lo = m : hi = m; }
  return lo;
})();
const FOLD_Y = REAR_CORNER, FOLD_SPREAD = -12 * rad, FOLD_MAX = 35.4 * rad;   // measured: up 12 deg spread, down 35.4 swept
// The fold line is fixed on the wing: where its leading and trailing edges cross y = FOLD_Y
// when fully swept, as fractions along those edges (a rigid turn keeps the fractions).
const FOLD_AT = (() => {
  const [r0, t0, t1, r1] = panelAt(SWEEP_BACK);
  return [(FOLD_Y - r0[1]) / (t0[1] - r0[1]), (FOLD_Y - r1[1]) / (t1[1] - r1[1])];
})();
function outerWing(sweep) {
  const [r0, t0, t1, r1] = panelAt(SWEEP_BACK * sweep);
  const lerp = (a, b, k) => [a[0] + (b[0] - a[0]) * k, a[1] + (b[1] - a[1]) * k, 0];
  const h0 = lerp(r0, t0, FOLD_AT[0]), h1 = lerp(r1, t1, FOLD_AT[1]);
  const ux = h1[0] - h0[0], uy = h1[1] - h0[1], ul = Math.hypot(ux, uy) || 1;
  let nx = -uy / ul, ny = ux / ul;
  if ((t0[0] - h0[0]) * nx + (t0[1] - h0[1]) * ny < 0) { nx = -nx; ny = -ny; }
  const fold = FOLD_SPREAD + (FOLD_MAX - FOLD_SPREAD) * sweep, c = Math.cos(fold), s = Math.sin(fold);
  const folded = q => {
    const d = (q[0] - h0[0]) * nx + (q[1] - h0[1]) * ny;
    return [q[0] - nx * d * (1 - c), q[1] - ny * d * (1 - c), -d * s];
  };
  return plate([r0, h0, h1, r1]).concat(plate([h0, folded(t0), folded(t1), h1]));
}
function ladon() {
  const base = ladonFixed(), cache = new Map();
  return { length: 128, tris(sweep = 1) {
    const key = Math.round(Math.max(0, Math.min(1, sweep)) * 20);
    if (!cache.has(key)) cache.set(key, base.concat(both(outerWing(key / 20))));
    return cache.get(key);
  } };
}

// A conventional fighter laid out on what the mesh probe measured (models/probed_<id>.js, made
// by build_model.py): the mesh's box (nose, tail, half span, top) and the skeleton's hinge
// points (intakes, canopy, leading-edge flaps, ailerons, elevators, rudders, nozzles, canards).
// The shapes between them are the template's: a 42 deg leading edge (or, without a leading-edge
// bone, from behind the canards to a narrow tip), straight trailing edge, elliptical sections.
function fromProbe(p) {
  const L = p.nose - p.tail, tanLE = Math.tan(42 * rad);
  // engines: twin or single by the nozzles; without nozzle bones, twin when the fins are a pair
  // (side by side inside them, as on the ADF-X02), else one on the centre line
  const twinFins = p.rudder && p.rudder[1] > 0.3;
  const nz = p.nozzle || [p.tail + 2, twinFins ? Math.max(0.9, p.rudder[1] * 0.45) : 0, 0];
  const yn = Math.abs(nz[1]) > 0.3 ? Math.abs(nz[1]) : 0;
  const rn = Math.max(0.45, Math.min(0.75, (yn || 0.9) * 0.55));
  const hw = Math.max(0.7, yn || 0.8);                       // centre body half width
  const zc = p.canopy ? p.canopy[2] - 0.75 : 0.4;            // fuselage top under the canopy
  const intake = p.intake || [p.nose - 0.45 * L, yn || 0.9, 0];
  const cx = p.canopy_x || [p.nose - 0.32 * L, p.nose - 0.27 * L];
  const tris = [].concat(
    body([[p.nose, 0, 0, 0], [p.nose - 0.06 * L, .32, .32, .3], [cx[1] + 0.4, .6, zc * .8, .55],
          [intake[0], hw * .85, zc, .6], [0, hw, zc * .85, .55], [nz[0], hw * .8, .55, .45], [p.tail + 0.4, .35, .3, .25]]),
    body([[cx[1] + 0.6, 0, 0, 0], [cx[1], .42, .45, .05], [cx[0] - 0.6, .45, .5, .05], [cx[0] - 1.8, .2, .25, .05]], { z: zc, dark: true }));
  // engines: from the intakes to the nozzles, open at the back
  const nacelle = [[intake[0], rn * .9, rn * .7, rn], [intake[0] - 2, rn * 1.05, rn * .85, rn], [nz[0], rn, rn * .85, rn * .9], [p.tail + 0.2, rn * .8, rn * .75, rn * .75]];
  tris.push(...(yn ? both(body(nacelle, { y: yn, z: nz[2], end: true })) : body(nacelle, { y: 0, z: nz[2] - 0.2, end: true })));
  // wing. Trailing edge: through the flap and aileron hinges when both are known (0.6 m behind
  // them), else 0.7 m behind the aileron hinge, straight. Leading edge: 0.35 m ahead of the
  // leading-edge flap hinge at 42 deg; without that bone, 42 deg in from a tip chord of 1.4 m,
  // its root no further forward than the canards or the canopy.
  const S = p.half_span, yr = (yn || 0) + rn;
  const ai = p.aileron || [p.nose - 0.62 * L, 0.55 * S, 0];
  const te = p.flap
    ? y => p.flap[0] - 0.3 + (y - p.flap[1]) * (ai[0] - p.flap[0]) / ((ai[1] - p.flap[1]) || 1)
    : y => ai[0] - 0.7;
  const teTip0 = te(S), lfb = p.le_flap;
  // without a leading-edge bone: from the fuselage just behind the canards (or the canopy)
  // straight out to a 1.1 m tip chord
  const leRootFree = p.canard ? p.canard[0] - 1.2 : cx[0] - 2.5, leTipFree = teTip0 + 1.1;
  const ll = p.le_line;   // measured: two leading-edge hinges, 0.35 m behind the edge
  const le = ll ? y => ll[0][0] + 0.35 + (y - ll[0][1]) * (ll[1][0] - ll[0][0]) / (ll[1][1] - ll[0][1])
           : lfb ? y => lfb[0] + 0.35 - (y - lfb[1]) * tanLE
                 : y => leRootFree + (leTipFree - leRootFree) * (y - yr) / ((S - yr) || 1);
  const rootCap = ll ? Infinity : Math.min(p.canard ? p.canard[0] - 0.8 : Infinity, cx[0] - 1.5);
  let leRoot = Math.min(le(yr), rootCap), leTip = le(S), teTip = Math.min(teTip0, leTip - 1.0);
  // a trailing edge that runs forward toward the tip (aileron ahead of the flap) and no leading-
  // edge bone: a forward-swept wing (ADF-X02), its leading edge also running forward, with the
  // chord narrowing from 0.21 to 0.08 of the length; the body in front of the root is left open
  const forward = !lfb && !ll && p.flap && ai[0] > p.flap[0] + 0.5;
  if (forward) { leRoot = te(yr) + 0.21 * L; leTip = teTip0 + 0.08 * L; teTip = teTip0; }
  const wz = ((lfb ? lfb[2] : 0) + ai[2]) / 2;
  tris.push(...both(plate([[leRoot, yr], [leTip, S], [teTip, S], [te(yr), yr]], { z: wz })));
  // leading-edge root extension: with canards, a strake from beside the canard root that meets
  // the wing's leading edge a little way out (the cranked leading edge of the ADF-X02);
  // otherwise from behind the canopy to the wing root
  if (forward) {
    // only a small fairing where the wing meets the body
    tris.push(...both(plate([[leRoot + 1.0, hw * .8], [leRoot, yr], [te(yr), yr], [te(yr), hw * .8]], { z: wz })));
  } else if (p.canard) {
    const yk = yr + 0.25 * (S - yr), xk = le(yk);
    tris.push(...both(plate([[p.canard[0] + 0.6, hw * .8], [xk, yk], [te(yk), yk], [te(hw * .8), hw * .8]], { z: wz })));
  } else if (leRoot < cx[0] - 1.3) tris.push(...both(plate([[cx[0] - 1.2, hw * .7], [leRoot, yr], [te(yr), yr], [te(yr), hw * .7]], { z: wz })));
  // horizontal tail on the elevator pivot
  if (p.elevator) {
    const [ex, ey, ez] = p.elevator, ys = Math.max(ey - 0.3, hw * .8), span = Math.max(ey + 1.5, S * .66);
    tris.push(...both(plate([[ex + 1.0, ys], [ex + 1.0 - (span - ys) * Math.tan(40 * rad), span], [ex - 0.9 - (span - ys) * Math.tan(40 * rad) + 1.4, span], [ex - 1.6, ys]], { z: ez })));
  }
  // fins on the rudder hinges (twin when the hinge is off the centre line), up to the box's top
  if (p.rudder) {
    const [rx, ry, rz] = p.rudder, h = p.top - rz, twin = ry > 0.3;
    // cant from the hinge's two points (y change per metre up), when measured
    const lean = p.rudder_top && p.rudder_top[2] > rz + 0.2 ? (p.rudder_top[1] - ry) / (p.rudder_top[2] - rz) : 0;
    const y0 = twin ? ry : 0, yTop = twin ? ry + lean * h : 0;
    const fin = [[rx + 3.0, y0, rz - 0.2], [rx + 3.0 - h * Math.tan(45 * rad), yTop, p.top], [rx - 0.6 - h * Math.tan(25 * rad), yTop, p.top], [rx - 0.6, y0, rz - 0.2]];
    tris.push(...(twin ? both(plate(fin)) : plate(fin)));
  }
  if (p.canard) {
    const [kx, ky, kz] = p.canard;
    tris.push(...both(plate([[kx + 1.2, 0.6], [kx - 0.6, ky + 1.6], [kx - 1.4, ky + 1.6], [kx - 1.2, 0.6]], { z: kz })));
  }
  return { length: L, tris: () => tris, probed: true };
}
const probedCache = {};
function probed(id) {
  const p = window.AC8_PROBED && window.AC8_PROBED[id];
  if (!p) return null;
  return probedCache[id] || (probedCache[id] = fromProbe(p));
}

const byId = {
  su35: fixed(21.9, flanker()), mig29: fixed(17.3, fulcrum()), light: fixed(15, light()), mirage: fixed(14.4, mirage()),
  mig21: fixed(15.8, fishbed()), a6: fixed(16.6, intruder()), drone: fixed(5, drone()), ladon: ladon(),
};
const RULES = [[/ladn/i, 'ladon'], [/su35|su27|su37|su57/i, 'su35'], [/m29/i, 'mig29'], [/mr2k|m2k|mir2/i, 'mirage'], [/m21/i, 'mig21'],
               [/a06|a6e/i, 'a6'], [/uav/i, 'drone']];
function forClass(cls) { for (const [re, id] of RULES) if (re.test(cls || '')) return byId[id]; return byId.light; }
window.AC8_MODELS = { byId, forClass, probed };
})();
