// LADON attitude proxy for M15. Wing and flap pivots are measured from the
// runtime SK_OP0045_ladn_Body skeleton (see ladon_bones.json). The inferred
// surfaces follow the XB-70 delta/canard layout visible in M15 screenshots;
// this is not a recovered game mesh.
(() => {
  const triangles = [];
  const tri = (a, b, c, shade = 1) => triangles.push({ v: [a, b, c], shade });
  const quad = (a, b, c, d, shade = 1) => { tri(a, b, c, shade); tri(a, c, d, shade); };
  const mirror = (a, b, c, shade = 1) => {
    tri(a, b, c, shade);
    tri([a[0], -a[1], a[2]], [c[0], -c[1], c[2]], [b[0], -b[1], b[2]], shade);
  };
  const mirroredQuad = (a, b, c, d, shade = 1) => {
    mirror(a, b, c, shade); mirror(a, c, d, shade);
  };

  // The M15 plan view puts the narrow nose about 24 m ahead of the measured
  // wing root at x=9.3 m. XB-70 proportions do not apply to LADON's hull.
  // The intake attachment is measured at x=17.3,z=-2.3 m.
  const stations = [
    [33.5, .12, .45, -.30], [29, .75, .90, -.65],
    [23, 1.05, 1.25, -.95], [17.3, 1.20, 1.55, -1.20],
    [9.3, 2.05, 2.0, -1.65], [0, 3.1, 2.3, -2.15],
    [-15, 5.0, 2.7, -2.55], [-29, 7.0, 2.6, -2.5],
    [-35, 6.8, 2.1, -2.25]
  ];
  for (let i = 0; i < stations.length - 1; i++) {
    const [x, w, top, bottom] = stations[i];
    const [nx, nw, nt, nb] = stations[i + 1];
    mirroredQuad([x, 0, top], [x, w, top * .8], [nx, nw, nt * .8], [nx, 0, nt], .82);
    mirroredQuad([x, w, top * .8], [x, w, bottom], [nx, nw, nb], [nx, nw, nt * .8], .66);
    mirroredQuad([x, w, bottom], [x, 0, bottom], [nx, 0, nb], [nx, nw, nb], .50);
  }
  // A pair of recessed cockpit windows follows the user's close front view.
  // No cockpit vertices were returned by the runtime probe.
  mirroredQuad([27, .45, 1.1], [25, .9, 1.3],
    [20, 1.1, 1.6], [19, .5, 1.75], .29);
  mirroredQuad([24, .95, 1.3], [21, 1.2, 1.55],
    [18, 1.3, 1.75], [19, .75, 1.9], .24);

  // The hinge at (9.30, 9.41) is a control bone, not a surface vertex: using
  // it as the wing outline made the nose look triangular. The visible delta
  // starts at the narrow hull around x=11 and follows the M15 plan view.
  // Further out, captured flap pivots constrain the sweep and folded tips.
  const leading = [[11, 1.0, -.2], [-8.30, 14.56, -.095],
    [-14.95, 18.70, -.10], [-21.75, 22.95, -.10],
    [-28.45, 27.14, -.08], [-36.58, 29.88, -.115],
    [-43.1698, 31.3506, -2.2661]];
  const trailing = [[-35, 1.0, -.6], [-35, 14.56, -.5],
    [-35, 18.70, -.45], [-35, 22.95, -.4],
    [-36, 27.14, -.35], [-39, 29.88, -.3],
    [-43.0636, 33.5497, -5.4000]];
  for (let i = 0; i < leading.length - 1; i++) {
    mirroredQuad(leading[i], leading[i + 1], trailing[i + 1], trailing[i], i % 2 ? .90 : .96);
    mirroredQuad([trailing[i][0], trailing[i][1], trailing[i][2] - .35],
      [trailing[i + 1][0], trailing[i + 1][1], trailing[i + 1][2] - .35],
      [leading[i + 1][0], leading[i + 1][1], leading[i + 1][2] - .35],
      [leading[i][0], leading[i][1], leading[i][2] - .35], .48);
  }
  // The measured aft pivot lies inside the wing, not on its trailing edge.
  mirror([-28, 7.0, 2.0], [-37.65, 13.20, 1.83], [-35, 6.8, 1.0], .72);

  // Small forward projections are visible near the nose in the plan view.
  mirroredQuad([27, 1.0, 1.2], [24, 4.5, 1.1], [21, 4.5, 1.0],
    [20, 1.3, 1.5], .89);
  mirroredQuad([20, 1.3, .9], [21, 4.5, .7], [24, 4.5, .8],
    [27, 1.0, 1.0], .51);
  // Raised centre spine and twin vertical fins are outline estimates.
  mirroredQuad([9, 0, 2.0], [9, 1.8, 1.8], [-30, 2.0, 2.4], [-35, 0, 2.1], .70);
  for (const side of [-1, 1]) {
    const fin = [[-24, side * 5.5, 2.2], [-32, side * 7.2, 8.5], [-36, side * 7.0, 1.4]];
    tri(...fin, .80); tri(fin[0], fin[2], fin[1], .58);
    // Intake positions are measured; ducts/nozzles are schematic.
    const y = side * 1.7;
    quad([17.3, y - 1.7, -2.3], [17.3, y + 1.7, -2.3],
      [-34, y + 2.0, -2.0], [-34, y - 2.0, -2.0], .54);
  }

  window.LADON_ATTITUDE_MESH = triangles;
})();
