// Su-35 attitude proxy. Control-surface pivots and intake/nozzle positions come
// from the runtime skeleton snapshot in su35_bones.json (metres, X forward,
// Y right, Z up). Outer edges are inferred; these triangles are not game mesh
// vertices. Keep this separate from the flight-plant model and enemy silhouette.
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

  // Fuselage cross-sections. The rear width follows the twin nozzle pivots;
  // the raised cockpit follows the measured canopy bones at x=7.1..8.0 m.
  const stations = [
    [11.5, 0.05, 0.20, 0.00],
    [8.0,  0.50, 0.95, -0.28],
    [4.24, 1.18, 0.80, -0.62],
    [1.0,  1.52, 0.52, -0.58],
    [-3.5, 1.43, 0.44, -0.58],
    [-5.2, 1.25, 0.30, -0.48],
    [-7.4, 0.82, 0.22, -0.38],
  ];
  for (let i = 0; i < stations.length - 1; i++) {
    const [x, w, top, bottom] = stations[i];
    const [nx, nw, ntop, nbottom] = stations[i + 1];
    mirroredQuad([x, 0, top], [x, w, top * .84], [nx, nw, ntop * .84], [nx, 0, ntop], .88);
    mirroredQuad([x, w, top * .84], [x, w, bottom], [nx, nw, nbottom], [nx, nw, ntop * .84], .68);
    mirroredQuad([x, w, bottom], [x, 0, bottom], [nx, 0, nbottom], [nx, nw, nbottom], .52);
  }

  // Main wing: the flap pivot is (0.862, 4.333, 0.312) and the aileron pivot
  // is (-1.507, 3.757, 0.366). The tip is inferred beyond those hinges.
  const rootFront = [3.9, 1.15, .28], rootBack = [-4.2, 1.35, .30];
  const hingeFront = [.862, 4.333, .312], hingeBack = [-1.507, 3.757, .366];
  const tipFront = [-.25, 7.55, .27], tipBack = [-2.05, 7.55, .28];
  mirroredQuad(rootFront, hingeFront, hingeBack, rootBack, 1.0);
  mirroredQuad(hingeFront, tipFront, tipBack, hingeBack, .91);
  // Thin underside gives the wing a readable edge when inverted.
  mirroredQuad([rootFront[0], rootFront[1], .12], [rootBack[0], rootBack[1], .14],
    [tipBack[0], tipBack[1], .12], [tipFront[0], tipFront[1], .12], .52);

  // Horizontal stabilizers, anchored around the measured elevator hinges.
  mirroredQuad([-3.75, 1.24, -.17], [-5.45, 3.66, -.16], [-7.78, 3.35, -.14],
    [-7.42, 1.15, -.18], .84);
  // Twin canted fins, rooted at the measured rudder pivots near ±2.1 m.
  for (const side of [-1, 1]) {
    const finRoot = [-3.45, side * 2.12, .55];
    const finTop = [-5.90, side * 2.75, 3.55];
    const finBack = [-7.60, side * 2.48, .45];
    tri(finRoot, finTop, finBack, .78);
    tri(finRoot, finBack, finTop, .58);
    // The measured nozzle centre is x=-5.003, y=±1.189, z=-0.159.
    const y = side * 1.189;
    quad([2.79, y - .43, -.15], [2.79, y + .43, -.15],
      [-5.80, y + .47, -.16], [-5.80, y - .47, -.16], .57);
  }

  // Raised canopy, with a visible peak at the measured x=8.02, z=1.68.
  tri([9.25, 0, .88], [8.02, -.53, 1.06], [8.02, 0, 1.68], .57);
  tri([9.25, 0, .88], [8.02, 0, 1.68], [8.02, .53, 1.06], .57);
  mirroredQuad([8.02, 0, 1.68], [8.02, .53, 1.06], [6.88, .62, .91], [6.88, 0, 1.10], .48);

  window.SU35_ATTITUDE_MESH = triangles;
})();
