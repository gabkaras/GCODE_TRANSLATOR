/* Simplified KUKA robot: forward kinematics (same conventions as gcode2krl/robot.py),
   procedural link meshes and reach envelope for the PathViewer. */
'use strict';
(() => {
  const DEG = Math.PI / 180;
  // ---- mat4 helpers (column-major, like WebGL) ----
  const I4 = () => new Float32Array([1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]);
  function mul(a, b) {
    const o = new Float32Array(16);
    for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++)
      o[c*4+r] = a[r]*b[c*4] + a[4+r]*b[c*4+1] + a[8+r]*b[c*4+2] + a[12+r]*b[c*4+3];
    return o;
  }
  const T = (x, y, z) => new Float32Array([1,0,0,0, 0,1,0,0, 0,0,1,0, x,y,z,1]);
  const RX = a => { const c = Math.cos(a), s = Math.sin(a); return new Float32Array([1,0,0,0, 0,c,s,0, 0,-s,c,0, 0,0,0,1]); };
  const RY = a => { const c = Math.cos(a), s = Math.sin(a); return new Float32Array([c,0,-s,0, 0,1,0,0, s,0,c,0, 0,0,0,1]); };
  const RZ = a => { const c = Math.cos(a), s = Math.sin(a); return new Float32Array([c,s,0,0, -s,c,0,0, 0,0,1,0, 0,0,0,1]); };
  const apply = (m, p) => [m[0]*p[0]+m[4]*p[1]+m[8]*p[2]+m[12], m[1]*p[0]+m[5]*p[1]+m[9]*p[2]+m[13], m[2]*p[0]+m[6]*p[1]+m[10]*p[2]+m[14]];
  function fromRowMajor(a) { const o = new Float32Array(16); for (let r = 0; r < 4; r++) for (let c = 0; c < 4; c++) o[c*4+r] = a[r*4+c]; return o; }
  const FLANGE0 = RY(Math.PI / 2);

  /** URDF-style chain: frames [base, link_1 .. link_6, flange] for joints in degrees (same as robot.py) */
  function forward(R, q) {
    const a = q.map(v => v * DEG), o = R.origins;
    const rots = [RZ(-a[0]), RY(a[1]), RY(a[2]), RX(-a[3]), RY(a[4]), RX(-a[5])];
    let t = I4();
    const f = [t];
    for (let k = 0; k < 6; k++) { t = mul(t, T(o[k][0], o[k][1], o[k][2])); t = mul(t, rots[k]); f.push(t); }
    t = mul(t, T(o[6][0], o[6][1], o[6][2])); t = mul(t, FLANGE0); f.push(t);
    return { frames: f, upper: f[2], fore: f[3], w4: f[4], w5: f[5] };
  }

  // ---- procedural meshes ----
  function box(sx, sy, sz, cx, cy, cz) {
    const P = [], N = [];
    const h = [sx/2, sy/2, sz/2], c = [cx||0, cy||0, cz||0];
    const faces = [[0,1],[0,-1],[1,1],[1,-1],[2,1],[2,-1]];
    for (const [ax, sg] of faces) {
      const u = (ax + 1) % 3, v = (ax + 2) % 3;
      const n = [0,0,0]; n[ax] = sg;
      const quad = [];
      for (const [su, sv] of [[-1,-1],[1,-1],[1,1],[-1,1]]) { const p = [0,0,0]; p[ax] = sg*h[ax]; p[u] = su*h[u]; p[v] = sv*h[v]; quad.push(p.map((x,i)=>x+c[i])); }
      const order = sg > 0 ? [0,1,2,0,2,3] : [0,2,1,0,3,2];
      for (const i of order) { P.push(...quad[i]); N.push(...n); }
    }
    return { positions: new Float32Array(P), normals: new Float32Array(N) };
  }
  /** cylinder of radius r from along axis (0:x,1:y,2:z) between a0 and a1, centred at (cx,cy,cz) in the other axes */
  function cylinder(r, axis, a0, a1, cx, cy, cz, seg) {
    seg = seg || 28;
    const P = [], N = [], c = [cx||0, cy||0, cz||0];
    const u = (axis + 1) % 3, v = (axis + 2) % 3;
    const pt = (ang, h) => { const p = [0,0,0]; p[axis] = h; p[u] = r*Math.cos(ang); p[v] = r*Math.sin(ang); return p.map((x,i)=>x+c[i]); };
    const nr = ang => { const n = [0,0,0]; n[u] = Math.cos(ang); n[v] = Math.sin(ang); return n; };
    for (let i = 0; i < seg; i++) {
      const t0 = 2*Math.PI*i/seg, t1 = 2*Math.PI*(i+1)/seg;
      const p00 = pt(t0, a0), p01 = pt(t0, a1), p10 = pt(t1, a0), p11 = pt(t1, a1);
      const n0 = nr(t0), n1 = nr(t1);
      P.push(...p00, ...p10, ...p11, ...p00, ...p11, ...p01); N.push(...n0, ...n1, ...n1, ...n0, ...n1, ...n0);
      const nc = [0,0,0]; nc[axis] = 1; const nc0 = [0,0,0]; nc0[axis] = -1;
      const c1 = [0,0,0]; c1[axis] = a1; const c0 = [0,0,0]; c0[axis] = a0;
      P.push(...c1.map((x,i)=>x+c[i]), ...p01, ...p11); N.push(...nc, ...nc, ...nc);
      P.push(...c0.map((x,i)=>x+c[i]), ...p10, ...p00); N.push(...nc0, ...nc0, ...nc0);
    }
    return { positions: new Float32Array(P), normals: new Float32Array(N) };
  }
  function merge(...geoms) {
    const P = [], N = [];
    for (const g of geoms) { P.push(...g.positions); N.push(...g.normals); }
    return { positions: new Float32Array(P), normals: new Float32Array(N) };
  }

  const ORANGE = 'var(--brand)', DARK = 'var(--brand-dark)', GREY = 'var(--brand-grey)', TOOL = 'var(--c-obstacle)';

  class RobotView {
    constructor(viewer) { this.viewer = viewer; this.model = null; this.robotInBase = I4(); this.tool = I4(); this.visible = true; this.reachVisible = true; this.joints = null; }

    setModel(R, meshes) {
      const v = this.viewer;
      v.removeGroup('robot');
      this.model = R;
      this.meshLinks = null;
      const g = 'robot';
      if (meshes && meshes.links && Object.keys(meshes.links).length) {
        this.meshLinks = {};
        for (const [idx, m] of Object.entries(meshes.links)) {
          const geom = { positions: decodeF32(m.positions), normals: decodeF32(m.normals) };
          const color = idx === '0' ? DARK : (idx === '6' ? GREY : ORANGE);
          v.setMesh('rb_link' + idx, geom, color, I4(), { group: g });
          this.meshLinks[idx] = true;
        }
        this.setTool(this.tool);
        this.buildReach();
        this.setPose(this.joints || R.home);
        return;
      }
      const V = R.visual || {}, br = V.base_r || 300, bh = V.base_h || 250, col = V.column || 400, arm = V.arm || 240, fa = V.forearm || 190, wr = V.wrist || 150;
      v.setMesh('rb_base', cylinder(br, 2, 0, bh, 0, 0, 0), DARK, I4(), { group: g });
      v.setMesh('rb_plate', box(br*2.2, br*2.2, 20, 0, 0, 10), DARK, I4(), { group: g });
      // column (link 1): from the base to the A2 axis, in the A1 frame (origin at base)
      // column in the link_1 frame (origin at the A2 height d1, so shift down by d1)
      const colGeom = merge(box(col, col, R.d1 - bh + col*0.3, R.a1*0.35, 0, (bh + R.d1 + col*0.3)/2 - col*0.15 - R.d1), cylinder(arm*0.7, 1, -col*0.6, col*0.6, R.a1, 0, 0));
      v.setMesh('rb_column', colGeom, ORANGE, I4(), { group: g });
      // upper arm (link 2): frame at A2 axis, +X along the arm
      v.setMesh('rb_upper', merge(box(R.a2, arm*0.8, arm, R.a2/2, 0, 0), cylinder(arm*0.75, 1, -col*0.62, col*0.62, R.a2, 0, 0)), ORANGE, I4(), { group: g });
      // forearm (link 3): frame at A3 axis, +X along forearm, A4 axis a3 below
      v.setMesh('rb_fore', merge(cylinder(fa*0.5, 0, fa*0.2, R.origins[3][0] - wr*0.9, 0, 0, R.a3), box(fa*1.2, fa*1.1, fa*1.1 + Math.abs(R.a3), fa*0.3, 0, R.a3/2)), ORANGE, I4(), { group: g });
      // wrist (A4 frame at A5 centre): hub around A5 axis (Y)
      v.setMesh('rb_wrist', cylinder(wr*0.55, 1, -wr*0.7, wr*0.7, 0, 0, 0), DARK, I4(), { group: g });
      // A5 frame: housing to the flange along +X
      v.setMesh('rb_w5', cylinder(wr*0.38, 0, 0, R.origins[4][0] + R.d6 - 12, 0, 0, 0), ORANGE, I4(), { group: g });
      // flange (in the flange frame, Z out)
      v.setMesh('rb_flange', cylinder(wr*0.42, 2, -12, 0, 0, 0, 0), GREY, I4(), { group: g });
      this.setTool(this.tool);
      this.buildReach();
      this.setPose(this.joints || R.home);
    }

    setTool(toolMat) {
      this.tool = toolMat;
      if (!this.model) return;
      const tcp = apply(toolMat, [0, 0, 0]);
      const len = Math.hypot(...tcp);
      const wr = (this.model.visual && this.model.visual.wrist) || 150;
      // tool: holder cylinder from the flange to the nozzle start + a thinner nozzle along the
      // tool Z axis (the nozzle may be angled to the flange: bent / 45 deg / 90 deg extruders)
      const zt = [toolMat[8], toolMat[9], toolMat[10]];
      const nozzle = Math.min(60, Math.max(20, len * 0.3));
      const segAlong = (a, b, r) => {
        const d = [b[0]-a[0], b[1]-a[1], b[2]-a[2]]; const L = Math.hypot(...d); if (L < 1e-6) return null;
        const z = d.map(x => x / L), x0 = Math.abs(z[2]) < 0.9 ? [0,0,1] : [1,0,0];
        const y = norm(cross(z, x0)), x = cross(y, z);
        const m = new Float32Array([x[0],x[1],x[2],0, y[0],y[1],y[2],0, z[0],z[1],z[2],0, a[0],a[1],a[2],1]);
        const g0 = cylinder(r, 2, 0, L, 0, 0, 0, 16);
        const P = new Float32Array(g0.positions.length), N = new Float32Array(g0.normals.length);
        for (let i = 0; i < P.length; i += 3) { const p = apply(m, [g0.positions[i], g0.positions[i+1], g0.positions[i+2]]); P[i]=p[0];P[i+1]=p[1];P[i+2]=p[2]; const n = [g0.normals[i], g0.normals[i+1], g0.normals[i+2]]; const nn = [m[0]*n[0]+m[4]*n[1]+m[8]*n[2], m[1]*n[0]+m[5]*n[1]+m[9]*n[2], m[2]*n[0]+m[6]*n[1]+m[10]*n[2]]; N[i]=nn[0];N[i+1]=nn[1];N[i+2]=nn[2]; }
        return { positions: P, normals: N };
      };
      let geom;
      if (len > 1) {
        const start = [tcp[0] - zt[0]*nozzle, tcp[1] - zt[1]*nozzle, tcp[2] - zt[2]*nozzle];
        const parts = [segAlong([0,0,0], start, wr*0.18), segAlong(start, tcp, wr*0.07)].filter(Boolean);
        const n = parts.reduce((a, g) => a + g.positions.length, 0);
        const P = new Float32Array(n), N = new Float32Array(n); let o = 0;
        for (const g of parts) { P.set(g.positions, o); N.set(g.normals, o); o += g.positions.length; }
        geom = { positions: P, normals: N };
      } else geom = cylinder(wr*0.2, 2, 0, 20, 0, 0, 0, 12);
      this.viewer.setMesh('rb_tool', geom, TOOL, I4(), { group: 'robot' });
    }

    buildReach() {
      const R = this.model, v = this.viewer;
      const n = 96, ground = [], arc = [];
      const rg = R.a1 + R.wrist_reach;
      for (let i = 0; i < n; i++) { const t0 = 2*Math.PI*i/n, t1 = 2*Math.PI*(i+1)/n; ground.push(rg*Math.cos(t0), rg*Math.sin(t0), 0, rg*Math.cos(t1), rg*Math.sin(t1), 0); }
      v.setLayer('rb_reach', new Float32Array(ground), ORANGE, { alpha: 0.5, order: 3, group: 'robot' });
      this.reachArc = arc;
    }

    setBasePlacement(robotInBase) { this.robotInBase = robotInBase; if (this.joints) this.setPose(this.joints); if (this.obstacles) this.setObstacles(this.obstacles); if (this.zones) this.setZones(this.zones); }

    setPose(q) {
      if (!this.model) return;
      this.joints = q;
      const R = this.model, v = this.viewer, B = this.robotInBase;
      const k = forward(R, q);
      const f = k.frames;
      const a1 = f[1];
      if (this.meshLinks) {
        for (const idx of Object.keys(this.meshLinks)) v.setMeshTransform('rb_link' + idx, mul(B, f[Number(idx)]));
      } else {
        v.setMeshTransform('rb_base', B);
        v.setMeshTransform('rb_plate', B);
        v.setMeshTransform('rb_column', mul(B, a1));
        v.setMeshTransform('rb_upper', mul(B, k.upper));
        v.setMeshTransform('rb_fore', mul(B, k.fore));
        v.setMeshTransform('rb_wrist', mul(B, k.w4));
        v.setMeshTransform('rb_w5', mul(B, k.w5));
        v.setMeshTransform('rb_flange', mul(B, f[7]));
      }
      v.setMeshTransform('rb_tool', mul(B, f[7]));
      if (this.toolMeshOn) { v.setMeshTransform('rb_toolmesh', mul(mul(B, f[7]), this.toolMeshFrame || I4())); v.setVisible('rb_tool', false); const M = v.meshes.get('rb_tool'); if (M) M.visible = false; }
      v.setMeshTransform('rb_reach', B);
      // reach arc of the wrist centre in the current arm plane + TCP axes
      const lines = [];
      const c = apply(mul(B, f[2]), [0, 0, 0]);
      const dir = apply(mul(B, a1), [1, 0, 0]).map((x, i) => x - apply(mul(B, a1), [0,0,0])[i]);
      const up = [B[8], B[9], B[10]];
      const rr = R.wrist_reach, n = 48;
      for (let i = 0; i < n; i++) {
        const t0 = -Math.PI/2 + Math.PI*i/n, t1 = -Math.PI/2 + Math.PI*(i+1)/n;
        for (const t of [t0, t1]) lines.push(c[0] + rr*(Math.cos(t)*dir[0] + Math.sin(t)*up[0]), c[1] + rr*(Math.cos(t)*dir[1] + Math.sin(t)*up[1]), c[2] + rr*(Math.cos(t)*dir[2] + Math.sin(t)*up[2]));
      }
      v.setLayer('rb_reach_arc', new Float32Array(lines), ORANGE, { alpha: 0.35, order: 3, group: 'robot', visible: this.reachVisible });
      this.flangeInBase = mul(B, f[7]);
      const tcpM = mul(mul(B, f[7]), this.tool);
      const o = apply(tcpM, [0,0,0]), L = Math.max(40, (R.visual && R.visual.wrist || 150) * 0.6);
      const ax = [], cols = ['var(--c-ax-x)', 'var(--c-ax-y)', 'var(--c-ax-z)'];
      [[L,0,0],[0,L,0],[0,0,L]].forEach((e, i) => { const p = apply(tcpM, e); v.setLayer('rb_tcp' + i, new Float32Array([...o, ...p]), cols[i], { order: 12, group: 'robot' }); });
      this.tcp = o;
      if (this.capsulesVisible) this.drawCapsules();
      v.setGroupVisible('robot', this.visible);
      if (!this.reachVisible) { v.setVisible('rb_reach', false); v.setVisible('rb_reach_arc', false); }
    }
    setVisible(vis) { this.visible = vis; this.viewer.setGroupVisible('robot', vis); this.viewer.setGroupVisible('toolmesh', vis); if (vis && this.toolMeshOn) { const M = this.viewer.meshes.get('rb_tool'); if (M) M.visible = false; } if (vis && !this.reachVisible) { this.viewer.setVisible('rb_reach', false); this.viewer.setVisible('rb_reach_arc', false); } }
    setReachVisible(vis) { this.reachVisible = vis; this.viewer.setVisible('rb_reach', vis && this.visible); this.viewer.setVisible('rb_reach_arc', vis && this.visible); }
  }
  /** obstacles: list from the server (robot frame); drawn in BASE coords via robotInBase */
  /** safety zones: translucent oriented boxes with edges, green = allowed, red = forbidden (robot frame) */
  RobotView.prototype.setZones = function (zones) {
    const v = this.viewer;
    v.removeGroup('zones');
    this.zones = zones || [];
    (zones || []).forEach((z, i) => {
      const R = z.rot;
      const base = new Float32Array([R[0],R[3],R[6],0, R[1],R[4],R[7],0, R[2],R[5],R[8],0, z.center[0],z.center[1],z.center[2],1]);
      const M = mul(this.robotInBase, base);
      const color = z.mode === 'allowed' ? 'var(--ok)' : 'var(--bad)';
      v.setMesh('zn_' + i, box(z.half[0]*2, z.half[1]*2, z.half[2]*2, 0, 0, 0), color, M, { group: 'zones', alpha: 0.08 });
      const h = z.half, E = [];
      const c = [[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],[-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]].map(s => [s[0]*h[0], s[1]*h[1], s[2]*h[2]]);
      const edges = [[0,1],[1,2],[2,3],[3,0],[4,5],[5,6],[6,7],[7,4],[0,4],[1,5],[2,6],[3,7]];
      for (const [a, b] of edges) E.push(...apply(M, c[a]), ...apply(M, c[b]));
      v.setLayer('zn_edge' + i, new Float32Array(E), color, { order: 13, group: 'zones' });
    });
  };

  RobotView.prototype.setObstacles = function (obs, meshLoader) {
    const v = this.viewer;
    v.removeGroup('obstacles');
    this.obstacles = obs || [];
    if (meshLoader) this.meshLoader = meshLoader;
    const meshGroups = {};
    (obs || []).forEach((o, i) => {
      if (o.mesh_id && o.mesh_frame) {
        if (!meshGroups[o.group || o.name]) meshGroups[o.group || o.name] = o;
        return;  // primitives of an imported mesh are drawn via the mesh itself (capsules toggle shows them)
      }
      let geom, model;
      const R = o.rot; // row-major 3x3
      const base = new Float32Array([R[0],R[3],R[6],0, R[1],R[4],R[7],0, R[2],R[5],R[8],0, o.center[0],o.center[1],o.center[2],1]);
      if (o.kind === 'box') geom = box(o.half[0]*2, o.half[1]*2, o.half[2]*2, 0, 0, 0);
      else if (o.kind === 'cylinder') geom = cylinder(o.radius, 2, -o.height/2, o.height/2, 0, 0, 0, 40);
      else if (o.kind === 'sphere') geom = sphere(o.radius);
      else if (o.kind === 'floor') geom = box(8000, 8000, 2, 0, 0, -1);
      else return;
      const color = o.workpiece ? 'var(--c-workpiece)' : (o.kind === 'floor' ? 'var(--brand-grey)' : 'var(--c-obstacle)');
      v.setMesh('ob_' + i, geom, color, mul(this.robotInBase, base), { group: 'obstacles', alpha: o.kind === 'floor' ? 0.25 : 0.42 });
      // outline edges for boxes
      if (o.kind === 'box') {
        const h = o.half, E = [];
        const c = [[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],[-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]].map(s => [s[0]*h[0], s[1]*h[1], s[2]*h[2]]);
        const edges = [[0,1],[1,2],[2,3],[3,0],[4,5],[5,6],[6,7],[7,4],[0,4],[1,5],[2,6],[3,7]];
        const M = mul(this.robotInBase, base);
        for (const [a, b] of edges) E.push(...apply(M, c[a]), ...apply(M, c[b]));
        v.setLayer('obe_' + i, new Float32Array(E), o.workpiece ? 'var(--c-workpiece)' : 'var(--c-obstacle)', { alpha: 0.9, order: 4, group: 'obstacles' });
      }
    });
    // imported meshes: fetch geometry and place with mesh_frame (robot frame, row-major 4x4)
    const self = this;
    Object.values(meshGroups).forEach((o, k) => {
      if (!self.meshLoader) return;
      const units = (o.raw && o.raw.match(/\s(mm|cm|m|in)(\s+workpiece)?\s*$/i) || [,'mm'])[1];
      self.meshLoader(o.mesh_id, units).then(g => {
        if (!g || g.error || !self.obstacles.includes(o)) return;
        const geom = { positions: decodeF32(g.positions), normals: decodeF32(g.normals) };
        const M = mul(self.robotInBase, fromRowMajor(o.mesh_frame));
        v.setMesh('obm_' + (o.group || o.name), geom, o.workpiece ? 'var(--c-workpiece)' : 'var(--c-obstacle)', M, { group: 'obstacles', alpha: 0.85 });
        v.setGroupVisible('obstacles', self.obstaclesVisible !== false);
      }).catch(() => {});
    });
    v.setGroupVisible('obstacles', this.obstaclesVisible !== false);
  };
  /** imported tool mesh (flange frame, mm): replaces the procedural tool cylinder */
  RobotView.prototype.setToolMeshFrame = function (m) { this.toolMeshFrame = m || I4(); if (this.joints) this.setPose(this.joints); };
  RobotView.prototype.setToolMesh = function (g, frameMat) {
    const v = this.viewer;
    v.removeGroup('toolmesh');
    this.toolMeshOn = !!g;
    if (frameMat) this.toolMeshFrame = frameMat;
    if (g) {
      const geom = { positions: decodeF32(g.positions), normals: decodeF32(g.normals) };
      v.setMesh('rb_toolmesh', geom, TOOL, I4(), { group: 'toolmesh' });
      v.setMeshTransform('rb_tool', I4());
    }
    v.setGroupVisible('toolmesh', this.visible);
    if (this.joints) this.setPose(this.joints);
  };
  RobotView.prototype.setObstaclesVisible = function (vis) { this.obstaclesVisible = vis; this.viewer.setGroupVisible('obstacles', vis); };
  /** capsules: list {name, link, a, b, radius}; drawn as axis + end circles in the current pose */
  RobotView.prototype.setCapsules = function (caps) { this.capsules = caps || []; this.drawCapsules(); };
  RobotView.prototype.drawCapsules = function () {
    const v = this.viewer;
    v.removeGroup('capsules');
    if (!this.capsules || !this.capsules.length || !this.model || !this.joints || !this.capsulesVisible) return;
    const k = forward(this.model, this.joints), B = this.robotInBase;
    const linkM = k.frames.map(fr => mul(B, fr));
    const L = [];
    for (const c of this.capsules) {
      const M = linkM[c.link]; if (!M) continue;
      const a = apply(M, c.a), b = apply(M, c.b);
      L.push(...a, ...b);
      // three circles at each end + silhouette lines
      const d = norm([b[0]-a[0], b[1]-a[1], b[2]-a[2]]);
      const x0 = Math.abs(d[2]) < 0.9 ? [0,0,1] : [1,0,0];
      const u = norm(cross(d, x0)), w = cross(d, u);
      for (const p of [a, b]) for (let i = 0; i < 24; i++) {
        const t0 = 2*Math.PI*i/24, t1 = 2*Math.PI*(i+1)/24;
        L.push(p[0] + c.radius*(Math.cos(t0)*u[0] + Math.sin(t0)*w[0]), p[1] + c.radius*(Math.cos(t0)*u[1] + Math.sin(t0)*w[1]), p[2] + c.radius*(Math.cos(t0)*u[2] + Math.sin(t0)*w[2]));
        L.push(p[0] + c.radius*(Math.cos(t1)*u[0] + Math.sin(t1)*w[0]), p[1] + c.radius*(Math.cos(t1)*u[1] + Math.sin(t1)*w[1]), p[2] + c.radius*(Math.cos(t1)*u[2] + Math.sin(t1)*w[2]));
      }
      for (const e of [u, w, u.map(x => -x), w.map(x => -x)]) L.push(a[0]+c.radius*e[0], a[1]+c.radius*e[1], a[2]+c.radius*e[2], b[0]+c.radius*e[0], b[1]+c.radius*e[1], b[2]+c.radius*e[2]);
    }
    v.setLayer('caps', new Float32Array(L), 'var(--c-caps)', { alpha: 0.7, order: 13, group: 'capsules' });
  };
  RobotView.prototype.setCapsulesVisible = function (vis) { this.capsulesVisible = vis; this.drawCapsules(); };
  function sphere(r, seg) {
    seg = seg || 20;
    const P = [], N = [];
    for (let i = 0; i < seg; i++) for (let j = 0; j < seg * 2; j++) {
      const t0 = Math.PI*i/seg, t1 = Math.PI*(i+1)/seg, p0 = 2*Math.PI*j/(seg*2), p1 = 2*Math.PI*(j+1)/(seg*2);
      const pt = (t, p) => [Math.sin(t)*Math.cos(p), Math.sin(t)*Math.sin(p), Math.cos(t)];
      const q = [pt(t0,p0), pt(t1,p0), pt(t1,p1), pt(t0,p1)];
      for (const i2 of [0,1,2,0,2,3]) { P.push(...q[i2].map(x => x*r)); N.push(...q[i2]); }
    }
    return { positions: new Float32Array(P), normals: new Float32Array(N) };
  }
  function decodeF32(b64) { const bin = atob(b64); const bytes = new Uint8Array(bin.length); for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i); return new Float32Array(bytes.buffer); }
  function cross(a, b) { return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]; }
  function norm(a) { const l = Math.hypot(a[0], a[1], a[2]) || 1; return [a[0]/l, a[1]/l, a[2]/l]; }

  window.RobotView = RobotView;
  window.robotMath = { forward, fromRowMajor, mul, apply };
})();
