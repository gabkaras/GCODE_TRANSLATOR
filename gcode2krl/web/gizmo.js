'use strict';
// Drag handles in the 3D view: print bed corners / centre, robot base position and
// yaw, TCP, safety zones (centre, size), obstacle boxes, models on the plater.
// Values go straight into the form fields of the cell setup; the translation is
// re-run when the drag ends.
(() => {
  const A = window.GCODE2KRL;
  if (!A) return;
  const $ = s => document.querySelector(s);
  const v3 = {
    sub: (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]],
    add: (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]],
    scale: (a, k) => [a[0] * k, a[1] * k, a[2] * k],
    dot: (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2],
    len: a => Math.hypot(a[0], a[1], a[2]),
  };
  // column-major 4x4 helpers (same layout as robot.js)
  const mcol = (m, c) => [m[c * 4], m[c * 4 + 1], m[c * 4 + 2]];
  const mapply = (m, p) => [m[0] * p[0] + m[4] * p[1] + m[8] * p[2] + m[12], m[1] * p[0] + m[5] * p[1] + m[9] * p[2] + m[13], m[2] * p[0] + m[6] * p[1] + m[10] * p[2] + m[14]];
  const mrot = (m, p) => [m[0] * p[0] + m[4] * p[1] + m[8] * p[2], m[1] * p[0] + m[5] * p[1] + m[9] * p[2], m[2] * p[0] + m[6] * p[1] + m[10] * p[2]];
  const mrotT = (m, p) => [m[0] * p[0] + m[1] * p[1] + m[2] * p[2], m[4] * p[0] + m[5] * p[1] + m[6] * p[2], m[8] * p[0] + m[9] * p[1] + m[10] * p[2]];
  const round = (x, d = 1) => Math.round(x * 10 ** d) / 10 ** d;

  const COLORS = { bed: 'var(--c-sl-adh)', base: 'var(--accent)', tcp: 'var(--c-ax-z)', zone: 'var(--c-zone)', obstacle: 'var(--c-obstacle)', model: 'var(--c-sl-wall)' };
  let handles = [];
  let enabled = true;
  let hover = null;
  let drag = null;

  // ---- HTML labels over the canvas (the WebGL view has no text)
  const labelHost = $('#gizmo-labels');
  const labelEls = new Map();
  function labelText(h) {
    const b = h.kind === 'bed' ? bedRect() : null;
    switch (h.kind) {
      case 'bed': return h.id === 'bedc' ? `stół: środek <small>${round(b.cx, 0)}, ${round(b.cy, 0)}</small>` : `stół: narożnik <small>${round(b.sx, 0)} × ${round(b.sy, 0)} mm</small>`;
      case 'base': { const f = A.parse6('#opt_base_frame', [0, 0, 0, 0, 0, 0]); return h.id === 'robot' ? `baza robota $BASE <small>X ${round(f[0], 0)} Y ${round(f[1], 0)} Z ${round(f[2], 0)}</small>` : `obrót bazy <small>A ${round(f[3], 1)}°</small>`; }
      case 'tcp': { const t = A.parse6('#opt_tool_frame', [0, 0, 0, 0, 0, 0]); return `TCP narzędzia <small>X ${round(t[0], 1)} Y ${round(t[1], 1)} Z ${round(t[2], 1)}</small>`; }
      default: return h.label || h.id;
    }
  }
  function updateLabels() {
    if (!labelHost) return;
    const v = A.viewer; if (!v) return;
    const seen = new Set();
    if (enabled) for (const h of handles) {
      if (!h.labelled && !(h === hover) && !(drag && drag.h === h)) continue;
      const sp = v.project(h.pos);
      if (!sp || sp[0] < -50 || sp[1] < -20 || sp[0] > v.canvas.clientWidth + 50 || sp[1] > v.canvas.clientHeight + 20) continue;
      let el = labelEls.get(h.id);
      if (!el) { el = document.createElement('div'); el.className = 'gz-label'; labelHost.appendChild(el); labelEls.set(h.id, el); }
      el.style.left = sp[0] + 'px'; el.style.top = sp[1] + 'px';
      el.style.borderLeftColor = COLORS[h.kind];
      el.classList.toggle('hot', h === hover || !!(drag && drag.h === h));
      el.innerHTML = labelText(h);
      seen.add(h.id);
    }
    for (const [id, el] of labelEls) if (!seen.has(id)) { el.remove(); labelEls.delete(id); }
  }

  function num(id, def) { const el = $('#' + id); const v = el ? Number(el.value) : NaN; return Number.isFinite(v) && el.value !== '' ? v : def; }
  function setNum(id, val) { const el = $('#' + id); if (el) el.value = String(round(val, 1)); }

  function pathCenter() {
    const r = A.state.result; const b = r && r.preview && r.preview.bounds;
    return b ? [(b.min[0] + b.max[0]) / 2, (b.min[1] + b.max[1]) / 2] : [0, 0];
  }
  function bedRect() {
    const sx = num('tbl_sx', 800), sy = num('tbl_sy', 800);
    const pc = pathCenter();
    const cx = num('tbl_cx', pc[0]), cy = num('tbl_cy', pc[1]);
    const z = -num('mat_t', 20);
    return { cx, cy, sx, sy, z };
  }
  function baseMatrix() { return A.kukaFrame(A.parse6('#opt_base_frame', [0, 0, 0, 0, 0, 0])); }  // robot -> base placement (base in robot frame)
  function robotInBase() { return A.invert(baseMatrix()); }

  // ------------------------------------------------------------ handle set
  function build() {
    const H = [];
    const rv = A.robotView;
    const push = h => H.push(h);
    // print bed (auto table) corners + centre
    if ($('#auto-table') && $('#auto-table').checked) {
      const b = bedRect();
      const corners = [[-1, -1], [1, -1], [1, 1], [-1, 1]];
      corners.forEach((s, i) => push({
        id: 'bed' + i, kind: 'bed', pos: [b.cx + s[0] * b.sx / 2, b.cy + s[1] * b.sy / 2, b.z], plane: 'xy', size: 9, labelled: i === 2,
        title: 'narożnik podstawy: przeciągnij, aby zmienić wymiary',
        drag(p) {
          const opp = [b.cx - s[0] * b.sx / 2, b.cy - s[1] * b.sy / 2];
          const sx = Math.max(50, Math.abs(p[0] - opp[0])), sy = Math.max(50, Math.abs(p[1] - opp[1]));
          setNum('tbl_sx', sx); setNum('tbl_sy', sy); setNum('tbl_cx', (p[0] + opp[0]) / 2); setNum('tbl_cy', (p[1] + opp[1]) / 2);
          liveBed();
        },
        end() { A.updateAutoObstacles(); },
      }));
      push({ id: 'bedc', kind: 'bed', pos: [b.cx, b.cy, b.z], plane: 'xy', size: 11, labelled: true, title: 'środek podstawy: przeciągnij, aby przesunąć stół',
        drag(p) { setNum('tbl_cx', p[0]); setNum('tbl_cy', p[1]); liveBed(); }, end() { A.updateAutoObstacles(); } });
    }
    // robot base position + yaw (the scene is the base frame: moving the robot = moving $BASE)
    if (rv && rv.model) {
      const rib = robotInBase();
      const o = mcol(rib, 3);
      const xdir = mcol(rib, 0);
      push({ id: 'robot', kind: 'base', pos: [o[0], o[1], o[2]], plane: 'xy', size: 12, labelled: true, title: 'podstawa robota: przeciągnij, aby przesunąć bazę ($BASE X Y)',
        drag(p, start) {
          const d = v3.sub(p, start);
          const B = baseMatrix();
          const cur = A.parse6('#opt_base_frame', [0, 0, 0, 0, 0, 0]);
          const dr = mrot(B, d);  // delta expressed in the robot frame
          A.setFrameFields('bf', 'opt_base_frame', [cur[0] - dr[0], cur[1] - dr[1], cur[2], cur[3], cur[4], cur[5]]);
          A.applyFrames();
        } });
      const r = 450;
      push({ id: 'robotyaw', kind: 'base', pos: [o[0] + xdir[0] * r, o[1] + xdir[1] * r, o[2]], plane: 'xy', size: 9, labelled: true, title: 'obrót bazy (A): przeciągnij po okręgu',
        drag(p) {
          const ang = Math.atan2(p[1] - o[1], p[0] - o[0]) * 180 / Math.PI;
          const cur = A.parse6('#opt_base_frame', [0, 0, 0, 0, 0, 0]);
          A.setFrameFields('bf', 'opt_base_frame', [cur[0], cur[1], cur[2], round(-ang, 1), cur[4], cur[5]]);
          A.applyFrames();
        } });
    }
    // TCP (tool frame X Y Z relative to the flange), dragged in the screen plane
    if (rv && rv.tcp && rv.flangeInBase) {
      push({ id: 'tcp', kind: 'tcp', pos: rv.tcp.slice(), plane: 'screen', size: 11, labelled: true, title: 'TCP: przeciągnij, aby zmienić X Y Z narzędzia (względem kołnierza)',
        drag(p, start) {
          const d = v3.sub(p, start);
          const df = mrotT(rv.flangeInBase, d);
          const cur = A.parse6('#opt_tool_frame', [0, 0, 150, 0, 0, 0]);
          A.setFrameFields('tf', 'opt_tool_frame', [cur[0] + df[0], cur[1] + df[1], cur[2] + df[2], cur[3], cur[4], cur[5]]);
          A.applyFrames();
        } });
    }
    // safety zones (centre + size), from the parsed text
    const zlines = zoneLines();
    zlines.forEach((z, i) => {
      const c = toBase(z.frame, z.c);
      push({ id: 'zone' + i, kind: 'zone', pos: c, plane: 'xy', size: 10, labelled: true, label: `strefa ${z.name} <small>${z.mode === 'allowed' ? 'dozwolona' : 'zabroniona'} · ${z.s[0]} × ${z.s[1]} × ${z.s[2]}</small>`, title: `strefa ${z.name}: przeciągnij środek`,
        drag(p) { const q = fromBase(z.frame, p); z.c = [round(q[0]), round(q[1]), z.c[2]]; writeZones(zlines); A.previewCell(); } });
      const cz = [c[0] + z.s[0] / 2, c[1] + z.s[1] / 2, c[2] + z.s[2] / 2];
      push({ id: 'zonesz' + i, kind: 'zone', pos: cz, plane: 'xy', size: 8, title: `strefa ${z.name}: przeciągnij narożnik, aby zmienić wymiary`,
        drag(p) { z.s = [Math.max(20, round(2 * Math.abs(p[0] - c[0]))), Math.max(20, round(2 * Math.abs(p[1] - c[1]))), z.s[2]]; writeZones(zlines); A.previewCell(); }, label: 'strefa: wymiary' });
    });
    // obstacle boxes written by hand (not the auto table / material)
    const olines = obstacleLines();
    olines.forEach((o, i) => {
      const c = toBase(o.frame, o.c);
      push({ id: 'obs' + i, kind: 'obstacle', pos: c, plane: 'xy', size: 9, labelled: true, label: `przeszkoda ${o.name} <small>${o.s.join(' × ')}</small>`, title: `przeszkoda ${o.name}: przeciągnij, aby przesunąć`,
        drag(p) { const q = fromBase(o.frame, p); o.c = [round(q[0]), round(q[1]), o.c[2]]; writeObstacles(olines); A.previewCell(); } });
    });
    // models on the plater
    A.platerRows().forEach((r, i) => push({ id: 'model' + i, kind: 'model', pos: [r.x, r.y, 0], plane: 'xy', size: 11, labelled: true, label: `model ${r.name} <small>X ${round(r.x, 0)} Y ${round(r.y, 0)}</small>`, title: `model ${r.name}: przeciągnij po stole`,
      drag(p) { const rows = A.platerRows(); rows[i].x = round(p[0]); rows[i].y = round(p[1]); A.savePlater(rows); } }));
    handles = H;
    draw();
  }

  // zones / obstacles text <-> structures
  function zoneLines() {
    const out = [];
    ($('#opt_zones').value || '').split('\n').forEach((raw, idx) => {
      const line = raw.split('#')[0].trim(); if (!line) return;
      const p = line.split(/[\s,]+/); if (p[0].toLowerCase() === 'zone') p.shift();
      if (p.length < 10) return;
      out.push({ idx, name: p[0], mode: p[1], scope: p[2], frame: p[3].toLowerCase(), c: p.slice(4, 7).map(Number), s: p.slice(7, 10).map(Number), abc: p.slice(10, 13).map(Number), raw });
    });
    return out;
  }
  function writeZones(zl) {
    const lines = $('#opt_zones').value.split('\n');
    for (const z of zl) lines[z.idx] = `zone ${z.name} ${z.mode} ${z.scope} ${z.frame} ${z.c.join(' ')} ${z.s.join(' ')}` + (z.abc.length === 3 && z.abc.some(x => x) ? ' ' + z.abc.join(' ') : '');
    $('#opt_zones').value = lines.join('\n');
  }
  function obstacleLines() {
    const out = [];
    ($('#opt_obstacles').value || '').split('\n').forEach((raw, idx) => {
      if (/auto/i.test(raw)) return;
      const line = raw.split('#')[0].trim(); if (!line) return;
      const p = line.split(/[\s,]+/);
      if (p[0].toLowerCase() !== 'box' || p.length < 9) return;
      const wp = p[p.length - 1].toLowerCase() === 'workpiece';
      const nums = p.slice(3, wp ? -1 : undefined).map(Number);
      out.push({ idx, name: p[1], frame: p[2].toLowerCase(), c: nums.slice(0, 3), s: nums.slice(3, 6), abc: nums.slice(6, 9), wp });
    });
    return out;
  }
  function writeObstacles(ol) {
    const lines = $('#opt_obstacles').value.split('\n');
    for (const o of ol) lines[o.idx] = `box ${o.name} ${o.frame} ${o.c.join(' ')} ${o.s.join(' ')}` + (o.abc.length === 3 && o.abc.some(x => x) ? ' ' + o.abc.join(' ') : '') + (o.wp ? ' workpiece' : '');
    $('#opt_obstacles').value = lines.join('\n');
  }
  function toBase(frame, c) { return frame === 'robot' ? mapply(robotInBase(), c) : c.slice(); }
  function fromBase(frame, p) { return frame === 'robot' ? mapply(baseMatrix(), p) : p.slice(); }

  // ------------------------------------------------------------ drawing
  function liveBed() {
    const v = A.viewer; if (!v) return;
    A.updateAutoObstacles();  // rewrites the auto table / material lines and redraws them
    const b = bedRect();
    const c = [[-1, -1], [1, -1], [1, 1], [-1, 1]].map(s => [b.cx + s[0] * b.sx / 2, b.cy + s[1] * b.sy / 2, b.z]);
    const E = [];
    for (let i = 0; i < 4; i++) E.push(...c[i], ...c[(i + 1) % 4]);
    v.setLayer('gz_bed', new Float32Array(E), COLORS.bed, { order: 30, group: 'gizmo', visible: enabled });
    for (const h of handles) if (h.kind === 'bed') { const i = Number(h.id.replace('bed', '')); h.pos = Number.isFinite(i) ? c[i] : [b.cx, b.cy, b.z]; }
    draw();
  }
  function draw() {
    const v = A.viewer; if (!v) return;
    const groups = {};
    for (const h of handles) { const k = h.kind + (h === hover || (drag && drag.h === h) ? '_hot' : ''); (groups[k] = groups[k] || []).push(...h.pos); }
    for (const k of Object.keys(COLORS)) {
      for (const suf of ['', '_hot']) {
        const arr = groups[k + suf] || [];
        v.setLayer('gz_' + k + suf, new Float32Array(arr), COLORS[k], { order: 31, group: 'gizmo', mode: 'points', size: suf ? 16 : 10, visible: enabled });
      }
    }
    v.invalidate();
    updateLabels();
  }

  // ------------------------------------------------------------ picking / dragging
  function pick(e) {
    const v = A.viewer; if (!v || !enabled) return null;
    const rect = v.canvas.getBoundingClientRect();
    const mx = e.clientX - rect.left, my = e.clientY - rect.top;
    let best = null, bd = 14;
    for (const h of handles) {
      const s = v.project(h.pos); if (!s) continue;
      const d = Math.hypot(s[0] - mx, s[1] - my);
      if (d < bd) { bd = d; best = h; }
    }
    return best;
  }
  function hitPlane(e, h, origin) {
    const v = A.viewer;
    const rect = v.canvas.getBoundingClientRect();
    const ray = v.ray(e.clientX - rect.left, e.clientY - rect.top);
    let n;
    if (h.plane === 'xy') n = [0, 0, 1];
    else { const eye = v.eye(); n = v3.sub(eye, origin); const l = v3.len(n); n = l > 0 ? v3.scale(n, 1 / l) : [0, 0, 1]; }
    const denom = v3.dot(ray.dir, n);
    if (Math.abs(denom) < 1e-9) return null;
    const t = v3.dot(v3.sub(origin, ray.origin), n) / denom;
    if (t < 0) return null;
    return v3.add(ray.origin, v3.scale(ray.dir, t));
  }
  const gizmo = {
    down(e) {
      const h = pick(e); if (!h) return false;
      const p0 = hitPlane(e, h, h.pos); if (!p0) return false;
      drag = { h, start: h.pos.slice(), hit0: p0, last: h.pos.slice() };
      A.viewer.canvas.style.cursor = 'grabbing';
      draw();
      return true;
    },
    move(e) {
      if (!drag) return;
      const p = hitPlane(e, drag.h, drag.h.pos);
      if (!p) return;
      const target = v3.add(drag.start, v3.sub(p, drag.hit0));
      drag.h.drag(target, drag.last);
      drag.last = target;
      drag.h.pos = target;
      draw();
    },
    up() {
      if (!drag) return;
      const h = drag.h; drag = null;
      A.viewer.canvas.style.cursor = '';
      if (h.end) h.end();
      A.saveOptions(); A.scheduleAuto();
      setTimeout(build, 50);
    },
    hover(e) {
      const h = pick(e);
      if (h !== hover) { hover = h; A.viewer.canvas.style.cursor = h ? 'grab' : ''; A.viewer.canvas.title = h ? h.title : ''; draw(); }
    },
  };

  function attach() {
    const v = A.viewer; if (!v) { setTimeout(attach, 200); return; }
    v.gizmo = gizmo;
    if (v.canvas.parentElement) v.canvas.parentElement.style.position = 'relative';
    v.onRender.push(updateLabels);
    build();
    setInterval(() => { if (!drag && enabled) build(); }, 800);
  }
  attach();
  const btn = $('#chk-gizmo');
  if (btn) btn.addEventListener('change', () => { enabled = btn.checked; if (A.viewer) A.viewer.setGroupVisible('gizmo', enabled); draw(); });
  window.GIZMO = { refresh: build, setEnabled(on) { enabled = on; draw(); } };
})();
