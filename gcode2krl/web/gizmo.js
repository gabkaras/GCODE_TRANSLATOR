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
  // editable fields inside a label: [key, value, writer]
  const inp = (k, val, key) => `<span class="k">${k}</span><input data-k="${key}" value="${round(val, 1)}">`;
  function labelText(h) {
    const b = h.kind === 'bed' ? bedRect() : null;
    switch (h.kind) {
      case 'bed': return h.id === 'bedc' ? `stół: środek ${inp('X', b.cx, 'tbl_cx')}${inp('Y', b.cy, 'tbl_cy')}` : `stół: wymiary ${inp('dł.', b.sx, 'tbl_sx')}${inp('szer.', b.sy, 'tbl_sy')}<span class="k">mm</span>` + (anchorSign() ? ` <small>· początek bazy w narożniku</small>` : '');
      case 'base': { const f = A.parse6('#opt_base_frame', [0, 0, 0, 0, 0, 0]);
        const sel = `<select data-k="tbl_anchor" title="gdzie na stole leży początek bazy"><option value="center"${A.tableAnchor() === 'center' ? ' selected' : ''}>w środku</option>${['mm', 'pm', 'pp', 'mp'].map(k => `<option value="${k}"${A.tableAnchor() === k ? ' selected' : ''}>narożnik ${k[0] === 'm' ? '−X' : '+X'} ${k[1] === 'm' ? '−Y' : '+Y'}</option>`).join('')}</select>`;
        return h.id === 'base' ? `baza stołu $BASE ${inp('X', f[0], 'bf_x')}${inp('Y', f[1], 'bf_y')}${inp('Z', f[2], 'bf_z')}${inp('A', f[3], 'bf_a')}<span class="k">°</span> ${sel}` : `obrót bazy ${inp('A', f[3], 'bf_a')}<span class="k">°</span>`; }
      case 'tcp': { const t = A.parse6('#opt_tool_frame', [0, 0, 0, 0, 0, 0]); return `TCP narzędzia ${inp('X', t[0], 'tf_x')}${inp('Y', t[1], 'tf_y')}${inp('Z', t[2], 'tf_z')}`; }
      default: return h.label || h.id;
    }
  }
  function commitLabelInput(el) {
    const key = el.dataset.k;
    if (key === 'tbl_anchor') { $('#tbl_anchor').value = el.value; A.applyTableAnchor(); A.updateAutoObstacles(); A.saveOptions(); A.scheduleAuto(); setTimeout(build, 50); return; }
    const val = Number(el.value);
    if (!Number.isFinite(val)) return;
    if (/^(bf|tf)_/.test(key)) {
      const prefix = key.slice(0, 2), hidden = prefix === 'bf' ? 'opt_base_frame' : 'opt_tool_frame';
      const f = A.parse6('#' + hidden, [0, 0, 0, 0, 0, 0]);
      f['xyzabc'.indexOf(key[3])] = val;
      A.setFrameFields(prefix, hidden, f); A.applyFrames();
    } else if (key.startsWith('tbl_')) {
      setNum(key, val); A.applyTableAnchor(); A.updateAutoObstacles();
    } else if (key.startsWith('zone:') || key.startsWith('obs:') || key.startsWith('model:')) {
      const [kind, idx, field] = key.split(':');
      if (kind === 'model') { const rows = A.platerRows(); if (rows[idx]) { rows[idx][field] = val; A.savePlater(rows); } }
      else { const lines = kind === 'zone' ? zoneLines() : obstacleLines(); const o = lines[idx]; if (o) { const [arr, i] = field[0] === 'c' ? [o.c, Number(field[1])] : [o.s, Number(field[1])]; arr[i] = val; (kind === 'zone' ? writeZones : writeObstacles)(lines); A.previewCell(); } }
    }
    A.saveOptions(); A.scheduleAuto();
    setTimeout(build, 50);
  }
  if (labelHost) {
    labelHost.addEventListener('change', e => { if (e.target.matches('input[data-k], select[data-k]')) commitLabelInput(e.target); });
    labelHost.addEventListener('keydown', e => { if (e.key === 'Enter' && e.target.matches('input[data-k]')) { e.target.blur(); } });
    labelHost.addEventListener('pointerdown', e => e.stopPropagation());
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
      if (!el.contains(document.activeElement)) { const html = labelText(h); if (el.dataset.html !== html) { el.innerHTML = html; el.dataset.html = html; } }
      seen.add(h.id);
    }
    for (const [id, el] of labelEls) if (!seen.has(id)) { el.remove(); labelEls.delete(id); }
    // keep labels from overlapping: push later ones down
    const placed = [];
    for (const el of [...labelEls.values()].sort((a, b) => parseFloat(a.style.top) - parseFloat(b.style.top))) {
      const w = el.offsetWidth, h = el.offsetHeight; let x = parseFloat(el.style.left), y = parseFloat(el.style.top);
      let moved = true, guard = 0;
      while (moved && guard++ < 20) {
        moved = false;
        for (const r of placed) if (x < r.x + r.w + 8 && x + w + 8 > r.x && y < r.y + r.h + 4 && y + h + 4 > r.y) { y = r.y + r.h + 6; moved = true; }
      }
      el.style.top = y + 'px';
      placed.push({ x, y, w, h });
    }
  }

  // ---- snapping
  function snapStep() { const el = $('#gizmo-snap'); return el ? Number(el.value) || 0 : 0; }
  let altHeld = false;
  window.addEventListener('keydown', e => { if (e.key === 'Alt') altHeld = true; });
  window.addEventListener('keyup', e => { if (e.key === 'Alt') altHeld = false; });
  const snap = v => { const st = snapStep(); return st > 0 && !altHeld ? Math.round(v / st) * st : v; };
  const snapAng = v => { const st = snapStep(); return st > 0 && !altHeld ? Math.round(v / 5) * 5 : v; };
  const anchorSign = () => { const a = A.tableAnchor(); return a === 'center' ? null : A.ANCHOR_SIGN[a]; };

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
      const anc = anchorSign();  // [kx, ky]: the base origin sits at the corner (-kx*sx/2, -ky*sy/2) from the centre
      const corners = [[-1, -1], [1, -1], [1, 1], [-1, 1]];
      corners.forEach((s, i) => {
        const isAnchor = anc && s[0] === -anc[0] && s[1] === -anc[1];
        if (isAnchor) return;  // that corner is the base origin: use the base handle
        push({
          id: 'bed' + i, kind: 'bed', pos: [b.cx + s[0] * b.sx / 2, b.cy + s[1] * b.sy / 2, b.z], plane: 'xy', size: 9, labelled: i === 2 || (anc && s[0] === anc[0] && s[1] === anc[1]),
          title: anc ? 'narożnik stołu: przeciągnij, aby zmienić wymiary (początek bazy zostaje w swoim narożniku)' : 'narożnik podstawy: przeciągnij, aby zmienić wymiary',
          drag(p) {
            if (anc) {
              // the anchored corner is at the base origin (0, 0): sizes measured from it
              const sx = Math.max(50, snap(Math.abs(p[0]))), sy = Math.max(50, snap(Math.abs(p[1])));
              if (s[0] === anc[0]) setNum('tbl_sx', sx);
              if (s[1] === anc[1]) setNum('tbl_sy', sy);
              A.applyTableAnchor();
            } else {
              const opp = [b.cx - s[0] * b.sx / 2, b.cy - s[1] * b.sy / 2];
              const px = snap(p[0]), py = snap(p[1]);
              const sx = Math.max(50, Math.abs(px - opp[0])), sy = Math.max(50, Math.abs(py - opp[1]));
              setNum('tbl_sx', sx); setNum('tbl_sy', sy); setNum('tbl_cx', (px + opp[0]) / 2); setNum('tbl_cy', (py + opp[1]) / 2);
            }
            liveBed();
          },
          end() { A.updateAutoObstacles(); },
        });
      });
      if (!anc) push({ id: 'bedc', kind: 'bed', pos: [b.cx, b.cy, b.z], plane: 'xy', size: 11, labelled: true, title: 'środek podstawy: przeciągnij, aby przesunąć stół',
        drag(p) { setNum('tbl_cx', snap(p[0])); setNum('tbl_cy', snap(p[1])); liveBed(); }, end() { A.updateAutoObstacles(); } });
    }
    // base of the table ($BASE): its origin and X axis in the data (base) frame; the robot stays at 0 0 0
    if (rv && rv.model) {
      push({ id: 'base', kind: 'base', pos: [0, 0, 0], plane: 'xy', size: 12, labelled: true, usesRaw: true, title: 'baza stołu: przeciągnij, aby przesunąć $BASE (X Y w układzie robota)',
        drag(p, last, raw) {
          // the mouse point in robot coordinates is invariant while the base (and the displayed
          // scene) moves under it: accumulate from the first raw hit, not incrementally
          const pr = mapply(baseMatrix(), raw || p);
          if (!this.r0) { this.r0 = pr; this.base0 = A.parse6('#opt_base_frame', [0, 0, 0, 0, 0, 0]); }
          const b = this.base0;
          A.setFrameFields('bf', 'opt_base_frame', [snap(b[0] + pr[0] - this.r0[0]), snap(b[1] + pr[1] - this.r0[1]), b[2], b[3], b[4], b[5]]);
          A.applyFrames();
        } });
      const r = Math.max(150, num('tbl_sx', 800) / 2 + 60);
      push({ id: 'baseyaw', kind: 'base', pos: [r, 0, 0], plane: 'xy', size: 9, labelled: true, title: 'obrót bazy stołu (A): przeciągnij po okręgu',
        drag(p) {
          const ang = Math.atan2(p[1], p[0]) * 180 / Math.PI;  // angle of the dragged point in the current base axes
          const cur = A.parse6('#opt_base_frame', [0, 0, 0, 0, 0, 0]);
          A.setFrameFields('bf', 'opt_base_frame', [cur[0], cur[1], cur[2], snapAng(round(((cur[3] + ang + 180) % 360 + 360) % 360 - 180, 1)), cur[4], cur[5]]);
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
          const st = Math.min(snapStep() || 1, 5);  // TCP: at most 5 mm steps
          const sn = v => (snapStep() > 0 && !altHeld) ? Math.round(v / st) * st : v;
          A.setFrameFields('tf', 'opt_tool_frame', [sn(cur[0] + df[0]), sn(cur[1] + df[1]), sn(cur[2] + df[2]), cur[3], cur[4], cur[5]]);
          A.applyFrames();
        } });
    }
    // safety zones (centre + size), from the parsed text
    const zlines = zoneLines();
    zlines.forEach((z, i) => {
      const c = toBase(z.frame, z.c);
      push({ id: 'zone' + i, kind: 'zone', pos: c, plane: 'xy', size: 10, labelled: true, label: `strefa ${z.name} <small>${z.mode === 'allowed' ? 'dozwolona' : 'zabroniona'}</small> ${inp('X', z.c[0], `zone:${i}:c0`)}${inp('Y', z.c[1], `zone:${i}:c1`)}${inp('Z', z.c[2], `zone:${i}:c2`)} ${inp('dł.', z.s[0], `zone:${i}:s0`)}${inp('szer.', z.s[1], `zone:${i}:s1`)}${inp('wys.', z.s[2], `zone:${i}:s2`)}`, title: `strefa ${z.name}: przeciągnij środek`,
        drag(p) { const q = fromBase(z.frame, p); z.c = [snap(round(q[0])), snap(round(q[1])), z.c[2]]; writeZones(zlines); A.previewCell(); } });
      const cz = [c[0] + z.s[0] / 2, c[1] + z.s[1] / 2, c[2] + z.s[2] / 2];
      push({ id: 'zonesz' + i, kind: 'zone', pos: cz, plane: 'xy', size: 8, title: `strefa ${z.name}: przeciągnij narożnik, aby zmienić wymiary`,
        drag(p) { z.s = [Math.max(20, snap(round(2 * Math.abs(p[0] - c[0])))), Math.max(20, snap(round(2 * Math.abs(p[1] - c[1])))), z.s[2]]; writeZones(zlines); A.previewCell(); }, label: 'strefa: wymiary' });
    });
    // obstacle boxes written by hand (not the auto table / material)
    const olines = obstacleLines();
    olines.forEach((o, i) => {
      const c = toBase(o.frame, o.c);
      push({ id: 'obs' + i, kind: 'obstacle', pos: c, plane: 'xy', size: 9, labelled: true, label: `przeszkoda ${o.name} ${inp('X', o.c[0], `obs:${i}:c0`)}${inp('Y', o.c[1], `obs:${i}:c1`)}${inp('Z', o.c[2], `obs:${i}:c2`)} ${inp('dł.', o.s[0], `obs:${i}:s0`)}${inp('szer.', o.s[1], `obs:${i}:s1`)}${inp('wys.', o.s[2], `obs:${i}:s2`)}`, title: `przeszkoda ${o.name}: przeciągnij, aby przesunąć`,
        drag(p) { const q = fromBase(o.frame, p); o.c = [snap(round(q[0])), snap(round(q[1])), o.c[2]]; writeObstacles(olines); A.previewCell(); } });
    });
    // models on the plater
    A.platerRows().forEach((r, i) => push({ id: 'model' + i, kind: 'model', pos: [r.x, r.y, 0], plane: 'xy', size: 11, labelled: true, label: `model ${r.name} ${inp('X', r.x, `model:${i}:x`)}${inp('Y', r.y, `model:${i}:y`)}${inp('obrót Z', r.rz, `model:${i}:rz`)}`, title: `model ${r.name}: przeciągnij po stole`,
      drag(p) { const rows = A.platerRows(); rows[i].x = snap(round(p[0])); rows[i].y = snap(round(p[1])); A.savePlater(rows); } }));
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
      if (h.usesRaw) h.drag(h.pos.slice(), h.pos.slice(), p0);  // anchor the raw hit at the press position
      A.viewer.canvas.style.cursor = 'grabbing';
      draw();
      return true;
    },
    move(e) {
      if (!drag) return;
      const p = hitPlane(e, drag.h, drag.h.pos);
      if (!p) return;
      const target = v3.add(drag.start, v3.sub(p, drag.hit0));
      drag.h.drag(target, drag.last, p);
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
