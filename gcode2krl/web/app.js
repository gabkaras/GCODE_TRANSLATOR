'use strict';
(() => {
  const $ = s => document.querySelector(s);
  const $$ = s => [...document.querySelectorAll(s)];

  const state = { gcode: '', filename: '', result: null, busy: false, currentFile: null, auto: true };
  const LAYERS = [
    { key: 'input', label: 'G-code wejściowy', color: '#7c8599', alpha: 0.55, order: 0, on: true },
    { key: 'input_rapid', label: 'G0 wejściowe', color: '#7c8599', alpha: 0.25, order: 1, on: false },
    { key: 'rapid', label: 'G0 → LIN (szybkie)', color: '#ef4444', order: 5, on: true },
    { key: 'slin', label: 'SLIN / LIN', color: '#60a5fa', order: 6, on: true },
    { key: 'scirc', label: 'SCIRC / CIRC', color: '#34d399', order: 7, on: true },
    { key: 'spl', label: 'SPL', color: '#f59e0b', order: 8, on: true },
    { key: 'arc_points', label: 'punkty pomocnicze łuków', color: '#c084fc', order: 9, on: false, mode: 'points', size: 7 },
    { key: 'bad_points', label: 'poza zasięgiem / limitem osi', color: '#ff3b3b', order: 11, on: true, mode: 'points', size: 9 },
  ];

  let viewer, robotView;
  try { viewer = new PathViewer($('#view')); robotView = new RobotView(viewer); }
  catch (e) { $('#viewer-error').textContent = 'WebGL niedostępny: ' + e.message; }
  const ROBOTS = {};
  const robotsReady = fetch('/api/robots').then(r => r.json()).then(d => {
    const sel = $('#opt_robot');
    for (const r of d.robots) { ROBOTS[r.key] = r; const o = document.createElement('option'); o.value = r.key; o.textContent = r.name; sel.appendChild(o); }
    if (!sel.value || sel.value === 'none') sel.value = d.default;
    sel.dataset.ready = '1';
    updateRobotModel();
  }).catch(() => {});

  // ----------------------------------------------------------------- options
  const OPTION_IDS = ['robot','base_frame','tool_frame','base_literal','tool_literal','robot_samples','tol','max_radius','min_radius','min_arc_points','refine_shift','spl_max_segment','corner_angle',
    'default_feed','offset','fit_arcs','use_spl','name','mode','tool','base','abc','rapid_vel','max_vel','acc','cdis',
    'round_corners','max_block_segments','max_motions_per_file','block_approx','spindle_out','coolant_out','spindle_anout',
    'spindle_max_rpm','bco','advance','halt_tool_change','dwell_ms','start'];

  function readOptions() {
    const o = {};
    for (const id of OPTION_IDS) {
      const el = document.getElementById('opt_' + id);
      if (!el) continue;
      if (el.type === 'checkbox') o[id] = el.checked;
      else if (el.value !== '') o[id] = el.value;
    }
    return o;
  }
  function saveOptions() { try { localStorage.setItem('gcode2krl.options', JSON.stringify(readOptions())); } catch (e) {} }
  function loadOptions() {
    let saved = null;
    try { saved = JSON.parse(localStorage.getItem('gcode2krl.options') || 'null'); } catch (e) {}
    if (!saved) return;
    for (const id of OPTION_IDS) {
      const el = document.getElementById('opt_' + id);
      if (!el || !(id in saved)) continue;
      if (el.type === 'checkbox') el.checked = !!saved[id]; else el.value = saved[id];
    }
  }
  function resetOptions() {
    $$('#options [data-default]').forEach(el => {
      if (el.type === 'checkbox') el.checked = el.dataset.default === 'true'; else el.value = el.dataset.default;
    });
    saveOptions();
    scheduleAuto();
  }

  // -------------------------------------------------------------------- files
  function setGcode(text, name) {
    state.gcode = text; state.filename = name;
    const lines = text.split(/\r?\n/).length;
    $('#file-name').textContent = name;
    $('#file-meta').textContent = `${lines.toLocaleString('pl-PL')} linii, ${(text.length / 1024).toFixed(1)} kB`;
    $('#drop').classList.add('has-file');
    if (!$('#opt_name').value || $('#opt_name').dataset.auto === '1') {
      $('#opt_name').value = ''; $('#opt_name').dataset.auto = '1';
    }
    state.auto = lines <= 40000;
    $('#auto').checked = state.auto;
    state.fitted = false;
    translate();
  }
  function readFile(file) {
    const r = new FileReader();
    r.onload = () => setGcode(String(r.result), file.name);
    r.readAsText(file);
  }
  const drop = $('#drop');
  drop.addEventListener('dragover', e => { e.preventDefault(); drop.classList.add('over'); });
  drop.addEventListener('dragleave', () => drop.classList.remove('over'));
  drop.addEventListener('drop', e => { e.preventDefault(); drop.classList.remove('over'); const f = e.dataTransfer.files[0]; if (f) readFile(f); });
  $('#file-input').addEventListener('change', e => { const f = e.target.files[0]; if (f) readFile(f); e.target.value = ''; });
  drop.addEventListener('click', e => { if (e.target.tagName !== 'SELECT' && e.target.tagName !== 'OPTION') $('#file-input').click(); });
  document.addEventListener('dragover', e => e.preventDefault());
  document.addEventListener('drop', e => { e.preventDefault(); const f = e.dataTransfer.files[0]; if (f) readFile(f); });

  fetch('/api/examples').then(r => r.json()).then(d => {
    const sel = $('#examples');
    for (const n of d.examples) { const o = document.createElement('option'); o.value = n; o.textContent = n; sel.appendChild(o); }
  }).catch(() => {});
  $('#examples').addEventListener('change', async e => {
    const n = e.target.value; if (!n) return;
    const d = await (await fetch('/api/example/' + encodeURIComponent(n))).json();
    setGcode(d.gcode, d.name);
    e.target.value = '';
  });
  $('#examples').addEventListener('click', e => e.stopPropagation());

  // ---------------------------------------------------------------- translate
  let autoTimer = null;
  function scheduleAuto() {
    if (!state.gcode || !$('#auto').checked) return;
    clearTimeout(autoTimer);
    autoTimer = setTimeout(translate, 350);
  }
  $$('#options input, #options select').forEach(el => {
    el.addEventListener('change', () => { if (el.id === 'opt_name') el.dataset.auto = '0'; saveOptions(); scheduleAuto(); });
    el.addEventListener('input', () => { if (el.type === 'range' || el.type === 'number') { saveOptions(); scheduleAuto(); } });
  });
  $('#opt_tol_range').addEventListener('input', e => { $('#opt_tol').value = e.target.value; saveOptions(); scheduleAuto(); });
  $('#opt_tol').addEventListener('input', e => { $('#opt_tol_range').value = e.target.value; });
  $('#btn-translate').addEventListener('click', translate);
  $('#btn-reset').addEventListener('click', resetOptions);
  document.addEventListener('keydown', e => {
    if ((e.ctrlKey || e.metaKey) && e.key === 'Enter') { translate(); e.preventDefault(); }
    if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA' || e.target.tagName === 'SELECT') return;
    if (e.key === 'f') viewer && viewer.fit();
    if (e.key === 't') viewer && viewer.viewTop();
    if (e.key === 'i') viewer && viewer.viewIso();
    if (e.key === 'c') fitCell();
  });

  async function translate() {
    if (!state.gcode || state.busy) { if (state.busy) state.pending = true; return; }
    state.busy = true; state.pending = false;
    const t0 = performance.now();
    setStatus('Tłumaczenie…', 'busy');
    $('#btn-translate').disabled = true;
    try {
      const res = await fetch('/api/translate', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ gcode: state.gcode, filename: state.filename, options: readOptions() }),
      });
      const data = await res.json();
      if (!res.ok || data.error) throw new Error(data.error || res.statusText);
      state.result = data;
      showResult(data, performance.now() - t0);
      setStatus(`Gotowe w ${((performance.now() - t0) / 1000).toFixed(2)} s`, 'ok');
    } catch (err) {
      setStatus('Błąd: ' + err.message, 'err');
    } finally {
      state.busy = false;
      $('#btn-translate').disabled = false;
      if (state.pending) translate();
    }
  }

  function setStatus(text, cls) { const s = $('#status'); s.textContent = text; s.className = 'status ' + (cls || ''); }
  const fmt = (n, d) => Number(n).toLocaleString('pl-PL', { maximumFractionDigits: d === undefined ? 0 : d, minimumFractionDigits: d || 0 });

  function decode(b64) {
    const bin = atob(b64);
    const bytes = new Uint8Array(bin.length);
    for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
    return new Float32Array(bytes.buffer);
  }

  function showResult(data, ms) {
    const s = data.stats;
    $('#results').classList.remove('hidden');
    $('#empty').classList.add('hidden');
    $('#st-in').textContent = fmt(s.input_moves);
    $('#st-in-sub').textContent = `G0 ${fmt(s.input_counts.G0)} · G1 ${fmt(s.input_counts.G1)} · G2/G3 ${fmt(s.input_counts.G2 + s.input_counts.G3)}`;
    $('#st-out').textContent = fmt(s.motions_out);
    $('#st-out-sub').textContent = `SLIN ${fmt(s.lines_out)} · SCIRC ${fmt(s.arcs_out)} · SPL ${fmt(s.spl_points)} · G0 ${fmt(s.rapids)}`;
    $('#st-ratio').textContent = s.compression ? fmt(s.compression, 1) + '×' : '–';
    $('#st-dev').textContent = fmt(s.max_dev, 4) + ' mm';
    $('#st-dev-sub').textContent = `tolerancja ${fmt(s.tol, 3)} mm`;
    $('#st-dev').className = 'stat-value ' + (s.max_dev > s.tol + 1e-9 ? 'bad' : 'good');
    $('#st-time').textContent = fmt(s.time_total, 2) + ' s';
    $('#st-time-sub').textContent = `parser ${fmt(s.time_parse, 2)} · łuki ${fmt(s.time_fit, 2)} · KRL ${fmt(s.time_write, 2)}`;

    const total = Math.max(1, s.lines_out + s.arcs_out + s.spl_points + s.rapids);
    const bar = $('#breakdown');
    bar.innerHTML = '';
    for (const [k, v, c] of [['SLIN', s.lines_out, '#60a5fa'], ['SCIRC', s.arcs_out, '#34d399'], ['SPL', s.spl_points, '#f59e0b'], ['G0', s.rapids, '#ef4444']]) {
      if (!v) continue;
      const d = document.createElement('div');
      d.style.width = (100 * v / total) + '%'; d.style.background = c; d.title = `${k}: ${v}`;
      d.textContent = (100 * v / total) > 8 ? `${k} ${v}` : '';
      bar.appendChild(d);
    }

    // warnings
    const wl = $('#warnings');
    wl.innerHTML = '';
    for (const w of data.warnings) { const li = document.createElement('li'); li.textContent = w; wl.appendChild(li); }
    $('#tab-warn-count').textContent = data.warnings.length ? `(${data.warnings.length})` : '';
    $('#warnings-empty').classList.toggle('hidden', data.warnings.length > 0);

    // files
    const sel = $('#file-select');
    sel.innerHTML = '';
    for (const name of Object.keys(data.files)) { const o = document.createElement('option'); o.value = name; o.textContent = name; sel.appendChild(o); }
    sel.value = data.main_file;
    showFile(data.main_file);
    $('#btn-zip').href = '/api/zip/' + data.token;
    $('#btn-zip').download = data.name + '.zip';

    // preview
    if (viewer) {
      const p = data.preview;
      for (const L of LAYERS) {
        const src = L.key === 'bad_points' ? (data.robot ? data.robot.bad_points : '') : p[L.key];
        const arr = decode(src);
        viewer.setLayer(L.key, arr, L.color, { alpha: L.alpha, order: L.order, mode: L.mode, size: L.size, visible: $('#layer-' + L.key).checked });
      }
      showRobot(data.robot);
      viewer.setBounds(p.bounds.min, p.bounds.max);
      if (!state.fitted) { viewer.viewIso(); state.fitted = true; } else viewer.invalidate();
      const b = p.bounds;
      $('#bounds').textContent = `X ${fmt(b.min[0], 2)} … ${fmt(b.max[0], 2)}   Y ${fmt(b.min[1], 2)} … ${fmt(b.max[1], 2)}   Z ${fmt(b.min[2], 2)} … ${fmt(b.max[2], 2)} mm`;
    }
  }

  // ----------------------------------------------------------------- KRL view
  const KW = /\b(SPLINE|ENDSPLINE|SLIN|SCIRC|SPL|LIN|CIRC|PTP|DEF|END|WAIT SEC|HALT|WITH|C_SPL|C_DIS|BAS|INTERRUPT|GLOBAL|DECL|ON|TRUE|FALSE|DEFDAT|ENDDAT)\b/g;
  function esc(t) { return t.replace(/&/g, '&amp;').replace(/</g, '&lt;'); }
  function highlight(text) {
    return text.split('\n').map(line => {
      const i = line.indexOf(';');
      const code = i >= 0 ? line.slice(0, i) : line, com = i >= 0 ? line.slice(i) : '';
      const h = esc(code).replace(KW, '<b>$1</b>').replace(/(\$[A-Z_.\[\]0-9]+)/g, '<i>$1</i>');
      return h + (com ? `<span class="c">${esc(com)}</span>` : '');
    }).join('\n');
  }
  function showFile(name) {
    const text = state.result.files[name];
    state.currentFile = name;
    const pre = $('#krl');
    const lines = text.split('\n').length;
    $('#krl-meta').textContent = `${fmt(lines)} linii · ${(text.length / 1024).toFixed(1)} kB`;
    if (lines > 20000) { pre.textContent = text; }
    else pre.innerHTML = highlight(text);
  }
  $('#file-select').addEventListener('change', e => showFile(e.target.value));
  $('#btn-copy').addEventListener('click', async () => {
    try { await navigator.clipboard.writeText(state.result.files[state.currentFile]); setStatus('Skopiowano do schowka', 'ok'); }
    catch (e) { setStatus('Kopiowanie nieudane: ' + e.message, 'err'); }
  });
  $('#btn-file').addEventListener('click', () => {
    const blob = new Blob([state.result.files[state.currentFile].replace(/\n/g, '\r\n')], { type: 'text/plain' });
    const a = document.createElement('a'); a.href = URL.createObjectURL(blob); a.download = state.currentFile; a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 2000);
  });

  // ------------------------------------------------------------------ robot
  const anim = { joints: null, n: 0, pos: 0, playing: false, last: 0, raf: null };
  function updateRobotModel() {
    const key = $('#opt_robot').value;
    const R = ROBOTS[key];
    if (!robotView) return;
    if (!R) { robotView.setVisible(false); $('#joints').classList.add('hidden'); return; }
    if (!robotView.model || robotView.model.key !== key) { robotView.setModel(R); buildJointRows(R); }
    robotView.setVisible($('#show-robot').checked);
    $('#joints').classList.remove('hidden');
    applyFrames();
  }
  function parse6(id, def) { const v = $(id).value.trim().split(/[\s,]+/).map(Number); return v.length === 6 && v.every(Number.isFinite) ? v : def; }
  function kukaFrame(f) {
    const d = Math.PI / 180, [x, y, z, a, b, c] = f;
    const ca = Math.cos(a*d), sa = Math.sin(a*d), cb = Math.cos(b*d), sb = Math.sin(b*d), cc = Math.cos(c*d), sc = Math.sin(c*d);
    // R = Rz(A) Ry(B) Rx(C), column-major
    return new Float32Array([ca*cb, sa*cb, -sb, 0,  ca*sb*sc - sa*cc, sa*sb*sc + ca*cc, cb*sc, 0,  ca*sb*cc + sa*sc, sa*sb*cc - ca*sc, cb*cc, 0,  x, y, z, 1]);
  }
  function invert(m) { // rigid transform inverse
    const r = [m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]], t = [m[12], m[13], m[14]];
    const o = new Float32Array(16);
    o[0]=r[0]; o[1]=r[3]; o[2]=r[6]; o[4]=r[1]; o[5]=r[4]; o[6]=r[7]; o[8]=r[2]; o[9]=r[5]; o[10]=r[8]; o[15]=1;
    o[12] = -(o[0]*t[0] + o[4]*t[1] + o[8]*t[2]); o[13] = -(o[1]*t[0] + o[5]*t[1] + o[9]*t[2]); o[14] = -(o[2]*t[0] + o[6]*t[1] + o[10]*t[2]);
    return o;
  }
  function applyFrames() {
    if (!robotView || !robotView.model) return;
    robotView.setTool(kukaFrame(parse6('#opt_tool_frame', [0,0,0,0,0,0])));
    robotView.setBasePlacement(invert(kukaFrame(parse6('#opt_base_frame', [0,0,0,0,0,0]))));
  }
  function buildJointRows(R) {
    const box = $('#joint-rows');
    box.innerHTML = '';
    R.limits.forEach((lim, i) => {
      const row = document.createElement('div');
      row.className = 'jrow';
      row.innerHTML = `<span>A${i+1}</span><input type="range" min="${lim[0]}" max="${lim[1]}" step="0.1" value="${R.home[i]}" data-axis="${i}"><span class="jv" id="jv${i}">${R.home[i].toFixed(1)}°</span>`;
      box.appendChild(row);
      row.querySelector('input').addEventListener('input', e => { stopAnim(); const q = currentSliders(); setPose(q, 'ręcznie'); });
    });
  }
  function currentSliders() { return $$('#joint-rows input').map(el => Number(el.value)); }
  function setPose(q, mode) {
    if (!robotView || !robotView.model) return;
    robotView.setPose(q);
    const R = robotView.model;
    q.forEach((v, i) => {
      const el = $('#jv' + i); if (!el) return;
      el.textContent = v.toFixed(1) + '°';
      el.classList.toggle('viol', v < R.limits[i][0] - 1e-6 || v > R.limits[i][1] + 1e-6);
      const sl = $$('#joint-rows input')[i]; if (sl && document.activeElement !== sl) sl.value = v;
    });
    if (mode) $('#joints-mode').textContent = mode;
    if (robotView.tcp) $('#tcp-pos').textContent = `TCP ${robotView.tcp.map(x => x.toFixed(1)).join('  ')}`;
  }
  function showRobot(rep) {
    const bar = $('#robot-bar');
    if (!rep || !robotView) { bar.classList.add('hidden'); anim.joints = null; $('#anim').max = 0; return; }
    bar.classList.remove('hidden');
    const R = ROBOTS[rep.robot];
    $('#robot-name').textContent = R ? R.name : rep.robot;
    const reach = $('#robot-reach'), lim = $('#robot-limit');
    reach.textContent = rep.n_unreachable ? `${rep.n_unreachable} / ${rep.n_samples} próbek poza zasięgiem` : `zasięg OK (${rep.n_samples} próbek)`;
    reach.className = 'rb-item ' + (rep.n_unreachable ? 'bad' : 'ok');
    lim.textContent = rep.n_limit ? `${rep.n_limit} próbek poza limitami osi` : 'limity osi OK';
    lim.className = 'rb-item ' + (rep.n_limit ? 'bad' : 'ok');
    $('#robot-axes').innerHTML = rep.joint_min.map((lo, i) => lo === null ? '' : `<span class="${rep.axis_violations[i] ? 'viol' : ''}">A${i+1} <b>${lo.toFixed(0)}…${rep.joint_max[i].toFixed(0)}°</b></span>`).join('');
    // animation data
    anim.joints = decode(rep.joints); anim.ok = Uint8Array.from(atob(rep.ok), c => c.charCodeAt(0)); anim.n = anim.joints.length / 6; anim.pos = 0;
    $('#anim').max = Math.max(0, anim.n - 1); $('#anim').value = 0;
    updateRobotModel();
    if (anim.n) poseAt(0);
  }
  function poseAt(x) {
    if (!anim.joints || !anim.n) return;
    x = Math.max(0, Math.min(anim.n - 1, x));
    let i = Math.floor(x), f = x - i;
    if (i >= anim.n - 1) { i = anim.n - 1; f = 0; }
    const q = [];
    const ok0 = anim.ok[i], ok1 = i + 1 < anim.n ? anim.ok[i + 1] : ok0;
    for (let k = 0; k < 6; k++) {
      const a = anim.joints[i*6 + k], b = i + 1 < anim.n ? anim.joints[(i+1)*6 + k] : a;
      q.push(ok0 && ok1 ? a + (b - a) * f : (ok0 ? a : b));
    }
    setPose(q, `ścieżka ${Math.round(x) + 1} / ${anim.n}` + (!ok0 ? ' · poza zasięgiem' : ''));
    $('#anim').value = x;
  }
  function tick(ts) {
    if (!anim.playing) return;
    const dt = Math.min(0.1, (ts - anim.last) / 1000); anim.last = ts;
    anim.pos += dt * Number($('#anim-speed').value);
    if (anim.pos >= anim.n - 1) anim.pos = 0;
    poseAt(anim.pos);
    anim.raf = requestAnimationFrame(tick);
  }
  function stopAnim() { anim.playing = false; $('#btn-play').textContent = '▶'; if (anim.raf) cancelAnimationFrame(anim.raf); }
  function startAnim() { if (!anim.n) return; anim.playing = true; anim.last = performance.now(); $('#btn-play').textContent = '❚❚'; anim.raf = requestAnimationFrame(tick); }
  $('#btn-play').addEventListener('click', () => anim.playing ? stopAnim() : startAnim());
  $('#anim').addEventListener('input', e => { stopAnim(); anim.pos = Number(e.target.value); poseAt(anim.pos); });
  $('#show-robot').addEventListener('change', e => robotView && robotView.setVisible(e.target.checked));
  $('#show-reach').addEventListener('change', e => robotView && robotView.setReachVisible(e.target.checked));
  $('#opt_robot').addEventListener('change', updateRobotModel);
  $('#opt_base_frame').addEventListener('input', applyFrames);
  $('#opt_tool_frame').addEventListener('input', applyFrames);
  document.addEventListener('keydown', e => {
    if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA' || e.target.tagName === 'SELECT') return;
    if (e.key === ' ') { e.preventDefault(); anim.playing ? stopAnim() : startAnim(); }
  });

  // ------------------------------------------------------------------- tabs
  $$('.tab').forEach(t => t.addEventListener('click', () => {
    $$('.tab').forEach(x => x.classList.toggle('active', x === t));
    $$('.pane').forEach(p => p.classList.toggle('active', p.id === 'pane-' + t.dataset.tab));
    if (t.dataset.tab === 'view' && viewer) viewer.invalidate();
  }));

  // ----------------------------------------------------------------- viewer UI
  const legend = $('#legend');
  for (const L of LAYERS) {
    const lab = document.createElement('label');
    lab.innerHTML = `<input type="checkbox" id="layer-${L.key}" ${L.on ? 'checked' : ''}><span class="sw" style="background:${L.color}"></span>${L.label}`;
    legend.appendChild(lab);
    lab.querySelector('input').addEventListener('change', e => viewer && viewer.setVisible(L.key, e.target.checked));
  }
  $('#btn-fit').addEventListener('click', () => viewer.fit());
  function fitCell() {
    if (!viewer) return;
    if (!robotView || !robotView.model || !robotView.visible) { viewer.fit(); return; }
    const R = robotView.model, B = robotView.robotInBase;
    const o = robotMath.apply(B, [0, 0, 0]);
    const r = R.a1 + R.wrist_reach + R.d6;
    const pb = viewer.bounds;
    const min = [Math.min(pb.min[0], o[0] - r), Math.min(pb.min[1], o[1] - r), Math.min(pb.min[2], o[2])];
    const max = [Math.max(pb.max[0], o[0] + r), Math.max(pb.max[1], o[1] + r), Math.max(pb.max[2], o[2] + R.d1 + R.wrist_reach)];
    const saved = viewer.bounds;
    viewer.bounds = { min, max };
    viewer.fit();
    viewer.bounds = saved;
  }
  $('#btn-cell').addEventListener('click', fitCell);
  $('#btn-top').addEventListener('click', () => viewer.viewTop());
  $('#btn-iso').addEventListener('click', () => viewer.viewIso());
  $('#btn-front').addEventListener('click', () => viewer.viewFront());
  $('#btn-ortho').addEventListener('click', e => { e.target.classList.toggle('active', viewer.toggleOrtho()); });
  $('#btn-png').addEventListener('click', () => {
    viewer.render();
    const a = document.createElement('a'); a.href = viewer.canvas.toDataURL('image/png'); a.download = (state.filename || 'path') + '.png'; a.click();
  });
  if (viewer) viewer.onChange = v => { $('#cam-info').textContent = `siatka ${v.gridStep} mm`; };

  $('#auto').addEventListener('change', e => { state.auto = e.target.checked; });

  loadOptions();
  $('#opt_tol_range').value = $('#opt_tol').value;
  robotsReady.then(() => { loadOptions(); updateRobotModel(); });
  fetch('/api/version').then(r => r.json()).then(d => { $('#version').textContent = 'v' + d.version; }).catch(() => {});
})();
