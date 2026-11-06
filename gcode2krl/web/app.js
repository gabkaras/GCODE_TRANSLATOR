'use strict';
(() => {
  const $ = s => document.querySelector(s);
  const $$ = s => [...document.querySelectorAll(s)];

  const state = { gcode: '', filename: '', result: null, busy: false, currentFile: null, auto: true };
  const LAYERS = [
    { key: 'input', label: 'G-code wejściowy', color: 'var(--c-input)', alpha: 0.55, order: 0, on: true },
    { key: 'input_rapid', label: 'G0 wejściowe', color: 'var(--c-input)', alpha: 0.25, order: 1, on: false },
    { key: 'rapid', label: 'G0 → LIN (szybkie)', color: 'var(--c-rapid)', order: 5, on: true },
    { key: 'slin', label: 'SLIN / LIN', color: 'var(--c-slin)', order: 6, on: true },
    { key: 'scirc', label: 'SCIRC / CIRC', color: 'var(--c-scirc)', order: 7, on: true },
    { key: 'spl', label: 'SPL', color: 'var(--c-spl)', order: 8, on: true },
    { key: 'arc_points', label: 'punkty pomocnicze łuków', color: 'var(--c-aux)', order: 9, on: false, mode: 'points', size: 7 },
    { key: 'bad_points', label: 'poza zasięgiem / limitem osi', color: 'var(--c-bad)', order: 11, on: true, mode: 'points', size: 9 },
    { key: 'collision_points', label: 'kolizja', color: 'var(--c-collision)', order: 12, on: true, mode: 'points', size: 11 },
    { key: 'margin_points', label: 'poniżej prześwitu', color: 'var(--c-margin)', order: 11, on: true, mode: 'points', size: 8 },
  ];
  // ---------------------------------------------------------------- theme
  function setTheme(mode, save) {
    const root = document.documentElement;
    if (mode === 'auto') root.removeAttribute('data-theme'); else root.setAttribute('data-theme', mode);
    $$('.theme-switch button').forEach(b => b.classList.toggle('active', b.dataset.theme === mode));
    if (save) { try { localStorage.setItem('gcode2krl.theme', mode); } catch (e) {} }
    if (window.__viewer) window.__viewer.applyTheme();
    if (typeof drawClearanceChart === 'function') drawClearanceChart();
    if (state.result) showBreakdown(state.result.stats);
  }


  let viewer, robotView;
  try { viewer = new PathViewer($('#view')); robotView = new RobotView(viewer); window.__viewer = viewer; }
  catch (e) { $('#viewer-error').textContent = 'WebGL niedostępny: ' + e.message; }
  const ROBOTS = {};
  const robotsReady = fetch('/api/robots').then(r => r.json()).then(d => {
    const sel = $('#opt_robot');
    for (const r of d.robots) { ROBOTS[r.key] = r; const o = document.createElement('option'); o.value = r.key; o.textContent = r.name; sel.appendChild(o); }
    if (!sel.value || sel.value === 'none') sel.value = d.default;
    sel.dataset.ready = '1';
    const ta = $('#opt_obstacles');
    const helpLines = d.obstacle_help.split('\n').filter(l => !/^box (stol|material) /.test(l));
    ta.dataset.default = helpLines.join('\n');
    if (!ta.value) ta.value = helpLines.join('\n');
    updateRobotModel();
  }).catch(() => {});

  // ----------------------------------------------------------------- options
  const OPTION_IDS = ['extruder','extruder_out','extruder_analog','extruder_anout','extruder_flow_full','extruder_delay',
    'extruder_trigger','extruder_on_delay','extruder_off_delay','extruder_reverse_out','krl_start','krl_end',
    'orient_solve','spin_step','singular_deg','max_tilt','robot','base_frame','tool_frame','base_literal','tool_literal','robot_samples','path_step','margin',
    'tbl_sx','tbl_sy','tbl_cx','tbl_cy','mat_t','mat_pad','tool_mesh','tool_mesh_units',
    'tool_radius','cutter_len','cutter_radius','obstacles','tol','max_radius','min_radius','min_arc_points','refine_shift','spl_max_segment','corner_angle',
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
  // the GUI speaks robot units (mm/s, ms); the API keeps the G-code conventions (mm/min, s)
  function sendOptions() {
    const o = readOptions();
    if (o.default_feed !== undefined) o.default_feed = Number(o.default_feed) * 60;
    if (o.extruder_on_delay !== undefined) o.extruder_on_delay = Number(o.extruder_on_delay) / 1000;
    if (o.extruder_off_delay !== undefined) o.extruder_off_delay = Number(o.extruder_off_delay) / 1000;
    return o;
  }
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
    syncFramesFromOptions();
    saveOptions();
    if (typeof applyFrames === 'function') applyFrames();
    updateAutoObstacles();
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
  // source of the path: the slicer (step 1) or a ready-made G-code file (step 2)
  function setSource(src, save) {
    $$('.src-tab').forEach(b => b.classList.toggle('active', b.dataset.src === src));
    $('#slicer').classList.toggle('hidden', src !== 'slicer');
    $('#drop').classList.toggle('hidden', src !== 'gcode');
    if (save) { try { localStorage.setItem('gcode2krl.source', src); } catch (e) {} }
  }
  $$('.src-tab').forEach(b => b.addEventListener('click', () => setSource(b.dataset.src, true)));
  let savedSource = 'slicer';
  try { savedSource = localStorage.getItem('gcode2krl.source') || 'slicer'; } catch (e) {}
  setSource(savedSource, false);
  function readFile(file) {
    setSource('gcode', true);
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
  const MODEL_EXT = /\.(stl|obj|step|stp|iges|igs)$/i;
  // a file dropped anywhere: models go to the slicer, everything else is G-code
  document.addEventListener('drop', e => {
    e.preventDefault(); const f = e.dataTransfer.files[0]; if (!f) return;
    if (MODEL_EXT.test(f.name)) uploadSliceModel(f); else readFile(f);
  });

  fetch('/api/examples').then(r => r.json()).then(d => {
    const sel = $('#examples');
    for (const n of d.examples) { const o = document.createElement('option'); o.value = n; o.textContent = n; sel.appendChild(o); }
  }).catch(() => {});
  $('#examples').addEventListener('change', async e => {
    if (e.target.value) setSource('gcode', true);
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
  $$('#options input, #options select, #options textarea').forEach(el => {
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
        body: JSON.stringify({ gcode: state.gcode, filename: state.filename, options: sendOptions() }),
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

    showBreakdown(s);

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
        const col = data.robot && data.robot.collision;
        const src = L.key === 'bad_points' ? (data.robot ? data.robot.bad_points : '')
          : L.key === 'collision_points' ? (col ? col.collision_points : '')
          : L.key === 'margin_points' ? (col ? col.margin_points : '') : p[L.key];
        const arr = decode(src);
        viewer.setLayer(L.key, arr, L.color, { alpha: L.alpha, order: L.order, mode: L.mode, size: L.size, visible: $('#layer-' + L.key).checked });
      }
      showRobot(data.robot);
      viewer.setBounds(p.bounds.min, p.bounds.max);
      drawFrames();
      if (!state.fitted) { viewer.viewIso(); state.fitted = true; } else viewer.invalidate();
      updateAutoObstacles();
      const b = p.bounds;
      $('#bounds').textContent = `X ${fmt(b.min[0], 2)} … ${fmt(b.max[0], 2)}   Y ${fmt(b.min[1], 2)} … ${fmt(b.max[1], 2)}   Z ${fmt(b.min[2], 2)} … ${fmt(b.max[2], 2)} mm`;
    }
  }

  function showBreakdown(s) {
    const total = Math.max(1, s.lines_out + s.arcs_out + s.spl_points + s.rapids);
    const bar = $('#breakdown');
    bar.innerHTML = '';
    for (const [k, v, c] of [['SLIN', s.lines_out, 'var(--c-slin)'], ['SCIRC', s.arcs_out, 'var(--c-scirc)'], ['SPL', s.spl_points, 'var(--c-spl)'], ['G0', s.rapids, 'var(--c-rapid)']]) {
      if (!v) continue;
      const d = document.createElement('div');
      d.style.width = (100 * v / total) + '%'; d.style.background = c; d.title = `${k}: ${v}`;
      d.textContent = (100 * v / total) > 8 ? `${k} ${v}` : '';
      bar.appendChild(d);
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

  // ------------------------------------------------------------ cell setup
  const AXES6 = ['x', 'y', 'z', 'a', 'b', 'c'];
  function hiddenToFields(prefix, hiddenId) {
    const v = parse6('#' + hiddenId, [0, 0, 0, 0, 0, 0]);
    AXES6.forEach((ax, i) => { const el = $('#' + prefix + '_' + ax); if (el && document.activeElement !== el) el.value = +v[i].toFixed(3); });
  }
  function fieldsToHidden(prefix, hiddenId) {
    const v = AXES6.map(ax => Number($('#' + prefix + '_' + ax).value) || 0);
    const h = $('#' + hiddenId);
    const text = v.map(x => +x.toFixed(3)).join(' ');
    if (h.value !== text) { h.value = text; return true; }
    return false;
  }
  function setFrameFields(prefix, hiddenId, v) {
    $('#' + hiddenId).value = v.map(x => +x.toFixed(3)).join(' ');
    hiddenToFields(prefix, hiddenId);
  }
  function onFrameEdit(prefix, hiddenId) {
    if (fieldsToHidden(prefix, hiddenId)) {
      saveOptions(); applyFrames(); updateAutoObstacles(); scheduleAuto();
    }
  }
  AXES6.forEach(ax => {
    $('#tf_' + ax).addEventListener('input', () => onFrameEdit('tf', 'opt_tool_frame'));
    $('#bf_' + ax).addEventListener('input', () => onFrameEdit('bf', 'opt_base_frame'));
  });
  function syncFramesFromOptions() { hiddenToFields('tf', 'opt_tool_frame'); hiddenToFields('bf', 'opt_base_frame'); }

  // tool presets (browser storage)
  const BUILTIN_TOOLS = [
    { name: 'Wrzeciono 150 mm', frame: [0, 0, 150, 0, 0, 0], tool_radius: 25, cutter_len: 20, cutter_radius: 6 },
    { name: 'Wrzeciono 250 mm', frame: [0, 0, 250, 0, 0, 0], tool_radius: 30, cutter_len: 30, cutter_radius: 8 },
    { name: 'Kołnierz (bez narzędzia)', frame: [0, 0, 0, 0, 0, 0], tool_radius: 25, cutter_len: 0, cutter_radius: 6 },
  ];
  function loadTools() { try { return JSON.parse(localStorage.getItem('gcode2krl.tools') || '[]'); } catch (e) { return []; } }
  function saveTools(t) { try { localStorage.setItem('gcode2krl.tools', JSON.stringify(t)); } catch (e) {} }
  function refreshToolPresets() {
    const sel = $('#tool-presets');
    sel.innerHTML = '<option value="">narzędzia…</option>';
    const all = BUILTIN_TOOLS.map(t => ({ ...t, builtin: true })).concat(loadTools());
    all.forEach((t, i) => { const o = document.createElement('option'); o.value = String(i); o.textContent = t.name + (t.builtin ? '' : ' ·'); sel.appendChild(o); });
    sel.dataset.all = JSON.stringify(all);
  }
  $('#tool-presets').addEventListener('change', e => {
    const all = JSON.parse(e.target.dataset.all || '[]'); const t = all[Number(e.target.value)];
    if (!t) return;
    setFrameFields('tf', 'opt_tool_frame', t.frame);
    $('#opt_tool_radius').value = t.tool_radius; $('#opt_cutter_len').value = t.cutter_len; $('#opt_cutter_radius').value = t.cutter_radius;
    $('#tool-name').value = t.name;
    saveOptions(); applyFrames(); scheduleAuto();
  });
  $('#tool-save').addEventListener('click', () => {
    const name = $('#tool-name').value.trim(); if (!name) { setStatus('Podaj nazwę narzędzia', 'err'); return; }
    const tools = loadTools().filter(t => t.name !== name);
    tools.push({ name, frame: parse6('#opt_tool_frame', [0,0,0,0,0,0]), tool_radius: Number($('#opt_tool_radius').value), cutter_len: Number($('#opt_cutter_len').value), cutter_radius: Number($('#opt_cutter_radius').value) });
    saveTools(tools); refreshToolPresets(); setStatus(`Zapisano narzędzie „${name}”`, 'ok');
  });
  $('#tool-del').addEventListener('click', () => {
    const name = $('#tool-name').value.trim();
    const tools = loadTools(); if (!tools.some(t => t.name === name)) { setStatus('Takie narzędzie nie jest zapisane (wbudowanych nie można usunąć)', 'err'); return; }
    saveTools(tools.filter(t => t.name !== name)); refreshToolPresets(); setStatus(`Usunięto „${name}”`, 'ok');
  });
  refreshToolPresets();

  // base from 3 points (KSS 3-point method)
  $('#base-3p-toggle').addEventListener('click', () => $('#base-3p').classList.toggle('hidden'));
  function rotToABC(x, y, z) { // columns of R
    const r00 = x[0], r10 = x[1], r20 = x[2], r21 = y[2], r22 = z[2];
    const d = 180 / Math.PI;
    const b = Math.atan2(-r20, Math.hypot(r00, r10));
    let a, c;
    if (Math.abs(Math.cos(b)) < 1e-9) { a = 0; c = Math.atan2(y[0], y[1]) * (b > 0 ? 1 : -1); }
    else { a = Math.atan2(r10, r00); c = Math.atan2(r21, r22); }
    return [a * d, b * d, c * d];
  }
  $('#base-3p-apply').addEventListener('click', () => {
    const P = k => ['x', 'y', 'z'].map(ax => Number($('#p' + k + '_' + ax).value));
    const o = P(1), px = P(2), pxy = P(3);
    const sub = (u, v) => u.map((a, i) => a - v[i]), nrm = v => Math.hypot(...v), cr = (u, v) => [u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0]];
    const msg = $('#base-3p-msg');
    let x = sub(px, o); if (nrm(x) < 1e-9 || [o, px, pxy].flat().some(v => !Number.isFinite(v))) { msg.textContent = 'P2 musi różnić się od P1 (uzupełnij wszystkie pola)'; msg.className = 'bad'; return; }
    x = x.map(v => v / nrm(x));
    let z = cr(x, sub(pxy, o)); if (nrm(z) < 1e-9) { msg.textContent = 'P3 leży na osi X'; msg.className = 'bad'; return; }
    z = z.map(v => v / nrm(z));
    const y = cr(z, x);
    const abc = rotToABC(x, y, z);
    setFrameFields('bf', 'opt_base_frame', [o[0], o[1], o[2], abc[0], abc[1], abc[2]]);
    msg.textContent = `baza: A ${abc[0].toFixed(2)}° B ${abc[1].toFixed(2)}° C ${abc[2].toFixed(2)}°`; msg.className = 'ok';
    saveOptions(); applyFrames(); updateAutoObstacles(); scheduleAuto();
  });
  $('#base-center').addEventListener('click', () => {
    const b = state.result ? state.result.preview.bounds : { min: [0, 0, 0], max: [0, 0, 0] };
    const cx = (b.min[0] + b.max[0]) / 2, cy = (b.min[1] + b.max[1]) / 2;
    const f = parse6('#opt_base_frame', [1400, 0, 500, 0, 0, 0]);
    const d = Math.PI / 180, ca = Math.cos(f[3] * d), sa = Math.sin(f[3] * d);
    // robot-frame position of the path centre = base origin + Rz(A)*(cx, cy): put it at (dist, 0)
    const dist = Number($('#base-dist').value) || 1400;
    f[0] = dist - (ca * cx - sa * cy); f[1] = 0 - (sa * cx + ca * cy);
    setFrameFields('bf', 'opt_base_frame', f);
    saveOptions(); applyFrames(); updateAutoObstacles(); scheduleAuto();
  });

  // automatic table and stock obstacles from the base frame and the path bounds
  function updateAutoObstacles() {
    const ta = $('#opt_obstacles'); if (!ta) return;
    const keep = ta.value.split('\n').filter(l => !/#\s*auto\s*$/.test(l));
    let lines = keep;
    if ($('#auto-table').checked) {
      const f = parse6('#opt_base_frame', [1400, 0, 500, 0, 0, 0]);
      const sx = Number($('#tbl_sx').value) || 900, sy = Number($('#tbl_sy').value) || 900;
      const b0 = state.result ? state.result.preview.bounds : { min: [-100, -100, 0], max: [100, 100, 0] };
      const cx = $('#tbl_cx').value === '' ? (b0.min[0] + b0.max[0]) / 2 : Number($('#tbl_cx').value);
      const cy = $('#tbl_cy').value === '' ? (b0.min[1] + b0.max[1]) / 2 : Number($('#tbl_cy').value);
      // table height: from the base plane down to the robot floor (z=0) when the base is level above it
      const t = Number($('#mat_t').value) || 20, pad = Number($('#mat_pad').value) || 0;
      // the table top carries the stock: top at base Z = -t, bottom on the robot floor when the base is level
      const h = (Math.abs(f[4]) < 1e-6 && Math.abs(f[5]) < 1e-6 && f[2] > t + 50) ? f[2] - t : 480;
      const b = state.result ? state.result.preview.bounds : { min: [-100, -100, 0], max: [100, 100, 0] };
      const mx = (b.min[0] + b.max[0]) / 2, my = (b.min[1] + b.max[1]) / 2;
      const msx = Math.max(10, b.max[0] - b.min[0] + 2 * pad), msy = Math.max(10, b.max[1] - b.min[1] + 2 * pad);
      const r = v => Math.round(v * 10) / 10;
      lines = keep.concat([
        `box stol base ${r(cx)} ${r(cy)} ${r(-t - h / 2)} ${r(sx)} ${r(sy)} ${r(h)}  # auto`,
        `box material base ${r(mx)} ${r(my)} ${r(-t / 2)} ${r(msx)} ${r(msy)} ${r(t)} workpiece  # auto`,
      ]);
    }
    const text = lines.join('\n').replace(/\n{3,}/g, '\n\n');
    if (text !== ta.value) { ta.value = text; saveOptions(); scheduleAuto(); }
  }
  ['#auto-table', '#tbl_sx', '#tbl_sy', '#tbl_cx', '#tbl_cy', '#mat_t', '#mat_pad'].forEach(sel => $(sel).addEventListener('input', updateAutoObstacles));
  // ---- imported meshes (tables, fixtures, tools) ------------------------
  function readFileB64(file) {
    return new Promise((res, rej) => { const r = new FileReader(); r.onload = () => res(String(r.result).split(',')[1]); r.onerror = rej; r.readAsDataURL(file); });
  }
  async function uploadMesh(file) {
    const data = await readFileB64(file);
    const deflection = Number(($('#step-deflection') || {}).value) || 0.2;
    const res = await fetch('/api/mesh', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ name: file.name, data, deflection }) });
    const meta = await res.json();
    if (!res.ok || meta.error) throw new Error(meta.error || res.statusText);
    return meta;
  }
  const UNIT_MM = { mm: 1, cm: 10, m: 1000, in: 25.4 };
  const MESH_LINE = /^mesh\s+(\S+)\s+(robot|base)\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+(\S+)(?:\s+(mm|cm|m|in))?(\s+workpiece)?\s*$/i;
  function parseMeshLines() {
    const out = [];
    $('#opt_obstacles').value.split('\n').forEach((line, idx) => {
      const m = line.split('#')[0].trim().match(MESH_LINE);
      if (m) out.push({ idx, name: m[1], frame: m[2].toLowerCase(), v: m.slice(3, 9).map(Number), file: m[9], units: m[10] || 'mm', workpiece: !!m[11], raw: line });
    });
    return out;
  }
  function writeMeshLine(e) {
    return `mesh ${e.name} ${e.frame} ${e.v.map(x => +Number(x).toFixed(3)).join(' ')} ${e.file} ${e.units}${e.workpiece ? ' workpiece' : ''}`;
  }
  function replaceObstacleLine(idx, text) {
    const ta = $('#opt_obstacles');
    const lines = ta.value.split('\n');
    if (text === null) lines.splice(idx, 1); else lines[idx] = text;
    ta.value = lines.join('\n');
    saveOptions(); renderMeshList(); scheduleAuto();
  }
  let meshMeta = {};
  async function refreshMeshMeta() {
    try {
      const d = await (await fetch('/api/meshes')).json(); meshMeta = {}; for (const m of d.meshes) meshMeta[m.id] = m;
      refreshSliceMeshes();
      const sb = $('#step-backend');
      if (sb) sb.textContent = d.step_backends && d.step_backends.length ? `Import STEP/IGES: kernel ${d.step_backends.join(', ')} (OpenCASCADE), jednostki z pliku.` : 'Import STEP wymaga kernela CAD: pip install cadquery-ocp (albo pip install gmsh).';
    } catch (e) {}
    renderMeshList();
  }
  function renderMeshList() {
    const box = $('#mesh-list'); if (!box) return;
    box.innerHTML = '';
    const entries = parseMeshLines();
    if (!entries.length) { box.innerHTML = '<div class="muted" style="font-size:11.5px">Brak modeli. Dodaj plik STL/OBJ stołu, uchwytu lub osłony.</div>'; return; }
    for (const e of entries) {
      const meta = meshMeta[e.file];
      const div = document.createElement('div');
      div.className = 'mesh-item';
      div.innerHTML = `<span class="mi-name" title="${e.file}">${e.name}</span>
        <span class="mi-actions"><label class="wp"><input type="checkbox" ${e.workpiece ? 'checked' : ''} data-k="wp"> materiał</label><button type="button" class="mini" data-k="del" title="usuń z przeszkód">×</button></span>
        <span class="mi-meta">${meta ? meta.name + ' · ' + meta.tris.toLocaleString('pl-PL') + ' trójkątów · ' + meta.min.map((v, i) => ((meta.max[i] - v) * UNIT_MM[e.units]).toFixed(0)).join('×') + ' mm' : e.file} · układ ${e.frame} · plik w ${e.units}</span>
        <span class="mi-frame">${['X','Y','Z','A','B','C'].map((ax, i) => `<input type="number" step="${i < 3 ? 10 : 1}" value="${e.v[i]}" data-k="${i}" title="${ax} w układzie ${e.frame}">`).join('')}</span>`;
      div.querySelectorAll('input[type=number]').forEach(inp => inp.addEventListener('change', () => { e.v[Number(inp.dataset.k)] = Number(inp.value) || 0; replaceObstacleLine(e.idx, writeMeshLine(e)); }));
      div.querySelector('[data-k=wp]').addEventListener('change', ev => { e.workpiece = ev.target.checked; replaceObstacleLine(e.idx, writeMeshLine(e)); });
      div.querySelector('[data-k=del]').addEventListener('click', () => replaceObstacleLine(e.idx, null));
      box.appendChild(div);
    }
  }
  $('#obstacle-mesh-file').addEventListener('change', async ev => {
    const files = [...ev.target.files]; ev.target.value = '';
    for (const f of files) {
      try {
        setStatus(`Wgrywanie ${f.name}…`, 'busy');
        const meta = await uploadMesh(f);
        meshMeta[meta.id] = meta;
        const name = f.name.replace(/\.[^.]+$/, '').replace(/[^\w-]+/g, '_').slice(0, 24) || 'model';
        const units = $('#obstacle-mesh-units').value;
        const b = state.result ? state.result.preview.bounds : { min: [0, 0, 0], max: [0, 0, 0] };
        const line = writeMeshLine({ name, frame: 'base', v: [(b.min[0] + b.max[0]) / 2, (b.min[1] + b.max[1]) / 2, 0, 0, 0, 0], file: meta.id, units, workpiece: false });
        const ta = $('#opt_obstacles'); ta.value = ta.value.replace(/\s*$/, '') + '\n' + line;
        saveOptions(); renderMeshList(); scheduleAuto();
        setStatus(`Dodano model ${meta.name} (${meta.tris} trójkątów)`, 'ok');
      } catch (err) { setStatus('Błąd wgrywania: ' + err.message, 'err'); }
    }
  });
  $('#opt_obstacles').addEventListener('input', renderMeshList);
  $('#tool-mesh-file').addEventListener('change', async ev => {
    const f = ev.target.files[0]; ev.target.value = ''; if (!f) return;
    try {
      setStatus(`Wgrywanie ${f.name}…`, 'busy');
      const meta = await uploadMesh(f);
      meshMeta[meta.id] = meta;
      $('#opt_tool_mesh').value = meta.id;
      updateToolMeshInfo();
      saveOptions(); scheduleAuto();
      setStatus(`Model narzędzia: ${meta.name}`, 'ok');
    } catch (err) { setStatus('Błąd wgrywania: ' + err.message, 'err'); }
  });
  $('#tool-mesh-clear').addEventListener('click', () => { $('#opt_tool_mesh').value = ''; updateToolMeshInfo(); saveOptions(); if (robotView) robotView.setToolMesh(null); scheduleAuto(); });
  $('#opt_tool_mesh_units').addEventListener('change', () => { saveOptions(); scheduleAuto(); loadToolMeshGeometry(); });
  function updateToolMeshInfo() {
    const id = $('#opt_tool_mesh').value; const meta = meshMeta[id];
    const u = UNIT_MM[$('#opt_tool_mesh_units').value] || 1;
    $('#tool-mesh-info').textContent = id ? (meta ? `Model: ${meta.name} · ${meta.tris.toLocaleString('pl-PL')} trójkątów · ${meta.min.map((v, i) => ((meta.max[i] - v) * u).toFixed(0)).join('×')} mm` : `Model: ${id}`) : 'Brak modelu: uchwyt jest kapsułą o promieniu z pola „promień uchwytu”.';
    loadToolMeshGeometry();
  }
  let toolMeshLoaded = '';
  async function loadToolMeshGeometry() {
    const id = $('#opt_tool_mesh').value; const units = $('#opt_tool_mesh_units').value;
    const key = id + '@' + units;
    if (!robotView) return;
    if (!id) { robotView.setToolMesh(null); toolMeshLoaded = ''; return; }
    if (toolMeshLoaded === key) return;
    try { const g = await (await fetch('/api/mesh/' + id + '?units=' + units)).json(); if (!g.error) { robotView.setToolMesh(g); toolMeshLoaded = key; } } catch (e) {}
  }
  // ---- slicer ------------------------------------------------------------
  function sliceParams() { return window.SLICER_FORM ? window.SLICER_FORM.params() : {}; }
  function refreshSliceMeshes() {
    const sel = $('#slice-mesh'); if (!sel) return;
    const cur = sel.value;
    sel.innerHTML = '<option value="">wybierz model…</option>';
    for (const m of Object.values(meshMeta)) { const o = document.createElement('option'); o.value = m.id; o.textContent = `${m.name} (${m.tris} tr.)`; sel.appendChild(o); }
    if (cur && meshMeta[cur]) sel.value = cur;
    if (!sel.value && !cur) { const first = Object.values(meshMeta)[0]; if (first) sel.value = first.id; }
    showModel(meshMeta[sel.value] || null);
  }
  function showModel(meta) {
    const md = $('#model-drop');
    if (!meta) { $('#model-name').textContent = 'Upuść model STEP/STL/OBJ lub kliknij'; $('#model-meta').textContent = '.step .stp .stl .obj .iges'; md.classList.remove('has-file'); return; }
    $('#model-name').textContent = meta.name;
    $('#model-meta').textContent = `${meta.tris.toLocaleString('pl-PL')} trójkątów · ${meta.min.map((v, i) => (meta.max[i] - v).toFixed(0)).join(' × ')} ${$('#slice-units').value}`;
    md.classList.add('has-file');
  }
  async function uploadSliceModel(f) {
    setSource('slicer', true);
    try {
      setStatus(`Wgrywanie ${f.name}…`, 'busy');
      const meta = await uploadMesh(f); meshMeta[meta.id] = meta; refreshSliceMeshes(); $('#slice-mesh').value = meta.id; showModel(meta);
      setStatus(`Model ${meta.name}: ${meta.tris} trójkątów, ${meta.min.map((v, i) => (meta.max[i] - v).toFixed(0)).join('×')} ${$('#slice-units').value} — ustaw parametry i pokrój`, 'ok');
    } catch (err) { setStatus('Błąd wgrywania: ' + err.message, 'err'); }
  }
  $('#slice-file').addEventListener('change', ev => { const f = ev.target.files[0]; ev.target.value = ''; if (f) uploadSliceModel(f); });
  $('#btn-model-upload').addEventListener('click', () => $('#slice-file').click());
  const modelDrop = $('#model-drop');
  modelDrop.addEventListener('click', () => $('#slice-file').click());
  modelDrop.addEventListener('dragover', e => { e.preventDefault(); e.stopPropagation(); modelDrop.classList.add('over'); });
  modelDrop.addEventListener('dragleave', () => modelDrop.classList.remove('over'));
  modelDrop.addEventListener('drop', e => { e.preventDefault(); e.stopPropagation(); modelDrop.classList.remove('over'); const f = e.dataTransfer.files[0]; if (f) uploadSliceModel(f); });
  $('#slice-mesh').addEventListener('change', e => showModel(meshMeta[e.target.value] || null));
  $('#slice-units').addEventListener('change', () => { const m = meshMeta[$('#slice-mesh').value]; if (m) showModel(m); });
  $('#btn-slice').addEventListener('click', async () => {
    const ref = $('#slice-mesh').value; if (!ref) { setStatus('Wybierz model do pokrojenia', 'err'); return; }
    setStatus('Krojenie…', 'busy'); $('#btn-slice').disabled = true;
    try {
      const params = sliceParams();
      const body = { mesh: ref, units: $('#slice-units').value, params };
      if (params.layer_mode === 'multi') { body.segments = window.SLICER_FORM.extras().segments; params.layer_mode = 'planar'; if (!body.segments.trim()) throw new Error('tryb wielokierunkowy wymaga definicji segmentów'); }
      const res = await fetch('/api/slice', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
      const d = await res.json();
      if (!res.ok || d.error) throw new Error(d.error || res.statusText);
      const st = d.stats;
      const modeTxt = { planar: 'płaskie', conical: 'stożkowe', bottom: 'konforemne (dół)', top: 'konforemne (góra)', multi: 'wielokierunkowe' }[st.layer_mode] || st.layer_mode;
      const hTxt = st.min_layer_height && st.max_layer_height && Math.abs(st.max_layer_height - st.min_layer_height) > 1e-6 ? ` (${st.min_layer_height.toFixed(2)}–${st.max_layer_height.toFixed(2)} mm)` : '';
      const bt = st.by_type || {};
      const feat = [];
      if (bt['GAP']) feat.push(`gap fill ${(bt['GAP'] / 1000).toFixed(1)} m`);
      if (st.bridges) feat.push(`${st.bridges} mostów`);
      if (st.overhang_length > 1) feat.push(`nawisy spowolnione ${(st.overhang_length / 1000).toFixed(1)} m`);
      if (st.support_volume > 0) feat.push(`podpory ${(st.support_volume / 1000).toFixed(0)} cm³`);
      if (bt['RAFT']) feat.push('raft');
      if (st.slowed_layers) feat.push(`${st.slowed_layers} warstw spowolnionych (min. czas)`);
      $('#slice-info').textContent = `${st.layers} warstw${hTxt} (${modeTxt}) · ${st.paths} ścieżek · ekstruzja ${(st.extrusion_length / 1000).toFixed(1)} m · przejazdy ${(st.travel_length / 1000).toFixed(1)} m · ok. ${(st.time_s / 60).toFixed(0)} min · materiał ${(st.volume_mm3 / 1000).toFixed(0)} cm³ ≈ ${st.mass_g.toFixed(0)} g`
        + (feat.length ? ' · ' + feat.join(' · ') : '')
        + (st.max_tilt > 0.01 ? ` · maks. pochylenie narzędzia ${st.max_tilt.toFixed(1)}°` : '')
        + (st.support_layers && !(st.support_volume > 0) ? ` · NAWISY: ${st.support_layers} warstw, ${(st.support_area / 100).toFixed(0)} cm² wymaga podpór` : (st.support_volume > 0 ? '' : ' · bez nawisów'))
        + (st.dropped_chains ? ` · ${st.dropped_chains} otwartych konturów pominięto` : '');
      $('#opt_extruder').checked = true; saveOptions();
      setGcode(d.gcode, (d.name || 'model').replace(/\.[^.]+$/, '') + '.gcode');
    } catch (err) { setStatus('Błąd krojenia: ' + err.message, 'err'); }
    finally { $('#btn-slice').disabled = false; }
  });
  function drawFrames() {
    if (!viewer) return;
    const b = viewer.bounds; const span = Math.max(b.max[0] - b.min[0], b.max[1] - b.min[1], 50);
    const L = Math.max(40, span * 0.25);
    const cols = ['var(--c-ax-x)', 'var(--c-ax-y)', 'var(--c-ax-z)'];
    [[L, 0, 0], [0, L, 0], [0, 0, L]].forEach((e, i) => viewer.setLayer('frame_base' + i, new Float32Array([0, 0, 0, ...e]), cols[i], { order: 14, group: 'frames' }));
  }

  // ------------------------------------------------------------------ robot
  const anim = { joints: null, n: 0, pos: 0, playing: false, last: 0, raf: null };
  function updateRobotModel() {
    const key = $('#opt_robot').value;
    const R = ROBOTS[key];
    if (!robotView) return;
    if (!R) { robotView.setVisible(false); $('#joints').classList.add('hidden'); return; }
    if (!robotView.model || robotView.model.key !== key) {
      robotView.setModel(R, null); buildJointRows(R);
      if (R.has_mesh) fetch('/api/robot_mesh/' + key).then(r => r.json()).then(m => { if (robotView.model && robotView.model.key === key) { robotView.setModel(R, m); applyFrames(); } }).catch(() => {});
      const src = $('#robot-sources');
      src.innerHTML = Object.entries(R.sources || {}).map(([k, v]) => `<div><b>${k}:</b> ${v}</div>`).join('') + (R.notes || []).map(n => `<div class="warn">${n}</div>`).join('');
    }
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
    $('#robot-axes').innerHTML = rep.joint_min.map((lo, i) => lo === null ? '' : `<span class="${rep.axis_violations[i] ? 'viol' : ''}">A${i+1} <b>${lo.toFixed(0)}…${rep.joint_max[i].toFixed(0)}°</b></span>`).join('')
      + (rep.orientation ? `<span title="rozwiązywanie obrotu wokół osi narzędzia">orient. <b>${rep.orientation.points}</b> pkt · ${rep.orientation.resampled} przestawień${rep.orientation.singular ? ` · <span class="viol">${rep.orientation.singular} osobl.</span>` : ''}${rep.orientation.unreachable ? ` · <span class="viol">${rep.orientation.unreachable} nieosiąg.</span>` : ''}</span>` : (rep.oriented ? '<span>orientacja z G-code</span>' : ''));
    showCollisions(rep.collision, rep);
    // animation data
    anim.joints = decode(rep.joints); anim.ok = Uint8Array.from(atob(rep.ok), c => c.charCodeAt(0)); anim.n = anim.joints.length / 6; anim.pos = 0;
    $('#anim').max = Math.max(0, anim.n - 1); $('#anim').value = 0;
    updateRobotModel();
    if (anim.n) poseAt(0);
  }
  let colData = null;
  function showCollisions(col, rep) {
    const badge = $('#tab-col-count');
    if (!col || !robotView) { colData = null; badge.textContent = ''; $('#col-summary').innerHTML = ''; $('#col-table tbody').innerHTML = ''; return; }
    colData = { clearance: decode(col.clearance), margin: col.margin, worst: col.worst_pair, pairs: col.pairs };
    badge.textContent = col.n_collision ? `(${col.n_collision}!)` : (col.n_margin ? `(${col.n_margin})` : '');
    badge.style.color = col.n_collision ? 'var(--bad)' : (col.n_margin ? 'var(--warn)' : '');
    const cls = col.n_collision ? 'bad' : (col.n_margin || col.unresolved ? 'warn' : 'ok');
    const minc = col.min_clearance === null ? '–' : col.min_clearance.toFixed(1) + ' mm';
    $('#col-summary').innerHTML = `
      <span class="${cls}"><b>${col.n_collision ? col.n_collision + ' kolizji' : 'brak kolizji'}</b></span>
      <span class="${col.n_margin ? 'warn' : ''}"><b>${col.n_margin}</b> próbek poniżej prześwitu ${col.margin} mm</span>
      <span>min. prześwit <b class="${cls}">${minc}</b></span>
      <span class="muted">${col.n_samples} próbek co ${rep.step} mm · ${col.refined_samples} zagęszczeń IK · <b class="${col.unresolved ? 'warn' : ''}">${col.unresolved}</b> nierozstrzygniętych</span>
      <span class="${col.config_jumps ? 'warn' : 'muted'}">skoki konfiguracji: <b>${col.config_jumps}</b></span>
      <span class="${col.singular ? 'warn' : 'muted'}">osobliwość nadgarstka: <b>${col.singular}</b></span>`;
    const tb = $('#col-table tbody');
    tb.innerHTML = '';
    for (const p of col.pairs) {
      const tr = document.createElement('tr');
      tr.className = p.min_clearance <= 0 ? 'bad' : (p.min_clearance < col.margin ? 'warn' : '');
      tr.innerHTML = `<td>${p.a}</td><td>${p.b}</td><td>${p.kind === 'self' ? 'autokolizja' : 'przeszkoda'}</td><td class="num">${p.min_clearance.toFixed(1)} mm</td><td class="num">${p.n_collision}</td><td class="num">${p.n_margin}</td><td class="num">${p.argmin + 1}</td>`;
      tr.addEventListener('click', () => { stopAnim(); anim.pos = p.argmin; poseAt(anim.pos); $$('.tab')[0].click(); });
      tb.appendChild(tr);
    }
    const notes = $('#col-notes');
    notes.innerHTML = '';
    for (const n of col.notes) { const li = document.createElement('li'); li.textContent = n; notes.appendChild(li); }
    robotView.setObstacles(col.obstacles, (id, units) => fetch('/api/mesh/' + encodeURIComponent(id) + '?units=' + encodeURIComponent(units || 'mm')).then(r => r.json()));
    robotView.setCapsules(col.capsules);
    updateToolMeshInfo();
    drawClearanceChart();
  }
  function drawClearanceChart() {
    const cv = $('#col-chart'); if (!colData) return;
    const dpr = window.devicePixelRatio || 1;
    const W = cv.clientWidth || 800, H = 140;
    cv.width = W * dpr; cv.height = H * dpr;
    const g = cv.getContext('2d'); g.scale(dpr, dpr);
    g.clearRect(0, 0, W, H);
    const c = colData.clearance, n = c.length; if (!n) return;
    let maxv = 0; for (let i = 0; i < n; i++) if (c[i] < 1e8) maxv = Math.max(maxv, c[i]);
    maxv = Math.max(maxv, colData.margin * 2, 1);
    let minv = 0; for (let i = 0; i < n; i++) minv = Math.min(minv, c[i]);
    const top = 10, bottom = H - 18, left = 44, right = W - 10;
    const y = v => bottom - (Math.min(v, maxv) - minv) / (maxv - minv) * (bottom - top);
    const x = i => left + (right - left) * i / Math.max(1, n - 1);
    const tk = n => cssVar(n, '#888');
    g.strokeStyle = tk('--line'); g.lineWidth = 1;
    g.beginPath(); g.moveTo(left, y(0)); g.lineTo(right, y(0)); g.stroke();
    g.strokeStyle = tk('--warn'); g.setLineDash([4, 4]); g.beginPath(); g.moveTo(left, y(colData.margin)); g.lineTo(right, y(colData.margin)); g.stroke(); g.setLineDash([]);
    g.fillStyle = tk('--muted'); g.font = '11px ' + tk('--font'); g.textAlign = 'right';
    g.fillText('0', left - 6, y(0) + 4); g.fillText(colData.margin + '', left - 6, y(colData.margin) + 4); g.fillText(maxv.toFixed(0), left - 6, top + 4);
    g.textAlign = 'left'; g.fillText('prześwit [mm] wzdłuż ścieżki', left, H - 5);
    g.lineWidth = 1.5; g.strokeStyle = tk('--ok'); g.beginPath();
    let started = false;
    for (let i = 0; i < n; i++) { if (c[i] > 1e8) { started = false; continue; } const px = x(i), py = y(c[i]); if (!started) { g.moveTo(px, py); started = true; } else g.lineTo(px, py); }
    g.stroke();
    g.fillStyle = tk('--c-collision');
    for (let i = 0; i < n; i++) if (c[i] <= 0) g.fillRect(x(i) - 1, top, 2, bottom - top);
    g.globalAlpha = 0.5; g.fillStyle = tk('--c-margin');
    for (let i = 0; i < n; i++) if (c[i] > 0 && c[i] < colData.margin) g.fillRect(x(i) - 1, y(colData.margin), 2, y(c[i]) - y(colData.margin));
    g.globalAlpha = 1;
    if (anim.n) { g.strokeStyle = tk('--text'); g.beginPath(); g.moveTo(x(anim.pos), top); g.lineTo(x(anim.pos), bottom); g.stroke(); }
  }
  window.addEventListener('resize', drawClearanceChart);
  $('#col-chart').addEventListener('click', e => {
    if (!colData || !anim.n) return;
    const r = e.target.getBoundingClientRect(); const f = (e.clientX - r.left - 44) / (r.width - 54);
    stopAnim(); anim.pos = Math.max(0, Math.min(anim.n - 1, Math.round(f * (anim.n - 1)))); poseAt(anim.pos); drawClearanceChart();
  });
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
    const cn = $('#clear-now');
    if (colData) {
      const c = colData.clearance[i];
      if (c > 1e8) { cn.textContent = ''; }
      else {
        const w = colData.worst[i] >= 0 && colData.pairs.length ? '' : '';
        cn.textContent = `prześwit ${c.toFixed(1)} mm` + (c <= 0 ? ' · KOLIZJA' : (c < colData.margin ? ' · poniżej prześwitu' : ''));
        cn.className = 'mono ' + (c <= 0 ? 'bad' : (c < colData.margin ? 'warn' : 'ok'));
      }
      if (document.querySelector('#pane-col').classList.contains('active')) drawClearanceChart();
    } else cn.textContent = '';
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
  $('#show-obstacles').addEventListener('change', e => robotView && robotView.setObstaclesVisible(e.target.checked));
  $('#show-capsules').addEventListener('change', e => robotView && robotView.setCapsulesVisible(e.target.checked));
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
    if (t.dataset.tab === 'col') drawClearanceChart();
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
  syncFramesFromOptions();
  robotsReady.then(() => { loadOptions(); syncFramesFromOptions(); updateRobotModel(); updateAutoObstacles(); refreshMeshMeta(); updateToolMeshInfo(); });
  let savedTheme = 'dark';  // the brand look is dark; 'auto' and 'light' are available in the header
  try { savedTheme = localStorage.getItem('gcode2krl.theme') || 'dark'; } catch (e) {}
  setTheme(savedTheme, false);
  $$('.theme-switch button').forEach(b => b.addEventListener('click', () => setTheme(b.dataset.theme, true)));
  window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => setTheme(document.documentElement.getAttribute('data-theme') || 'auto', false));
  fetch('/api/version').then(r => r.json()).then(d => { $('#version').textContent = 'v' + d.version; }).catch(() => {});
})();
