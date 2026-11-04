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
  ];

  let viewer;
  try { viewer = new PathViewer($('#view')); }
  catch (e) { $('#viewer-error').textContent = 'WebGL niedostępny: ' + e.message; }

  // ----------------------------------------------------------------- options
  const OPTION_IDS = ['tol','max_radius','min_radius','min_arc_points','refine_shift','spl_max_segment','corner_angle',
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
        const arr = decode(p[L.key]);
        viewer.setLayer(L.key, arr, L.color, { alpha: L.alpha, order: L.order, mode: L.mode, size: L.size, visible: $('#layer-' + L.key).checked });
      }
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
  fetch('/api/version').then(r => r.json()).then(d => { $('#version').textContent = 'v' + d.version; }).catch(() => {});
})();
