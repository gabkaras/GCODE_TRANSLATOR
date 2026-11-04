/* Lightweight WebGL path viewer: layers of line segments, orbit/pan/zoom camera,
   grid and axis triad. No dependencies. */
'use strict';

const VS = `
attribute vec3 aPos;
uniform mat4 uMVP;
uniform float uPointSize;
void main() {
  gl_Position = uMVP * vec4(aPos, 1.0);
  gl_PointSize = uPointSize;
}`;
const FS = `
precision mediump float;
uniform vec4 uColor;
void main() { gl_FragColor = uColor; }`;
const MVS = `
attribute vec3 aPos;
attribute vec3 aNormal;
uniform mat4 uMVP;
uniform mat4 uModel;
varying vec3 vN;
varying vec3 vP;
void main() {
  vec4 wp = uModel * vec4(aPos, 1.0);
  vP = wp.xyz;
  vN = mat3(uModel) * aNormal;
  gl_Position = uMVP * wp;
}`;
const MFS = `
precision mediump float;
uniform vec4 uColor;
uniform vec3 uLight;
uniform vec3 uEye;
varying vec3 vN;
varying vec3 vP;
void main() {
  vec3 n = normalize(vN);
  float d = max(dot(n, normalize(uLight)), 0.0);
  float d2 = max(dot(n, normalize(vec3(-0.3, -0.5, 0.8))), 0.0);
  vec3 v = normalize(uEye - vP);
  float spec = pow(max(dot(normalize(normalize(uLight) + v), n), 0.0), 24.0) * 0.25;
  vec3 col = uColor.rgb * (0.28 + 0.6 * d + 0.25 * d2) + spec;
  gl_FragColor = vec4(col, uColor.a);
}`;

const M4 = {
  identity() { return new Float32Array([1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]); },
  mul(a, b) {
    const o = new Float32Array(16);
    for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) {
      o[c*4+r] = a[r]*b[c*4] + a[4+r]*b[c*4+1] + a[8+r]*b[c*4+2] + a[12+r]*b[c*4+3];
    }
    return o;
  },
  perspective(fovy, aspect, near, far) {
    const f = 1 / Math.tan(fovy / 2), nf = 1 / (near - far);
    return new Float32Array([f/aspect,0,0,0, 0,f,0,0, 0,0,(far+near)*nf,-1, 0,0,2*far*near*nf,0]);
  },
  ortho(l, r, b, t, n, f) {
    return new Float32Array([2/(r-l),0,0,0, 0,2/(t-b),0,0, 0,0,-2/(f-n),0, -(r+l)/(r-l),-(t+b)/(t-b),-(f+n)/(f-n),1]);
  },
  lookAt(eye, target, up) {
    const z = norm3(sub3(eye, target));
    const x = norm3(cross3(up, z));
    const y = cross3(z, x);
    return new Float32Array([
      x[0], y[0], z[0], 0,
      x[1], y[1], z[1], 0,
      x[2], y[2], z[2], 0,
      -dot3(x, eye), -dot3(y, eye), -dot3(z, eye), 1]);
  },
};
function sub3(a, b) { return [a[0]-b[0], a[1]-b[1], a[2]-b[2]]; }
function add3(a, b) { return [a[0]+b[0], a[1]+b[1], a[2]+b[2]]; }
function scale3(a, s) { return [a[0]*s, a[1]*s, a[2]*s]; }
function dot3(a, b) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
function cross3(a, b) { return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]; }
function norm3(a) { const l = Math.hypot(a[0], a[1], a[2]) || 1; return [a[0]/l, a[1]/l, a[2]/l]; }

function hexToRgba(hex, alpha) {
  const v = parseInt(hex.slice(1), 16);
  return [((v >> 16) & 255) / 255, ((v >> 8) & 255) / 255, (v & 255) / 255, alpha === undefined ? 1 : alpha];
}

class PathViewer {
  constructor(canvas) {
    this.canvas = canvas;
    const gl = canvas.getContext('webgl', { antialias: true, alpha: false, depth: true, preserveDrawingBuffer: true });
    if (!gl) throw new Error('WebGL is not available');
    this.gl = gl;
    this.prog = this._program(VS, FS);
    this.aPos = gl.getAttribLocation(this.prog, 'aPos');
    this.uMVP = gl.getUniformLocation(this.prog, 'uMVP');
    this.uColor = gl.getUniformLocation(this.prog, 'uColor');
    this.uPointSize = gl.getUniformLocation(this.prog, 'uPointSize');
    this.mprog = this._program(MVS, MFS);
    this.m = {
      aPos: gl.getAttribLocation(this.mprog, 'aPos'), aNormal: gl.getAttribLocation(this.mprog, 'aNormal'),
      uMVP: gl.getUniformLocation(this.mprog, 'uMVP'), uModel: gl.getUniformLocation(this.mprog, 'uModel'),
      uColor: gl.getUniformLocation(this.mprog, 'uColor'), uLight: gl.getUniformLocation(this.mprog, 'uLight'),
      uEye: gl.getUniformLocation(this.mprog, 'uEye'),
    };
    this.meshes = new Map();   // name -> {pbuf, nbuf, count, color, model, visible, group}
    this.layers = new Map();   // name -> {buf, count, color, visible, mode, order, size}
    this.bounds = { min: [0, 0, 0], max: [100, 100, 10] };
    this.cam = { target: [50, 50, 0], dist: 250, yaw: -0.6, pitch: 0.9, ortho: false, fov: Math.PI / 4 };
    this.bg = [0.07, 0.08, 0.1, 1];
    this.gridColor = hexToRgba('#2a3140');
    this.gridMajor = hexToRgba('#3a4357');
    this._dirty = true;
    this._raf = null;
    this.onChange = null;
    this._grid = null;
    this._axes = null;
    this._bindEvents();
    this._resize();
    new ResizeObserver(() => { this._resize(); this.invalidate(); }).observe(canvas);
  }

  _program(vs, fs) {
    const gl = this.gl;
    const mk = (type, src) => {
      const s = gl.createShader(type); gl.shaderSource(s, src); gl.compileShader(s);
      if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(s));
      return s;
    };
    const p = gl.createProgram();
    gl.attachShader(p, mk(gl.VERTEX_SHADER, vs)); gl.attachShader(p, mk(gl.FRAGMENT_SHADER, fs));
    gl.linkProgram(p);
    if (!gl.getProgramParameter(p, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(p));
    return p;
  }

  _resize() {
    const dpr = window.devicePixelRatio || 1;
    const w = Math.max(1, Math.floor(this.canvas.clientWidth * dpr));
    const h = Math.max(1, Math.floor(this.canvas.clientHeight * dpr));
    if (this.canvas.width !== w || this.canvas.height !== h) { this.canvas.width = w; this.canvas.height = h; }
  }

  /** data: Float32Array of xyz pairs (segments) or xyz (points). mode: 'lines' | 'points' */
  setLayer(name, data, color, opts) {
    const gl = this.gl;
    opts = opts || {};
    let L = this.layers.get(name);
    if (!L) { L = { buf: gl.createBuffer(), visible: true, order: this.layers.size }; this.layers.set(name, L); }
    gl.bindBuffer(gl.ARRAY_BUFFER, L.buf);
    gl.bufferData(gl.ARRAY_BUFFER, data, gl.STATIC_DRAW);
    L.count = data.length / 3;
    L.color = typeof color === 'string' ? hexToRgba(color, opts.alpha) : color;
    L.mode = opts.mode || 'lines';
    L.size = opts.size || 6;
    if (opts.order !== undefined) L.order = opts.order;
    if (opts.visible !== undefined) L.visible = opts.visible;
    if (opts.group) L.group = opts.group;
    this.invalidate();
  }
  setVisible(name, v) { const L = this.layers.get(name); if (L) { L.visible = v; this.invalidate(); } }

  /** Shaded triangle mesh. geom: {positions: Float32Array, normals: Float32Array}; model: mat4 (column-major). */
  setMesh(name, geom, color, model, opts) {
    const gl = this.gl;
    opts = opts || {};
    let M = this.meshes.get(name);
    if (!M) { M = { pbuf: gl.createBuffer(), nbuf: gl.createBuffer(), visible: true }; this.meshes.set(name, M); }
    if (geom) {
      gl.bindBuffer(gl.ARRAY_BUFFER, M.pbuf); gl.bufferData(gl.ARRAY_BUFFER, geom.positions, gl.STATIC_DRAW);
      gl.bindBuffer(gl.ARRAY_BUFFER, M.nbuf); gl.bufferData(gl.ARRAY_BUFFER, geom.normals, gl.STATIC_DRAW);
      M.count = geom.positions.length / 3;
    }
    if (color) M.color = typeof color === 'string' ? hexToRgba(color, opts.alpha) : color;
    if (model) M.model = model;
    if (opts.group) M.group = opts.group;
    if (opts.visible !== undefined) M.visible = opts.visible;
    this.invalidate();
  }
  setMeshTransform(name, model) { const M = this.meshes.get(name); if (M) { M.model = model; this.invalidate(); } }
  setGroupVisible(group, v) {
    for (const M of this.meshes.values()) if (M.group === group) M.visible = v;
    for (const [n, L] of this.layers) if (L.group === group) L.visible = v;
    this.invalidate();
  }
  removeGroup(group) {
    for (const [n, M] of [...this.meshes]) if (M.group === group) { this.gl.deleteBuffer(M.pbuf); this.gl.deleteBuffer(M.nbuf); this.meshes.delete(n); }
    for (const [n, L] of [...this.layers]) if (L.group === group) { this.gl.deleteBuffer(L.buf); this.layers.delete(n); }
    this.invalidate();
  }
  clearLayers() { for (const L of this.layers.values()) this.gl.deleteBuffer(L.buf); this.layers.clear(); this.invalidate(); }

  setBounds(min, max) {
    this.bounds = { min: min.slice(), max: max.slice() };
    this._buildGrid();
  }

  fit() {
    const b = this.bounds;
    const c = scale3(add3(b.min, b.max), 0.5);
    const r = Math.max(1e-3, Math.hypot(b.max[0]-b.min[0], b.max[1]-b.min[1], b.max[2]-b.min[2]) / 2);
    this.cam.target = c;
    this.cam.dist = r / Math.sin(this.cam.fov / 2) * 1.15;
    this.invalidate();
  }
  viewTop() { this.cam.yaw = 0; this.cam.pitch = Math.PI / 2 - 1e-4; this.fit(); }
  viewIso() { this.cam.yaw = -0.6; this.cam.pitch = 0.9; this.fit(); }
  viewFront() { this.cam.yaw = 0; this.cam.pitch = 1e-3; this.fit(); }
  toggleOrtho() { this.cam.ortho = !this.cam.ortho; this.invalidate(); return this.cam.ortho; }

  _buildGrid() {
    const b = this.bounds;
    const span = Math.max(b.max[0]-b.min[0], b.max[1]-b.min[1], 1);
    const step = Math.pow(10, Math.floor(Math.log10(span / 8)));
    const nice = span / step > 50 ? step * 5 : (span / step > 20 ? step * 2 : step);
    const pad = nice * 2;
    const x0 = Math.floor((b.min[0]-pad)/nice)*nice, x1 = Math.ceil((b.max[0]+pad)/nice)*nice;
    const y0 = Math.floor((b.min[1]-pad)/nice)*nice, y1 = Math.ceil((b.max[1]+pad)/nice)*nice;
    const z = b.min[2];
    const minor = [], major = [];
    for (let x = x0; x <= x1 + 1e-9; x += nice) (Math.abs(x / (nice*5) - Math.round(x/(nice*5))) < 1e-6 ? major : minor).push(x, y0, z, x, y1, z);
    for (let y = y0; y <= y1 + 1e-9; y += nice) (Math.abs(y / (nice*5) - Math.round(y/(nice*5))) < 1e-6 ? major : minor).push(x0, y, z, x1, y, z);
    this.setLayer('_grid', new Float32Array(minor), this.gridColor, { order: -20 });
    this.setLayer('_gridMajor', new Float32Array(major), this.gridMajor, { order: -19 });
    const a = Math.max(nice * 2, span * 0.08);
    this.setLayer('_axX', new Float32Array([0,0,z, a,0,z]), '#e5484d', { order: -18 });
    this.setLayer('_axY', new Float32Array([0,0,z, 0,a,z]), '#46a758', { order: -17 });
    this.setLayer('_axZ', new Float32Array([0,0,z, 0,0,z+a]), '#3e8cff', { order: -16 });
    this.gridStep = nice;
  }

  _eye() {
    const c = this.cam;
    const cp = Math.cos(c.pitch), sp = Math.sin(c.pitch);
    return [c.target[0] + c.dist * cp * Math.sin(c.yaw), c.target[1] - c.dist * cp * Math.cos(c.yaw), c.target[2] + c.dist * sp];
  }

  _mvp() {
    const w = this.canvas.width, h = this.canvas.height, aspect = w / Math.max(1, h);
    const c = this.cam;
    const eye = this._eye();
    const view = M4.lookAt(eye, c.target, [0, 0, 1]);
    const near = Math.max(c.dist * 0.001, 0.01), far = c.dist * 50;
    let proj;
    if (c.ortho) {
      const hh = c.dist * Math.tan(c.fov / 2);
      proj = M4.ortho(-hh * aspect, hh * aspect, -hh, hh, -far, far);
    } else {
      proj = M4.perspective(c.fov, aspect, near, far);
    }
    return M4.mul(proj, view);
  }

  invalidate() {
    this._dirty = true;
    if (this._raf === null) this._raf = requestAnimationFrame(() => { this._raf = null; this.render(); });
  }

  render() {
    const gl = this.gl;
    this._resize();
    gl.viewport(0, 0, this.canvas.width, this.canvas.height);
    gl.clearColor(...this.bg);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    gl.enable(gl.BLEND);
    gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
    gl.enable(gl.DEPTH_TEST);
    gl.depthFunc(gl.LEQUAL);
    const mvp = this._mvp();
    const meshes = [...this.meshes.values()].filter(M => M.visible && M.count > 0 && M.model)
      .sort((a, b) => (a.color[3] < 1 ? 1 : 0) - (b.color[3] < 1 ? 1 : 0));
    if (meshes.length) {
      gl.useProgram(this.mprog);
      gl.uniformMatrix4fv(this.m.uMVP, false, mvp);
      gl.uniform3fv(this.m.uLight, [0.4, -0.6, 1.0]);
      gl.uniform3fv(this.m.uEye, this._eye());
      gl.enable(gl.CULL_FACE);
      gl.cullFace(gl.BACK);
      for (const M of meshes) {
        const translucent = M.color[3] < 1;
        gl.depthMask(!translucent);
        if (translucent) gl.disable(gl.CULL_FACE); else gl.enable(gl.CULL_FACE);
        gl.bindBuffer(gl.ARRAY_BUFFER, M.pbuf);
        gl.enableVertexAttribArray(this.m.aPos);
        gl.vertexAttribPointer(this.m.aPos, 3, gl.FLOAT, false, 0, 0);
        gl.bindBuffer(gl.ARRAY_BUFFER, M.nbuf);
        gl.enableVertexAttribArray(this.m.aNormal);
        gl.vertexAttribPointer(this.m.aNormal, 3, gl.FLOAT, false, 0, 0);
        gl.uniformMatrix4fv(this.m.uModel, false, M.model);
        gl.uniform4fv(this.m.uColor, M.color);
        gl.drawArrays(gl.TRIANGLES, 0, M.count);
      }
      gl.disable(gl.CULL_FACE);
      gl.depthMask(true);
      gl.disableVertexAttribArray(this.m.aNormal);
    }
    gl.useProgram(this.prog);
    gl.uniformMatrix4fv(this.uMVP, false, mvp);
    const layers = [...this.layers.values()].filter(L => L.visible && L.count > 0).sort((a, b) => a.order - b.order);
    for (const L of layers) {
      gl.bindBuffer(gl.ARRAY_BUFFER, L.buf);
      gl.enableVertexAttribArray(this.aPos);
      gl.vertexAttribPointer(this.aPos, 3, gl.FLOAT, false, 0, 0);
      gl.uniform4fv(this.uColor, L.color);
      gl.uniform1f(this.uPointSize, L.size * (window.devicePixelRatio || 1));
      gl.drawArrays(L.mode === 'points' ? gl.POINTS : gl.LINES, 0, L.count);
    }
    this._dirty = false;
    if (this.onChange) this.onChange(this);
  }

  _bindEvents() {
    const cv = this.canvas;
    let drag = null;
    cv.addEventListener('contextmenu', e => e.preventDefault());
    cv.addEventListener('pointerdown', e => {
      cv.setPointerCapture(e.pointerId);
      drag = { x: e.clientX, y: e.clientY, mode: (e.button === 2 || e.button === 1 || e.shiftKey) ? 'pan' : 'orbit' };
    });
    cv.addEventListener('pointermove', e => {
      if (!drag) return;
      const dx = e.clientX - drag.x, dy = e.clientY - drag.y;
      drag.x = e.clientX; drag.y = e.clientY;
      const c = this.cam;
      if (drag.mode === 'orbit') {
        c.yaw += dx * 0.008;
        c.pitch = Math.max(-Math.PI/2 + 1e-3, Math.min(Math.PI/2 - 1e-3, c.pitch + dy * 0.008));
      } else {
        const eye = this._eye();
        const z = norm3(sub3(eye, c.target));
        const x = norm3(cross3([0, 0, 1], z));
        const y = cross3(z, x);
        const k = 2 * c.dist * Math.tan(c.fov / 2) / cv.clientHeight;
        c.target = add3(c.target, add3(scale3(x, -dx * k), scale3(y, dy * k)));
      }
      this.invalidate();
    });
    const end = e => { drag = null; };
    cv.addEventListener('pointerup', end); cv.addEventListener('pointercancel', end);
    cv.addEventListener('wheel', e => {
      e.preventDefault();
      const f = Math.exp(e.deltaY * 0.0015);
      this.cam.dist = Math.max(0.05, Math.min(1e6, this.cam.dist * f));
      this.invalidate();
    }, { passive: false });
    cv.addEventListener('dblclick', () => this.fit());
    // pinch zoom
    const touches = new Map();
    cv.addEventListener('touchstart', e => { for (const t of e.changedTouches) touches.set(t.identifier, [t.clientX, t.clientY]); }, { passive: true });
    cv.addEventListener('touchmove', e => {
      if (e.touches.length === 2) {
        e.preventDefault();
        const [a, b] = [e.touches[0], e.touches[1]];
        const pa = touches.get(a.identifier), pb = touches.get(b.identifier);
        if (pa && pb) {
          const d0 = Math.hypot(pa[0]-pb[0], pa[1]-pb[1]), d1 = Math.hypot(a.clientX-b.clientX, a.clientY-b.clientY);
          if (d0 > 0) { this.cam.dist = Math.max(0.05, this.cam.dist * d0 / d1); this.invalidate(); }
        }
        for (const t of e.touches) touches.set(t.identifier, [t.clientX, t.clientY]);
      }
    }, { passive: false });
    cv.addEventListener('touchend', e => { for (const t of e.changedTouches) touches.delete(t.identifier); }, { passive: true });
  }
}

window.PathViewer = PathViewer;
window.M4 = M4;
