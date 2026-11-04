"""Local web GUI server for gcode2krl (standard library only).

    gcode2krl-gui                 # starts http://127.0.0.1:8765 and opens the browser
    gcode2krl-gui --port 9000 --no-browser

API (JSON):
    GET  /api/examples               list of bundled example files
    GET  /api/example/<name>         example G-code text
    POST /api/translate              {gcode, filename, options} -> result
    GET  /api/zip/<token>            zip with the generated .src/.dat files
"""
from __future__ import annotations

import argparse
import base64
import io
import json
import math
import os
import socket
import sys
import threading
import time
import uuid
import webbrowser
import zipfile
from collections import OrderedDict
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote

import numpy as np

from . import __version__
from .gcode_parser import parse_gcode
from .krl_writer import KrlOptions, write_krl
from .translator import ArcItem, EventItem, LineItem, RapidItem, SplItem, TranslateOptions, translate

WEB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")
EXAMPLES_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "examples")
MAX_BODY = 200 * 1024 * 1024

_results: "OrderedDict[str, dict]" = OrderedDict()
_results_lock = threading.Lock()


def _remember(files: dict[str, str]) -> str:
    token = uuid.uuid4().hex
    with _results_lock:
        _results[token] = files
        while len(_results) > 16:
            _results.popitem(last=False)
    return token


def _f32(arr: list[float] | np.ndarray) -> str:
    return base64.b64encode(np.asarray(arr, dtype=np.float32).tobytes()).decode("ascii")


def _opt(options: dict, key: str, default, cast):
    val = options.get(key, default)
    if val is None or val == "":
        return default
    try:
        return cast(val)
    except (TypeError, ValueError):
        raise ValueError(f"invalid value for {key}: {val!r}")


def _triple(options: dict, key: str, default):
    val = options.get(key, default)
    if isinstance(val, str):
        val = [float(x) for x in val.replace(",", " ").split()]
    if len(val) != 3:
        raise ValueError(f"{key} needs three numbers")
    return tuple(float(x) for x in val)


def build_options(options: dict, filename: str) -> tuple[TranslateOptions, KrlOptions, dict]:
    topts = TranslateOptions(
        tol=_opt(options, "tol", 0.05, float),
        max_radius=_opt(options, "max_radius", 5000.0, float),
        min_radius=_opt(options, "min_radius", 0.1, float),
        min_arc_points=_opt(options, "min_arc_points", 4, int),
        refine_shift=_opt(options, "refine_shift", 3, int),
        spl_max_segment=_opt(options, "spl_max_segment", 3.0, float),
        corner_angle_deg=_opt(options, "corner_angle", 10.0, float),
        default_feed=_opt(options, "default_feed", 1000.0, float),
        offset=_triple(options, "offset", (0.0, 0.0, 0.0)),
        fit_arcs=_opt(options, "fit_arcs", True, bool),
        use_spl=_opt(options, "use_spl", True, bool),
    )
    base = os.path.splitext(os.path.basename(filename or "PROG"))[0]
    default_name = "".join(c if c.isalnum() else "_" for c in base).upper()[:20] or "PROG"
    if not default_name[0].isalpha():
        default_name = "P_" + default_name[:18]
    kopts = KrlOptions(
        name=_opt(options, "name", default_name, str) or default_name,
        mode=_opt(options, "mode", "spline", str),
        tool=_opt(options, "tool", 1, int),
        base=_opt(options, "base", 1, int),
        abc=_triple(options, "abc", (0.0, 180.0, 0.0)),
        rapid_vel=_opt(options, "rapid_vel", 0.5, float),
        max_vel=_opt(options, "max_vel", 2.0, float),
        acc=_opt(options, "acc", 2.0, float),
        cdis=_opt(options, "cdis", 0.2, float),
        corner_angle_deg=_opt(options, "corner_angle", 10.0, float),
        exact_corners=not _opt(options, "round_corners", False, bool),
        max_block_segments=_opt(options, "max_block_segments", 500, int),
        max_motions_per_file=_opt(options, "max_motions_per_file", 4000, int),
        block_approx=_opt(options, "block_approx", True, bool),
        spindle_out=_opt(options, "spindle_out", 0, int),
        coolant_out=_opt(options, "coolant_out", 0, int),
        spindle_anout=_opt(options, "spindle_anout", 0, int),
        spindle_max_rpm=_opt(options, "spindle_max_rpm", 24000.0, float),
        bco_ptp=_opt(options, "bco", True, bool),
        advance=_opt(options, "advance", 3, int),
        source_name=os.path.basename(filename or ""),
        halt_on_tool_change=_opt(options, "halt_tool_change", True, bool),
    )
    if kopts.mode not in ("spline", "lin"):
        raise ValueError("mode must be 'spline' or 'lin'")
    parser_kw = {
        "start": _triple(options, "start", (0.0, 0.0, 0.0)),
        "dwell_in_ms": _opt(options, "dwell_ms", False, bool),
    }
    return topts, kopts, parser_kw


def preview_geometry(parsed, items, offset) -> dict:
    """Flat float32 segment lists (x0,y0,z0,x1,y1,z1,...) per layer for WebGL."""
    from .gcode_parser import ArcMove, LinearMove
    from .geometry import sample_arc

    off = np.array(offset, dtype=float)
    inp: list[np.ndarray] = []
    inp_rapid: list[np.ndarray] = []
    for c in parsed.commands:
        if isinstance(c, LinearMove):
            (inp_rapid if c.rapid else inp).append(np.concatenate([c.start + off, c.end + off]))
        elif isinstance(c, ArcMove):
            from .geometry import arc_from_center
            fit = arc_from_center(c.start + off, c.end + off, c.center + off, c.normal, c.ccw)
            pts = sample_arc(fit, 0.02, min_points=8)
            if c.helical:
                rise = float((c.end - c.start) @ c.normal)
                pts = pts + np.outer(np.linspace(0, 1, len(pts)) * rise, c.normal)
            inp.append(np.column_stack([pts[:-1], pts[1:]]).ravel())
    layers: dict[str, list[np.ndarray]] = {"rapid": [], "slin": [], "scirc": [], "spl": []}
    arc_mids: list[np.ndarray] = []
    for it in items:
        if isinstance(it, RapidItem):
            layers["rapid"].append(np.concatenate([it.start, it.target]))
        elif isinstance(it, LineItem):
            layers["slin"].append(np.concatenate([it.start, it.end]))
        elif isinstance(it, ArcItem):
            pts = sample_arc(it.fit, 0.01, min_points=12)
            pts[0] = it.start
            pts[-1] = it.end
            layers["scirc"].append(np.column_stack([pts[:-1], pts[1:]]).ravel())
            arc_mids.append(it.mid)
        elif isinstance(it, SplItem):
            p = it.points
            layers["spl"].append(np.column_stack([p[:-1], p[1:]]).ravel())

    def pack(chunks):
        return _f32(np.concatenate(chunks) if chunks else np.zeros(0))

    allpts = [np.asarray(x).reshape(-1, 3) for x in inp + inp_rapid]
    if allpts:
        a = np.concatenate(allpts)
        bounds = {"min": a.min(axis=0).tolist(), "max": a.max(axis=0).tolist()}
    else:
        bounds = {"min": [0, 0, 0], "max": [1, 1, 1]}
    return {
        "input": pack(inp),
        "input_rapid": pack(inp_rapid),
        "rapid": pack(layers["rapid"]),
        "slin": pack(layers["slin"]),
        "scirc": pack(layers["scirc"]),
        "spl": pack(layers["spl"]),
        "arc_points": pack(arc_mids),
        "bounds": bounds,
    }


def run_translation(gcode: str, filename: str, options: dict) -> dict:
    t0 = time.perf_counter()
    topts, kopts, parser_kw = build_options(options or {}, filename)
    parsed = parse_gcode(gcode, **parser_kw)
    t1 = time.perf_counter()
    items, stats = translate(parsed, topts)
    t2 = time.perf_counter()
    prog = write_krl(items, kopts, stats)
    t3 = time.perf_counter()
    token = _remember(prog.files)
    ic = stats.input_counts
    n_in = sum(ic.values())
    result = {
        "token": token,
        "name": kopts.name,
        "files": prog.files,
        "main_file": f"{kopts.name}.src",
        "stats": {
            "input_moves": n_in,
            "input_counts": ic,
            "input_lines": parsed.n_lines,
            "chains": stats.chains,
            "chain_vertices": stats.chain_vertices,
            "lines_out": stats.lines_out,
            "arcs_out": stats.arcs_out,
            "spl_points": stats.spl_points,
            "spl_runs": stats.spl_runs,
            "rapids": stats.rapids,
            "motions_out": stats.motions_out,
            "compression": (n_in / stats.motions_out) if stats.motions_out else 0.0,
            "max_dev": stats.max_dev,
            "tol": topts.tol,
            "time_parse": t1 - t0,
            "time_fit": t2 - t1,
            "time_write": t3 - t2,
            "time_total": t3 - t0,
        },
        "warnings": stats.warnings,
        "preview": preview_geometry(parsed, items, topts.offset),
    }
    return result


class Handler(SimpleHTTPRequestHandler):
    server_version = f"gcode2krl/{__version__}"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=WEB_DIR, **kwargs)

    def log_message(self, fmt, *args):  # quieter
        if getattr(self.server, "verbose", False):
            super().log_message(fmt, *args)

    # ------------------------------------------------------------ helpers
    def send_json(self, obj, status: int = 200) -> None:
        data = json.dumps(obj).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def send_bytes(self, data: bytes, ctype: str, filename: str | None = None) -> None:
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        if filename:
            self.send_header("Content-Disposition", f'attachment; filename="{filename}"')
        self.end_headers()
        self.wfile.write(data)

    def read_json(self) -> dict:
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0 or length > MAX_BODY:
            raise ValueError("request body missing or too large")
        return json.loads(self.rfile.read(length).decode("utf-8"))

    # --------------------------------------------------------------- GET
    def do_GET(self) -> None:
        path = unquote(self.path.split("?", 1)[0])
        if path == "/api/examples":
            names = sorted(f for f in os.listdir(EXAMPLES_DIR) if f.lower().endswith((".nc", ".gcode", ".tap", ".ngc")))\
                if os.path.isdir(EXAMPLES_DIR) else []
            self.send_json({"examples": names})
            return
        if path.startswith("/api/example/"):
            name = os.path.basename(path[len("/api/example/"):])
            full = os.path.join(EXAMPLES_DIR, name)
            if not os.path.isfile(full):
                self.send_json({"error": "not found"}, 404)
                return
            with open(full, "r", encoding="utf-8", errors="replace") as fh:
                self.send_json({"name": name, "gcode": fh.read()})
            return
        if path.startswith("/api/zip/"):
            token = path[len("/api/zip/"):].strip("/")
            with _results_lock:
                files = _results.get(token)
            if files is None:
                self.send_json({"error": "unknown or expired token"}, 404)
                return
            buf = io.BytesIO()
            with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as zf:
                for fname, content in files.items():
                    zf.writestr(fname, content.replace("\n", "\r\n"))
            main = next((f[:-4] for f in files if f.endswith(".src") and "_" not in f[-8:-4]), "krl")
            self.send_bytes(buf.getvalue(), "application/zip", f"{main}.zip")
            return
        if path == "/api/version":
            self.send_json({"version": __version__})
            return
        if path == "/":
            self.path = "/index.html"
        super().do_GET()

    # -------------------------------------------------------------- POST
    def do_POST(self) -> None:
        path = unquote(self.path.split("?", 1)[0])
        if path != "/api/translate":
            self.send_json({"error": "not found"}, 404)
            return
        try:
            body = self.read_json()
            gcode = body.get("gcode", "")
            if not isinstance(gcode, str) or not gcode.strip():
                raise ValueError("no G-code supplied")
            result = run_translation(gcode, body.get("filename", "PROG"), body.get("options") or {})
        except ValueError as exc:
            self.send_json({"error": str(exc)}, 400)
            return
        except Exception as exc:  # noqa: BLE001 - report to the UI
            self.send_json({"error": f"{type(exc).__name__}: {exc}"}, 500)
            return
        self.send_json(result)


def free_port(preferred: int) -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.bind(("127.0.0.1", preferred))
            return preferred
        except OSError:
            s.bind(("127.0.0.1", 0))
            return s.getsockname()[1]


def make_server(host: str = "127.0.0.1", port: int = 0, verbose: bool = False) -> ThreadingHTTPServer:
    srv = ThreadingHTTPServer((host, port), Handler)
    srv.daemon_threads = True
    srv.verbose = verbose  # type: ignore[attr-defined]
    return srv


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="gcode2krl-gui", description="Local web GUI for gcode2krl")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--no-browser", action="store_true")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args(argv)
    port = free_port(args.port) if args.host == "127.0.0.1" else args.port
    srv = make_server(args.host, port, args.verbose)
    url = f"http://{args.host}:{srv.server_address[1]}/"
    print(f"gcode2krl GUI: {url}  (Ctrl+C to stop)")
    if not args.no_browser:
        threading.Timer(0.4, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
