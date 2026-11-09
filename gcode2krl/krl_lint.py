"""Static check of a generated KRL program against the KSS spline rules that make the
controller stop or complain ("Velocity 0 m/s in point ... (SLIN-SCIRC sequence) /
(broken XYZ path)", "Approximation inadmissible"):

* inside a SPLINE block two SLIN / SCIRC segments must join tangentially,
* a point must not be repeated (zero-length segment),
* ENDSPLINE C_SPL may only approximate into a following CP motion.
"""
from __future__ import annotations

import math
import re

import numpy as np

_FRAME = re.compile(r"\{X ([-\d.]+), Y ([-\d.]+), Z ([-\d.]+)")
_CP_NEXT = ("SPLINE", "LIN", "CIRC")


def _frames(line: str) -> list[np.ndarray]:
    return [np.array([float(x) for x in m]) for m in _FRAME.findall(line)]


def _angle(a: np.ndarray, b: np.ndarray) -> float:
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na < 1e-12 or nb < 1e-12:
        return 0.0
    return math.degrees(math.acos(max(-1.0, min(1.0, float(a @ b) / (na * nb)))))


def lint_krl(text: str, tangent_deg: float = 0.1) -> list[tuple[int, str]]:
    """Return [(line_no, message)] for the problems above."""
    lines = text.splitlines()
    pos = None
    in_block = False
    t_prev = None
    kind_prev = None
    out: list[tuple[int, str]] = []
    for i, raw in enumerate(lines, 1):
        s = raw.strip()
        if not s or s.startswith(";"):
            continue
        w = s.split()[0]
        if w == "SPLINE":
            in_block, t_prev, kind_prev = True, None, None
            continue
        if w == "ENDSPLINE":
            if "C_SPL" in s:
                nxt = ""
                for x in lines[i:]:
                    xs = x.strip()
                    if not xs or xs.startswith(";") or xs.startswith("TRIGGER"):
                        continue
                    nxt = xs
                    break
                if not nxt.startswith(_CP_NEXT):
                    out.append((i, f"ENDSPLINE C_SPL przed instrukcja bez ruchu CP: {nxt[:40] or 'koniec'}"))
            in_block = False
            continue
        if in_block and w in ("SLIN", "SCIRC", "SPL"):
            f = _frames(s)
            if not f:
                continue
            if w == "SCIRC" and len(f) == 2 and pos is not None:
                a, b, c = pos, f[0], f[1]
                n = np.cross(b - a, c - a)
                nn = float(np.linalg.norm(n))
                if nn < 1e-9:
                    t_in = t_out = c - a
                else:
                    ab, ac = b - a, c - a
                    cen = a + (np.cross(n, ab) * float(ac @ ac) + np.cross(ac, n) * float(ab @ ab)) / (2.0 * nn * nn)
                    t_in, t_out = np.cross(n, a - cen), np.cross(n, c - cen)
                end = c
            else:
                end = f[-1]
                t_in = t_out = (end - pos) if pos is not None else None
            if pos is not None and float(np.linalg.norm(end - pos)) < 1e-6:
                out.append((i, f"powtorzony punkt w bloku SPLINE ({w})"))
            if (t_prev is not None and t_in is not None and kind_prev in ("SLIN", "SCIRC") and w in ("SLIN", "SCIRC")):
                ang = _angle(t_prev, t_in)
                if ang > tangent_deg:
                    out.append((i, f"niestyczne zlacze {kind_prev}-{w} w bloku SPLINE: {ang:.2f} deg (robot zatrzyma sie w punkcie)"))
            t_prev, kind_prev, pos = t_out, w, end
        elif w in ("LIN", "PTP", "CIRC"):
            f = _frames(s)
            if f:
                pos = f[-1]
            t_prev = kind_prev = None
    return out


def lint_files(files: dict[str, str], tangent_deg: float = 0.1) -> list[str]:
    """Short warnings per file (for the stats / GUI)."""
    msgs: list[str] = []
    for name, text in files.items():
        if not name.lower().endswith(".src"):
            continue
        issues = lint_krl(text, tangent_deg)
        if not issues:
            continue
        kinds: dict[str, int] = {}
        for _, m in issues:
            k = m.split(":")[0]
            kinds[k] = kinds.get(k, 0) + 1
        first = issues[0]
        msgs.append(f"KSS {name}: " + ", ".join(f"{v}x {k}" for k, v in kinds.items()) + f" (pierwsze: linia {first[0]})")
    return msgs


def main(argv=None) -> int:
    import sys
    args = sys.argv[1:] if argv is None else argv
    bad = 0
    for p in args:
        issues = lint_krl(open(p, encoding="utf-8", errors="replace").read())
        print(f"{p}: {len(issues)} problemow")
        for ln, m in issues[:30]:
            print(f"  linia {ln}: {m}")
        bad += len(issues)
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
