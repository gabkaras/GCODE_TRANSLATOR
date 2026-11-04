"""gcode2krl - translate G-code tool paths into KUKA KRL programs.

Pipeline:
    G-code text -> gcode_parser (modal parser) -> list of moves/events
    -> translator (arc reconstruction, SPL grouping) -> motion items
    -> krl_writer (SPLINE/SLIN/SCIRC/SPL or LIN/CIRC) -> .src/.dat files
"""

__version__ = "0.1.0"
