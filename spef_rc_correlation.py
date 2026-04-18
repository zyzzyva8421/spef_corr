#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations

"""
SPEF RC Correlation Tool - C++ accelerated Python wrapper.

All heavy computation (parsing, resistance calculation, correlation, backmark)
is delegated to the C++ extension (spef_core) for maximum performance.

Python provides:
- CLI argument parsing and command dispatch
- GUI (Tkinter) interface
- Data file I/O and CSV output
- Simple statistics (Pearson correlation)

Usage:
    python spef_rc_correlation.py spef1.spef spef2.spef --csv-prefix output
    python spef_rc_correlation.py --gui
    python spef_rc_correlation.py --backmark input.spef --net-cap-data caps.data --output output.spef
"""

import argparse
import csv
import math
import os
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional, Iterable
try:
    import spef_core
    HAS_CPP = True
except ImportError:
    HAS_CPP = False
    print("[warn] C++ extension not available, using Python fallback (slower)")
try:
    import tkinter as tk
    from tkinter import filedialog, messagebox, ttk, simpledialog
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure
except ImportError:
    tk = None
    FigureCanvasTkAgg = None
    Figure = None
    # GUI will fail gracefully if not available

# ===================== Python data structures for GUI =====================

@dataclass
class CapComparison:
    net: str
    c1: float
    c2: float

@dataclass
class CouplingCapComparison:
    """Compare coupling capacitance between two nets across two SPEF files."""
    net1: str  # First net name
    net2: str  # Second net name
    c1: float  # Coupling cap in spef1
    c2: float  # Coupling cap in spef2

@dataclass
class ResComparison:
    net: str
    driver: str
    load: str
    r1: float
    r2: float

@dataclass
class NetRC:
    name: str
    total_cap: float
    driver: Optional[str] = None
    sinks: List[str] = None
    res_graph: Dict[str, List[Tuple[str, float]]] = None
    def __post_init__(self):
        if self.sinks is None:
            self.sinks = []
        if self.res_graph is None:
            self.res_graph = {}


def pearson_corr(xs: Iterable[float], ys: Iterable[float]) -> Optional[float]:
    """Compute Pearson correlation coefficient."""
    xs = list(xs)
    ys = list(ys)
    if len(xs) != len(ys) or len(xs) == 0:
        return None
    n = len(xs)
    mean_x = sum(xs) / n
    mean_y = sum(ys) / n
    num = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
    den_x = math.sqrt(sum((x - mean_x) ** 2 for x in xs))
    den_y = math.sqrt(sum((y - mean_y) ** 2 for y in ys))
    if den_x == 0.0 or den_y == 0.0:
        return None
    return num / (den_x * den_y)


# ===================== Data file functions =====================

def parse_net_cap_data(path: str) -> List[CapComparison]:
    """Parse net_cap.data file."""
    caps: List[CapComparison] = []
    with open(path, 'r', encoding='utf-8') as f:
        for lineno, line in enumerate(f, 1):
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) < 3:
                print(f"[warn] {path}:{lineno}: expected 3 fields, got {len(parts)}, skipping")
                continue
            try:
                caps.append(CapComparison(parts[0], float(parts[1]), float(parts[2])))
            except ValueError:
                print(f"[warn] {path}:{lineno}: non-numeric value, skipping")
    return caps


def parse_net_res_data(path: str) -> List[ResComparison]:
    """Parse net_res.data file."""
    ress: List[ResComparison] = []
    with open(path, 'r', encoding='utf-8') as f:
        for lineno, line in enumerate(f, 1):
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) < 5:
                print(f"[warn] {path}:{lineno}: expected 5 fields, skipping")
                continue
            try:
                ress.append(ResComparison(parts[0], parts[1], parts[2], float(parts[3]), float(parts[4])))
            except ValueError:
                print(f"[warn] {path}:{lineno}: non-numeric value, skipping")
    return ress


def parse_net_ccap_data(path: str) -> List[CouplingCapComparison]:
    """Parse net_ccap.data file."""
    caps: List[CouplingCapComparison] = []
    with open(path, 'r', encoding='utf-8') as f:
        for lineno, line in enumerate(f, 1):
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) < 4:
                print(f"[warn] {path}:{lineno}: expected 4 fields, got {len(parts)}, skipping")
                continue
            try:
                caps.append(CouplingCapComparison(parts[0], parts[1], float(parts[2]), float(parts[3])))
            except ValueError:
                print(f"[warn] {path}:{lineno}: non-numeric value, skipping")
    return caps


def write_caps_csv(path: str, caps: List[CapComparison]) -> None:
    """Write capacitance comparisons to CSV."""
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["net", "C_tool1", "C_tool2", "ratio", "delta"])
        for row in caps:
            ratio = (row.c2 / row.c1) if row.c1 != 0 else "inf"
            delta = row.c2 - row.c1
            w.writerow([row.net, row.c1, row.c2, ratio, delta])


def write_res_csv(path: str, ress: List[ResComparison], r_agg: str) -> None:
    """Write resistance comparisons to CSV."""
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["net", f"R_{r_agg}_tool1", f"R_{r_agg}_tool2", "ratio", "delta"])
        for row in ress:
            ratio = (row.r2 / row.r1) if row.r1 != 0 else "inf"
            delta = row.r2 - row.r1
            w.writerow([row.net, row.r1, row.r2, ratio, delta])


# ===================== C++ wrapper functions =====================

def compare_spef_cpp(spef1_path: str, spef2_path: str, num_threads: int = 0) -> Tuple[List[CapComparison], List[ResComparison], List[CapComparison], List[ResComparison]]:
    """Compare two SPEF files using C++ backend."""
    if not HAS_CPP:
        raise RuntimeError("C++ extension not available")
    
    print(f"Parsing {spef1_path} and {spef2_path}...")
    spef1 = spef_core.parse_spef(spef1_path)
    spef2 = spef_core.parse_spef(spef2_path)
    
    result = spef_core.compare_spef_full(spef1, spef2, num_threads)
    
    caps = [CapComparison(c.net_name, c.c1, c.c2) for c in result.cap_rows]
    ress = [ResComparison(r.net_name, r.driver, r.sink, r.r1, r.r2) for r in result.res_rows]
    top_10_cap = [CapComparison(c.net_name, c.c1, c.c2) for c in result.top_10_cap]
    top_10_res = [ResComparison(r.net_name, r.driver, r.sink, r.r1, r.r2) for r in result.top_10_res]
    
    return caps, ress, top_10_cap, top_10_res


def compare_spef_with_coupling_cpp(
    spef1_path: str,
    spef2_path: str,
    num_threads: int = 0,
) -> Tuple[List[CapComparison], List[ResComparison], List[CouplingCapComparison], List[CapComparison], List[ResComparison]]:
    """Compare total cap, resistance, and coupling capacitance using one shared parse."""
    if not HAS_CPP:
        raise RuntimeError("C++ extension not available")

    print(f"Parsing {spef1_path} and {spef2_path} in parallel (C++ threads)...")
    cpp_spefs = spef_core.parse_spef_parallel([spef1_path, spef2_path], max(num_threads, 2))
    result = spef_core.compare_spef_full(cpp_spefs[0], cpp_spefs[1], num_threads)
    cc_results = spef_core.compare_coupling_caps(cpp_spefs[0], cpp_spefs[1])

    caps = [CapComparison(c.net_name, c.c1, c.c2) for c in result.cap_rows]
    ress = [ResComparison(r.net_name, r.driver, r.sink, r.r1, r.r2) for r in result.res_rows]
    coupling_caps = [CouplingCapComparison(cc.net1, cc.net2, cc.c1, cc.c2) for cc in cc_results]
    top_10_cap = [CapComparison(c.net_name, c.c1, c.c2) for c in result.top_10_cap]
    top_10_res = [ResComparison(r.net_name, r.driver, r.sink, r.r1, r.r2) for r in result.top_10_res]

    return caps, ress, coupling_caps, top_10_cap, top_10_res


def compare_coupling_caps_cpp(spef1: "SpefFile", spef2: "SpefFile") -> List[CouplingCapComparison]:
    """Compare coupling capacitances between two SPEF files using C++ backend."""
    if not HAS_CPP:
        raise RuntimeError("C++ extension not available")
    
    if not hasattr(spef1, '_cpp_spef') or spef1._cpp_spef is None:
        raise RuntimeError("SPEF1 not parsed with C++ backend")
    if not hasattr(spef2, '_cpp_spef') or spef2._cpp_spef is None:
        raise RuntimeError("SPEF2 not parsed with C++ backend")
    
    try:
        cc_results = spef_core.compare_coupling_caps(spef1._cpp_spef, spef2._cpp_spef)
        return [CouplingCapComparison(cc.net1, cc.net2, cc.c1, cc.c2) for cc in cc_results]
    except Exception as e:
        print(f"[warn] Coupling cap comparison failed: {e}")
        return []


def compare_spef_cpp_objs(spef1: "SpefFile", spef2: "SpefFile", num_threads: int = 0):
    """Compare two already-parsed SPEF objects using C++ backend."""
    if not HAS_CPP:
        raise RuntimeError("C++ extension not available")
    result = spef_core.compare_spef_full(spef1._cpp_spef, spef2._cpp_spef, num_threads)
    caps = [CapComparison(c.net_name, c.c1, c.c2) for c in result.cap_rows]
    ress = [ResComparison(r.net_name, r.driver, r.sink, r.r1, r.r2) for r in result.res_rows]
    top_10_cap = [CapComparison(c.net_name, c.c1, c.c2) for c in result.top_10_cap]
    top_10_res = [ResComparison(r.net_name, r.driver, r.sink, r.r1, r.r2) for r in result.top_10_res]
    return caps, ress, top_10_cap, top_10_res


def backmark_spef_cpp(spef_path: str, cap_data_path: Optional[str], res_data_path: Optional[str], output_path: str) -> None:
    """Apply new RC values to SPEF using C++ backend."""
    if not HAS_CPP:
        raise RuntimeError("C++ extension not available")
    spef_core.backmark_spef(spef_path, cap_data_path or "", res_data_path or "", output_path)


def shuffle_spef_cpp(spef_path: str, output_path: str, seed: Optional[int] = None) -> None:
    """Shuffle net_id mapping using C++ backend."""
    if not HAS_CPP:
        raise RuntimeError("C++ extension not available")
    actual_seed = seed if seed is not None else -1
    spef_core.shuffle_spef(spef_path, output_path, actual_seed)


def parse_spefs_parallel(path1: str, path2: str) -> Tuple[SpefFile, SpefFile]:
    """Parse two SPEF files concurrently using C++ multithreaded backend.
    
    Optimized for 1M+ nets: keeps data in C++ format, no Python conversion.
    """
    if not HAS_CPP:
        raise RuntimeError("C++ extension not available")
    print(f"Parsing {path1} and {path2} in parallel (C++ threads)...")
    try:
        cpp_spefs = spef_core.parse_spef_parallel([path1, path2], 2)
        
        # Wrap in SpefFile - keep data in C++ format for performance
        s1 = SpefFile(path1)
        s1._cpp_spef = cpp_spefs[0]
        s1.name_map = dict(cpp_spefs[0].name_map)
        s1.t_unit = cpp_spefs[0].t_unit
        s1.c_unit = cpp_spefs[0].c_unit
        s1.r_unit = cpp_spefs[0].r_unit
        s1.l_unit = cpp_spefs[0].l_unit
        # DON'T build s1.nets Python dict - keep in C++!
        # Lazy load nets only when needed for tree view display
        s1._net_count = len(cpp_spefs[0].nets)  # Store count from C++

        s2 = SpefFile(path2)
        s2._cpp_spef = cpp_spefs[1]
        s2.name_map = dict(cpp_spefs[1].name_map)
        s2.t_unit = cpp_spefs[1].t_unit
        s2.c_unit = cpp_spefs[1].c_unit
        s2.r_unit = cpp_spefs[1].r_unit
        s2.l_unit = cpp_spefs[1].l_unit
        s2._net_count = len(cpp_spefs[1].nets)  # Store count from C++
        
        return s1, s2
    except Exception as exc:
        print(f"[warn] C++ parallel parse failed ({exc}), falling back to sequential")
        s1 = SpefFile(path1)
        s1.parse()
        s2 = SpefFile(path2)
        s2.parse()
        return s1, s2


class SpefFile:
    """Simple SPEF wrapper that uses C++ backend."""
    def __init__(self, path: str):
        self.path = path
        self.name_map: Dict[str, str] = {}
        self.nets: Dict[str, NetRC] = {}  # Lazy-loaded on demand
        self.t_unit = 'NS'
        self.c_unit = 'PF'
        self.r_unit = 'OHM'
        self.l_unit = 'HENRY'
        self._cpp_spef = None
        self._net_count = 0  # Cached net count
    
    def parse(self) -> None:
        """Parse SPEF file using C++ backend."""
        if not HAS_CPP:
            raise RuntimeError("C++ extension required for parsing")
        print(f"[{self.path}] parsing with C++ extension...")
        self._cpp_spef = spef_core.parse_spef(self.path)
        self.name_map = dict(self._cpp_spef.name_map)
        self.t_unit = self._cpp_spef.t_unit
        self.c_unit = self._cpp_spef.c_unit
        self.r_unit = self._cpp_spef.r_unit
        self.l_unit = self._cpp_spef.l_unit
        self._net_count = len(self._cpp_spef.nets)
        print(f"[{self.path}] parsed {self._net_count} nets... (C++)")
    
    def __len__(self) -> int:
        """Return net count - from C++ if available, else from Python dict."""
        if self._cpp_spef is not None:
            return len(self._cpp_spef.nets)
        return len(self.nets)
    
    def get_net_count(self) -> int:
        """Get net count without building Python dict - optimized for 1M+ nets."""
        if self._net_count > 0:
            return self._net_count
        if self._cpp_spef is not None:
            return len(self._cpp_spef.nets)
        return len(self.nets)


def summarize_and_print(caps: List[CapComparison], ress: List[ResComparison], 
                        spef1_path: str, spef2_path: str, r_agg: str) -> None:
    """Print human-readable summary."""
    print("=== SPEF RC Correlation Summary ===")
    print(f"Tool1 SPEF: {spef1_path}")
    print(f"Tool2 SPEF: {spef2_path}")
    print(f"Common nets: {len({c.net for c in caps})}")
    print(f"Cap rows: {len(caps)}")
    print(f"Res rows: {len(ress)}")
    
    xs_c = [c.c1 for c in caps]
    ys_c = [c.c2 for c in caps]
    corr_c = pearson_corr(xs_c, ys_c)
    if corr_c is not None:
        print(f"Total C correlation (Pearson): {corr_c:.6f} over {len(caps)} nets")
    else:
        print("Total C correlation: N/A")
    
    xs_r = [r.r1 for r in ress]
    ys_r = [r.r2 for r in ress]
    corr_r = pearson_corr(xs_r, ys_r)
    if corr_r is not None:
        print(f"Driver->sink R correlation (Pearson, agg={r_agg}): {corr_r:.6f} over {len(ress)} nets")
    else:
        print("Driver->sink R correlation: N/A")


# ===================== Main CLI =====================

def main() -> None:
    parser = argparse.ArgumentParser(description="SPEF RC correlation between two extraction tools")
    parser.add_argument("spef1", nargs="?", help="First SPEF file (tool 1)")
    parser.add_argument("spef2", nargs="?", help="Second SPEF file (tool 2)")
    parser.add_argument("--csv-prefix", help="Write CSVs with this prefix")
    parser.add_argument("--r-agg", choices=["max", "avg", "total"], default="max",
                        help="Aggregation mode for R")
    parser.add_argument("--gui", action="store_true", help="Launch GUI")
    parser.add_argument("--gui-auto-run", action="store_true", help="Auto-run in GUI")
    parser.add_argument("--net-cap-data", metavar="FILE", help="Pre-computed cap data file")
    parser.add_argument("--net-res-data", metavar="FILE", help="Pre-computed res data file")
    parser.add_argument("--net-ccap-data", metavar="FILE", help="Pre-computed coupling cap data file")
    parser.add_argument("--backmark", action="store_true", help="Backmark mode")
    parser.add_argument("--shuffle", action="store_true", help="Shuffle mode")
    parser.add_argument("--seed", type=int, default=None, metavar="INT", help="Random seed")
    parser.add_argument("--output", "-o", metavar="FILE", help="Output path")
    parser.add_argument("--threads", "-t", type=int, default=0, help="Number of threads")

    args = parser.parse_args()

    # Backmark mode
    if args.backmark:
        if not args.spef1:
            parser.error("--backmark requires spef1")
        if not args.net_cap_data and not args.net_res_data:
            parser.error("--backmark requires at least one of --net-cap-data or --net-res-data")
        out = args.output or os.path.splitext(args.spef1)[0] + "_backmarked.spef"
        print(f"[backmark] Processing {args.spef1}...")
        backmark_spef_cpp(args.spef1, args.net_cap_data, args.net_res_data, out)
        print(f"[backmark] Done: {out}")
        return

    # Shuffle mode
    if args.shuffle:
        if not args.spef1:
            parser.error("--shuffle requires spef1")
        out = args.output or os.path.splitext(args.spef1)[0] + "_shuffled.spef"
        print(f"[shuffle] Processing {args.spef1}...")
        shuffle_spef_cpp(args.spef1, out, args.seed)
        print(f"[shuffle] Done: {out}")
        return

    # Normal SPEF mode
    if args.spef1 and args.spef2:
        if args.gui_auto_run:
            # Use numpy export for maximum performance with large datasets
            s1, s2 = parse_spefs_parallel(args.spef1, args.spef2)
            plot_data = spef_core.export_plot_data(s1._cpp_spef, s2._cpp_spef, args.threads)
            launch_gui(
                preload_paths=None,
                auto_run=True,
                preload_caps=None,
                preload_ress=None,
                preload_spef_objs=[
                    (os.path.splitext(os.path.basename(args.spef1))[0], s1),
                    (os.path.splitext(os.path.basename(args.spef2))[0], s2)
                ],
                preload_cpp_result=plot_data  # Pass PlotData with numpy arrays
            )
            return
        elif args.gui:
            # --gui: 也并行解析 spef，传递给 GUI
            s1, s2 = parse_spefs_parallel(args.spef1, args.spef2)
            launch_gui(
                preload_paths=None,
                auto_run=False,
                preload_spef_objs=[
                    (os.path.splitext(os.path.basename(args.spef1))[0], s1),
                    (os.path.splitext(os.path.basename(args.spef2))[0], s2)
                ]
            )
        else:
            cap_out = args.net_cap_data or "net_cap.data"
            res_out = args.net_res_data or "net_res.data"
            ccap_out = args.net_ccap_data or "net_ccap.data"
            caps, ress, coupling_caps, top_10_cap, top_10_res = compare_spef_with_coupling_cpp(args.spef1, args.spef2, args.threads)
            with open(cap_out, 'w') as fc, open(res_out, 'w') as fr, open(ccap_out, 'w') as fcc:
                for cap in caps:
                    print(f"{cap.net} {cap.c1} {cap.c2}", file=fc)
                for res in ress:
                    print(f"{res.net} {res.driver} {res.load} {res.r1} {res.r2}", file=fr)
                for ccap in coupling_caps:
                    print(f"{ccap.net1} {ccap.net2} {ccap.c1} {ccap.c2}", file=fcc)
            print(f"Cap data written to: {cap_out}")
            print(f"Res data written to: {res_out}")
            print(f"Coupling cap data written to: {ccap_out}")
            summarize_and_print(caps, ress, args.spef1, args.spef2, args.r_agg)

            print("\nTop 10 Cap Deviations:")
            for cap in top_10_cap:
                print(f"  {cap.net}: {cap.c1:.6f} vs {cap.c2:.6f} (delta={abs(cap.c2-cap.c1):.6f})")

            print("\nTop 10 Res Deviations:")
            for res in top_10_res:
                print(f"  {res.net}/{res.load}: {res.r1:.6f} vs {res.r2:.6f} (delta={abs(res.r2-res.r1):.6f})")

            if args.csv_prefix:
                write_caps_csv(f"{args.csv_prefix}_caps.csv", caps)
                write_res_csv(f"{args.csv_prefix}_res_{args.r_agg}.csv", ress, args.r_agg)
                print(f"\nCSV written: {args.csv_prefix}_caps.csv, {args.csv_prefix}_res_{args.r_agg}.csv")
        return

    # Data-file mode
    if args.net_cap_data or args.net_res_data or args.net_ccap_data:
        if args.gui or args.gui_auto_run:
            launch_gui(
                collect_spef_paths([p for p in [args.spef1, args.spef2] if p]),
                auto_run=args.gui_auto_run,
                preload_cap_data=args.net_cap_data,
                preload_res_data=args.net_res_data,
                preload_ccap_data=args.net_ccap_data,
            )
            return

        caps = parse_net_cap_data(args.net_cap_data) if args.net_cap_data else []
        ress = parse_net_res_data(args.net_res_data) if args.net_res_data else []
        coupling_caps = parse_net_ccap_data(args.net_ccap_data) if args.net_ccap_data else []

        print("=== Summary (from data files) ===")
        if caps:
            print(f"Cap entries: {len(caps)}")
            corr_c = pearson_corr([c.c1 for c in caps], [c.c2 for c in caps])
            print(f"Cap correlation: {corr_c:.6f}" if corr_c else "Cap correlation: N/A")
        if ress:
            print(f"Res entries: {len(ress)}")
            corr_r = pearson_corr([r.r1 for r in ress], [r.r2 for r in ress])
            print(f"Res correlation: {corr_r:.6f}" if corr_r else "Res correlation: N/A")
        if coupling_caps:
            print(f"Coupling cap entries: {len(coupling_caps)}")
            corr_cc = pearson_corr([c.c1 for c in coupling_caps], [c.c2 for c in coupling_caps])
            print(f"Coupling cap correlation: {corr_cc:.6f}" if corr_cc else "Coupling cap correlation: N/A")

        if args.csv_prefix:
            if caps: write_caps_csv(f"{args.csv_prefix}_caps.csv", caps)
            if ress: write_res_csv(f"{args.csv_prefix}_res.csv", ress, args.r_agg)
        return

    # GUI only
    if args.gui or args.gui_auto_run:
        launch_gui(auto_run=args.gui_auto_run)
        return

    parser.print_help()


# ===================== GUI =====================

def collect_spef_paths(inputs: Iterable[str]) -> List[str]:
    """Expand file and directory inputs into SPEF path list."""
    spef_paths: List[str] = []
    for raw_path in inputs:
        if not raw_path:
            continue
        path = os.path.abspath(raw_path)
        if os.path.isdir(path):
            try:
                for name in sorted(os.listdir(path)):
                    child = os.path.join(path, name)
                    if os.path.isfile(child) and name.lower().endswith(".spef"):
                        spef_paths.append(child)
            except OSError:
                continue
        elif os.path.isfile(path) and path.lower().endswith(".spef"):
            spef_paths.append(path)
    return spef_paths



def launch_gui(
    preload_paths: Optional[Iterable[str]] = None,
    auto_run: bool = False,
    preload_cap_data: Optional[str] = None,
    preload_res_data: Optional[str] = None,
    preload_ccap_data: Optional[str] = None,
    preload_caps: Optional[List[CapComparison]] = None,
    preload_ress: Optional[List[ResComparison]] = None,
    preload_ccaps: Optional[List[CouplingCapComparison]] = None,
    preload_spef_objs: Optional[list] = None,
    preload_cpp_result: Optional[object] = None
) -> None:
    """Launch interactive GUI."""

    if tk is None or FigureCanvasTkAgg is None or Figure is None:
        print("GUI requires tkinter and matplotlib.")
        return

    root = tk.Tk()
    root.protocol("WM_DELETE_WINDOW", root.destroy)
    RcCorrApp(root, preload_paths, auto_run, preload_cap_data, preload_res_data, preload_ccap_data, preload_caps, preload_ress, preload_ccaps, preload_spef_objs, preload_cpp_result)
    root.mainloop()


class RcCorrApp:
    def __init__(self, root, preload_paths=None, auto_run=False,
                 preload_cap_data=None, preload_res_data=None, preload_ccap_data=None,
                 preload_caps=None, preload_ress=None, preload_ccaps=None, preload_spef_objs=None,
                 preload_cpp_result=None):
        self.root = root
        self.root.title("SPEF RC Correlation")
        self.spefs: Dict[str, SpefFile] = {}
        self._data_caps: List[CapComparison] = []
        self._data_ress: List[ResComparison] = []
        self._data_coupling_caps: List[CouplingCapComparison] = []  # New: coupling cap comparisons
        self._cpp_result = preload_cpp_result  # Keep C++ result directly
        self._fanout_cache: Optional[Dict[str, int]] = None  # net_name -> fanout count
        self._fanout_cache_ref: Optional[str] = None  # ref SPEF name for cache
        self.ref_var = tk.StringVar()
        self.fit_var = tk.StringVar()
        self.r_agg_var = tk.StringVar(value="max")
        self.view_mode_var = tk.StringVar(value="total_cap")  # New: view mode (total_cap or coupling_cap)
        self.stat_metric_var = tk.StringVar(value="stddev")  # New: histogram metric (stddev or rmse)
        self._auto_run_requested = auto_run
        self._build_ui()

        if preload_spef_objs:
            for name, spef in preload_spef_objs:
                self.spefs[name] = spef
                # Use get_net_count() to avoid building Python dict for 1M+ nets
                net_count = spef.get_net_count() if hasattr(spef, 'get_net_count') else len(spef.nets)
                self.tree.insert("", "end", iid=name, values=(name, net_count, spef.path))
            self._refresh_choices()
        elif preload_paths:
            self._preload_spefs(preload_paths)
        if preload_cap_data:
            self._load_cap_data(preload_cap_data)
        if preload_res_data:
            self._load_res_data(preload_res_data)
        if preload_ccap_data:
            self._load_ccap_data(preload_ccap_data)
        if preload_caps:
            self._data_caps = list(preload_caps)
            self.cap_data_label.config(text=f"(preloaded) ({len(preload_caps)} nets)", foreground="black")
        if preload_ress:
            self._data_ress = list(preload_ress)
            self.res_data_label.config(text=f"(preloaded) ({len(preload_ress)} pairs)", foreground="black")
        if preload_ccaps:
            self._data_coupling_caps = list(preload_ccaps)
            self.ccap_data_label.config(text=f"(preloaded) ({len(preload_ccaps)} pairs)", foreground="black")
            if self._cpp_result is None or (self._cpp_result.cap_count == 0 and self._cpp_result.res_count == 0):
                self.view_mode_var.set("coupling_cap")

        # 自动分析逻辑：如果 auto_run=True 且 spef 文件数>=2，则自动运行分析
        if self._auto_run_requested:
            if len(self.spefs) >= 2:
                self.root.after(0, self._auto_run)
            # 也可根据需求支持 data file 自动分析
            elif self._data_caps or self._data_ress or self._data_coupling_caps:
                self.root.after(0, self._run_from_data)
            self._auto_run_requested = False

    def _init_annot(self, ax):
        # For Tkinter backend, use a separate Toplevel window as tooltip instead of matplotlib annotation
        self._tooltip_window = None
        self._tooltip_label = None
        # Return a dummy annot for compatibility
        return None

    def _cache_plot_arrays(self, points_cap, points_res):
        try:
            import numpy as np
        except ImportError:
            self._xs_c = self._ys_c = self._xs_r = self._ys_r = None
            self._kd_c = self._kd_r = None
            return
        # Check if scipy is available for kd-tree optimization
        try:
            from scipy.spatial import cKDTree
            HAS_CKD = True
        except ImportError:
            HAS_CKD = False
        if points_cap:
            self._xs_c = np.array([p["c_ref"] for p in points_cap])
            self._ys_c = np.array([p["c_fit"] for p in points_cap])
            self._cap_points = points_cap
            # Build kd-tree for fast nearest neighbor search with large datasets
            if HAS_CKD and len(points_cap) > 100000:
                self._kd_c = cKDTree(np.column_stack((self._xs_c, self._ys_c)))
            else:
                self._kd_c = None
        else:
            self._xs_c = self._ys_c = None
            self._cap_points = []
            self._kd_c = None
        if points_res:
            self._xs_r = np.array([p["r_ref"] for p in points_res])
            self._ys_r = np.array([p["r_fit"] for p in points_res])
            self._res_points = points_res
            # Build kd-tree for fast nearest neighbor search with large datasets
            if HAS_CKD and len(points_res) > 100000:
                self._kd_r = cKDTree(np.column_stack((self._xs_r, self._ys_r)))
            else:
                self._kd_r = None
        else:
            self._xs_r = self._ys_r = None
            self._res_points = []
            self._kd_r = None

    def _fmt_hover(self, v):
        av = abs(v)
        if av == 0:
            return "0"
        if av < 1e-3:
            return f"{v:.4e}"
        if av < 1:
            return f"{v:.6f}"
        if av < 1000:
            return f"{v:.4f}"
        return f"{v:.4g}"

    def _on_motion(self, event):
        if not hasattr(self, "_cap_points") or not hasattr(self, "_res_points"):
            return
        ax = event.inaxes
        if ax not in (self.ax_c, self.ax_r):
            # Hide tooltip when mouse leaves axes
            if hasattr(self, '_tooltip_window') and self._tooltip_window is not None:
                self._hide_tooltip()
            return
        if event.xdata is None or event.ydata is None:
            return
        # In coupling-cap mode, only ax_c is a scatter plot; ax_r is histogram.
        if self.view_mode_var.get() == "coupling_cap" and ax is self.ax_r:
            if hasattr(self, '_tooltip_window') and self._tooltip_window is not None:
                self._hide_tooltip()
            return
        import numpy as np
        if ax is self.ax_c and self._xs_c is not None and len(self._xs_c):
            xs_arr = self._xs_c
            ys_arr = self._ys_c
            points = self._cap_points
            kd_tree = getattr(self, "_kd_c", None)
        elif ax is self.ax_r and self._xs_r is not None and len(self._xs_r):
            xs_arr = self._xs_r
            ys_arr = self._ys_r
            points = self._res_points
            kd_tree = getattr(self, "_kd_r", None)
        else:
            return
        transform = ax.transData.transform
        mouse_px = transform([[event.xdata, event.ydata]])[0]
        # Use kd-tree for fast lookup if available (for large datasets)
        if kd_tree is not None:
            dist, best_i = kd_tree.query([event.xdata, event.ydata])
            # dist is Euclidean distance in data units
            # Use a threshold based on data coordinate scale
            # Convert ~20 pixel radius to data units
            inv_transform = transform.inverted()
            # Get data coordinates at two points 20 pixels apart in x
            p1 = inv_transform([[event.xdata, event.ydata]])[0]
            p2 = inv_transform([[event.xdata + 20, event.ydata]])[0]
            data_threshold = abs(p2[0] - p1[0])
            if dist < data_threshold:
                self._show_annotation(event, ax, xs_arr, ys_arr, points, best_i, kd_tree)
                return
        else:
            data_pts = np.column_stack((xs_arr, ys_arr))
            px_pts = transform(data_pts)
            dx = px_pts[:, 0] - mouse_px[0]
            dy = px_pts[:, 1] - mouse_px[1]
            d2 = dx * dx + dy * dy
            best_i = int(np.argmin(d2))
            # d2 is in pixel^2, threshold is 20 pixel radius -> 400 pixel^2
            if d2[best_i] < 400:
                self._show_annotation(event, ax, xs_arr, ys_arr, points, best_i, None)
                return
        # Hide tooltip if no point is close enough
        if hasattr(self, '_tooltip_window') and self._tooltip_window is not None:
            self._hide_tooltip()

    def _show_annotation(self, event, ax, xs_arr, ys_arr, points, best_i, kd_tree):
        """Show tooltip using Tkinter."""
        p = points[best_i]
        fmt = self._fmt_hover
        if ax is self.ax_c:
            ref_v = p["c_ref"]
            fit_v = p["c_fit"]
            delta = fit_v - ref_v
            ratio = fit_v / ref_v if ref_v != 0 else float('inf')
            if "net1" in p and "net2" in p:
                text = (
                    f"net1: {p['net1']}\n"
                    f"net2: {p['net2']}\n"
                    f"Cc_ref : {fmt(ref_v)}\n"
                    f"Cc_fit : {fmt(fit_v)}\n"
                    f"delta  : {fmt(delta)}\n"
                    f"ratio  : {ratio:.4f}"
                )
                if "pct_ref" in p:
                    text += f"\nCc% ref: {p['pct_ref']:.4f}%"
                if "pct_fit" in p:
                    text += f"\nCc% fit: {p['pct_fit']:.4f}%"
            else:
                text = (
                    f"net: {p['net']}\n"
                    f"C_ref : {fmt(ref_v)}\n"
                    f"C_fit : {fmt(fit_v)}\n"
                    f"delta : {fmt(delta)}\n"
                    f"ratio : {ratio:.4f}"
                )
            if "fanout" in p:
                text += f"\nfanout: {int(p['fanout'])}"
        else:
            ref_v = p["r_ref"]
            fit_v = p["r_fit"]
            delta = fit_v - ref_v
            ratio = fit_v / ref_v if ref_v != 0 else float('inf')
            text = (
                f"net: {p['net']}\n"
                f"driver: {p.get('driver','')}\n"
                f"load: {p.get('load','')}\n"
                f"R_ref : {fmt(ref_v)}\n"
                f"R_fit : {fmt(fit_v)}\n"
                f"delta : {fmt(delta)}\n"
                f"ratio : {ratio:.4f}"
            )
            if "fanout" in p:
                text += f"\nfanout: {int(p['fanout'])}"
        # Show tooltip window
        self._show_tooltip(event, text)

    def _show_tooltip(self, event, text):
        """Show a Tkinter tooltip window at mouse position."""
        # Hide existing tooltip
        self._hide_tooltip()
        # Create tooltip window
        self._tooltip_window = tk.Toplevel(self.root)
        self._tooltip_window.wm_overrideredirect(True)  # No window border
        self._tooltip_window.wm_geometry("+99999+99999")  # Start off-screen
        # Add label
        self._tooltip_label = tk.Label(
            self._tooltip_window,
            text=text,
            background="#ffffcc",
            foreground="black",
            relief=tk.SOLID,
            borderwidth=1,
            font=("monospace", 9),
            justify=tk.LEFT,
            padx=5,
            pady=5,
        )
        self._tooltip_label.pack()
        # Get mouse position
        x = self.root.winfo_pointerx() + 10
        y = self.root.winfo_pointery() + 10
        # Ensure it stays on screen
        w = self._tooltip_label.winfo_reqwidth()
        h = self._tooltip_label.winfo_reqheight()
        screen_w = self.root.winfo_screenwidth()
        screen_h = self.root.winfo_screenheight()
        if x + w > screen_w:
            x = screen_w - w - 10
        if y + h > screen_h:
            y = screen_h - h - 10
        self._tooltip_window.wm_geometry(f"+{x}+{y}")
        self._tooltip_window.lift()

    def _hide_tooltip(self):
        """Hide the tooltip window."""
        if hasattr(self, '_tooltip_window') and self._tooltip_window is not None:
            try:
                self._tooltip_window.destroy()
            except:
                pass
            self._tooltip_window = None
            self._tooltip_label = None

    def _build_ui(self) -> None:
        # SPEF files frame
        frm = ttk.LabelFrame(self.root, text="SPEF Files")
        frm.pack(fill="x", padx=5, pady=5)

        self.tree = ttk.Treeview(frm, columns=("name", "nets", "path"), show="headings", height=4)
        self.tree.heading("name", text="Name")
        self.tree.heading("nets", text="#Nets")
        self.tree.heading("path", text="Path")
        self.tree.column("name", width=80, anchor="center")
        self.tree.column("nets", width=70, anchor="center")
        self.tree.column("path", width=400, anchor="w")
        self.tree.pack(side="left", fill="x", expand=True, padx=(5, 0), pady=5)

        btn_frm = ttk.Frame(frm)
        btn_frm.pack(side="right", fill="y", padx=5, pady=5)


        ttk.Button(btn_frm, text="Add SPEF", command=self._add_spef).pack(fill="x", pady=2)
        ttk.Button(btn_frm, text="Remove", command=self._remove_selected).pack(fill="x", pady=2)

        # Data files frame
        dfrm = ttk.LabelFrame(self.root, text="Data Files")
        dfrm.pack(fill="x", padx=5, pady=5)

        cap_row = ttk.Frame(dfrm)
        cap_row.pack(fill="x", padx=5, pady=2)
        ttk.Label(cap_row, text="Cap Data File:").pack(side="left")
        self.cap_data_label = ttk.Label(cap_row, text="(none)", foreground="gray", anchor="w")
        self.cap_data_label.pack(side="left", padx=5, fill="x", expand=True)
        ttk.Button(cap_row, text="Load...", command=self._add_cap_data).pack(side="right")

        res_row = ttk.Frame(dfrm)
        res_row.pack(fill="x", padx=5, pady=2)
        ttk.Label(res_row, text="Res Data File:").pack(side="left")
        self.res_data_label = ttk.Label(res_row, text="(none)", foreground="gray", anchor="w")
        self.res_data_label.pack(side="left", padx=5, fill="x", expand=True)
        ttk.Button(res_row, text="Load...", command=self._add_res_data).pack(side="right")

        ccap_row = ttk.Frame(dfrm)
        ccap_row.pack(fill="x", padx=5, pady=2)
        ttk.Label(ccap_row, text="Ccap Data File:").pack(side="left")
        self.ccap_data_label = ttk.Label(ccap_row, text="(none)", foreground="gray", anchor="w")
        self.ccap_data_label.pack(side="left", padx=5, fill="x", expand=True)
        ttk.Button(ccap_row, text="Load...", command=self._add_ccap_data).pack(side="right")

        ttk.Frame(dfrm, height=5).pack()
        ttk.Button(dfrm, text="Run from Data Files", command=self._run_from_data).pack(fill="x", padx=5, pady=(0, 4))

        # Settings & Filters frame
        srm = ttk.LabelFrame(self.root, text="Settings & Filters")
        srm.pack(fill="x", padx=5, pady=5)

        # Row 0a: view mode selection (NEW)
        ttk.Label(srm, text="View Mode:").grid(row=0, column=0, sticky="w", padx=5, pady=2)
        self.view_mode_combo = ttk.Combobox(srm, textvariable=self.view_mode_var, 
                                           values=["total_cap", "coupling_cap"], 
                                           state="readonly", width=15)
        self.view_mode_combo.grid(row=0, column=1, sticky="w", padx=5, pady=2)
        self.view_mode_combo.bind("<<ComboboxSelected>>", lambda e: self._on_view_mode_change())
        
        # Row 0b: stat metric selection (NEW)
        ttk.Label(srm, text="Histogram Metric:").grid(row=0, column=2, sticky="w", padx=5, pady=2)
        self.stat_metric_combo = ttk.Combobox(srm, textvariable=self.stat_metric_var, 
                                             values=["stddev", "rmse"], 
                                             state="readonly", width=12)
        self.stat_metric_combo.grid(row=0, column=3, sticky="w", padx=5, pady=2)

        # Row 0: reference & fit selection
        ttk.Label(srm, text="Reference:").grid(row=1, column=0, sticky="w", padx=5, pady=2)
        self.ref_combo = ttk.Combobox(srm, textvariable=self.ref_var, state="readonly", width=15)
        self.ref_combo.grid(row=1, column=1, sticky="w", padx=5, pady=2)

        ttk.Label(srm, text="Fit:").grid(row=1, column=2, sticky="w", padx=5, pady=2)
        self.fit_combo = ttk.Combobox(srm, textvariable=self.fit_var, state="readonly", width=15)
        self.fit_combo.grid(row=1, column=3, sticky="w", padx=5, pady=2)

        # Row 2: fanout range
        self.min_fanout_var = tk.StringVar(value="")
        self.max_fanout_var = tk.StringVar(value="")
        ttk.Label(srm, text="Fanout range:").grid(row=2, column=0, sticky="w", padx=5, pady=2)
        ttk.Entry(srm, textvariable=self.min_fanout_var, width=8).grid(row=2, column=1, sticky="w", padx=5, pady=2)
        ttk.Label(srm, text="to").grid(row=2, column=2, sticky="w", padx=2, pady=2)
        ttk.Entry(srm, textvariable=self.max_fanout_var, width=8).grid(row=2, column=3, sticky="w", padx=5, pady=2)

        # Row 3: cap range (reference) - for total cap
        self.min_c_var = tk.StringVar(value="")
        self.max_c_var = tk.StringVar(value="")
        ttk.Label(srm, text="Cap range (ref C):").grid(row=3, column=0, sticky="w", padx=5, pady=2)
        ttk.Entry(srm, textvariable=self.min_c_var, width=10).grid(row=3, column=1, sticky="w", padx=5, pady=2)
        ttk.Label(srm, text="to").grid(row=3, column=2, sticky="w", padx=2, pady=2)
        ttk.Entry(srm, textvariable=self.max_c_var, width=10).grid(row=3, column=3, sticky="w", padx=5, pady=2)

        # Row 4: coupling cap range (NEW)
        self.min_coupling_cap_var = tk.StringVar(value="")
        self.max_coupling_cap_var = tk.StringVar(value="")
        ttk.Label(srm, text="Coupling cap range:").grid(row=4, column=0, sticky="w", padx=5, pady=2)
        ttk.Entry(srm, textvariable=self.min_coupling_cap_var, width=10).grid(row=4, column=1, sticky="w", padx=5, pady=2)
        ttk.Label(srm, text="to").grid(row=4, column=2, sticky="w", padx=2, pady=2)
        ttk.Entry(srm, textvariable=self.max_coupling_cap_var, width=10).grid(row=4, column=3, sticky="w", padx=5, pady=2)

        # Row 5: coupling cap percentage range (NEW)
        self.min_coupling_pct_var = tk.StringVar(value="")
        self.max_coupling_pct_var = tk.StringVar(value="")
        ttk.Label(srm, text="Coupling % (ref):").grid(row=5, column=0, sticky="w", padx=5, pady=2)
        ttk.Entry(srm, textvariable=self.min_coupling_pct_var, width=10).grid(row=5, column=1, sticky="w", padx=5, pady=2)
        ttk.Label(srm, text="to").grid(row=5, column=2, sticky="w", padx=2, pady=2)
        ttk.Entry(srm, textvariable=self.max_coupling_pct_var, width=10).grid(row=5, column=3, sticky="w", padx=5, pady=2)

        # Row 6: R range (reference, aggregated)
        self.min_r_var = tk.StringVar(value="")
        self.max_r_var = tk.StringVar(value="")
        ttk.Label(srm, text="R range (ref, agg):").grid(row=6, column=0, sticky="w", padx=5, pady=2)
        ttk.Entry(srm, textvariable=self.min_r_var, width=10).grid(row=6, column=1, sticky="w", padx=5, pady=2)
        ttk.Label(srm, text="to").grid(row=6, column=2, sticky="w", padx=2, pady=2)
        ttk.Entry(srm, textvariable=self.max_r_var, width=10).grid(row=6, column=3, sticky="w", padx=5, pady=2)

        # Row 7: R aggregation, run button, correlation label
        ttk.Label(srm, text="R aggregation:").grid(row=7, column=0, sticky="w", padx=5, pady=2)
        ttk.Combobox(srm, textvariable=self.r_agg_var, values=["max", "avg", "total"], state="readonly", width=8).grid(row=7, column=1, sticky="w", padx=5, pady=2)

        ttk.Button(srm, text="Run Analysis", command=self._run_analysis).grid(row=7, column=2, sticky="w", padx=5, pady=2)
        ttk.Button(srm, text="Diff Histogram", command=self._show_histogram).grid(row=7, column=4, sticky="w", padx=5, pady=2)

        self.corr_label = ttk.Label(srm, text="")
        self.corr_label.grid(row=7, column=3, sticky="w", padx=5, pady=2)

        # Plot frame
        pframe = ttk.LabelFrame(self.root, text="Correlation Plot")
        pframe.pack(fill="both", expand=True, padx=5, pady=5)

        self.fig = Figure(figsize=(7, 5))
        self.ax_c = self.fig.add_subplot(2, 1, 1)
        self.ax_r = self.fig.add_subplot(2, 1, 2)
        self.canvas = FigureCanvasTkAgg(self.fig, master=pframe)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self.fig.tight_layout()
        self.canvas.mpl_connect("motion_notify_event", self._on_motion)

    def _parse_filters(self):
        def to_int(s):
            s = s.strip()
            if not s:
                return None
            return int(s)
        def to_float(s):
            s = s.strip()
            if not s:
                return None
            return float(s)
        try:
            return {
                "min_fanout": to_int(self.min_fanout_var.get()),
                "max_fanout": to_int(self.max_fanout_var.get()),
                "min_c": to_float(self.min_c_var.get()),
                "max_c": to_float(self.max_c_var.get()),
                "min_coupling_cap": to_float(self.min_coupling_cap_var.get()),
                "max_coupling_cap": to_float(self.max_coupling_cap_var.get()),
                "min_coupling_pct": to_float(self.min_coupling_pct_var.get()),
                "max_coupling_pct": to_float(self.max_coupling_pct_var.get()),
                "min_r": to_float(self.min_r_var.get()),
                "max_r": to_float(self.max_r_var.get()),
            }
        except Exception as exc:
            import tkinter.messagebox as messagebox
            messagebox.showerror("Invalid filter", f"Filter values must be numeric.\n{exc}")
            return None

    def _on_view_mode_change(self) -> None:
        """Handle view mode change between total_cap and coupling_cap."""
        self._update_plot()

    def _get_fanout_cache(self) -> Dict[str, int]:
        """Return a net_name -> fanout (sink count) mapping from the reference SPEF.
        
        Returns an empty dict when no SPEF is loaded (CSV-only mode); callers
        should treat a missing key as "unknown fanout" and skip the fanout filter
        for that point.
        """
        ref_name = self.ref_var.get() if hasattr(self, 'ref_var') else None
        if ref_name == self._fanout_cache_ref and self._fanout_cache is not None:
            return self._fanout_cache
        cache: Dict[str, int] = {}
        if ref_name and ref_name in self.spefs:
            spef = self.spefs[ref_name]
            if hasattr(spef, '_cpp_spef') and spef._cpp_spef is not None:
                try:
                    for net_name, net_data in spef._cpp_spef.nets.items():
                        cache[net_name] = len(net_data.sinks)
                except Exception as exc:
                    print(f"[warn] Failed to build fanout cache: {exc}")
        self._fanout_cache = cache
        self._fanout_cache_ref = ref_name
        return cache

    def _passes_filters(self, p, flt):
        mf = flt["min_fanout"]
        xf = flt["max_fanout"]
        fanout = p.get("fanout")
        # None means fanout is unknown (CSV-only mode) – skip fanout filter
        if fanout is not None:
            if mf is not None and fanout < mf:
                return False
            if xf is not None and fanout > xf:
                return False
        if "c_ref" in p:
            mc = flt["min_c"]
            xc = flt["max_c"]
            if mc is not None and p["c_ref"] < mc:
                return False
            if xc is not None and p["c_ref"] > xc:
                return False
        if "cc_ref" in p:
            mc = flt["min_coupling_cap"]
            xc = flt["max_coupling_cap"]
            if mc is not None and p["cc_ref"] < mc:
                return False
            if xc is not None and p["cc_ref"] > xc:
                return False
        if "cc_pct_ref" in p:
            mp = flt["min_coupling_pct"]
            xp = flt["max_coupling_pct"]
            if mp is not None and p["cc_pct_ref"] < mp:
                return False
            if xp is not None and p["cc_pct_ref"] > xp:
                return False
        if "r_ref" in p:
            mr = flt["min_r"]
            xr = flt["max_r"]
            if mr is not None and p["r_ref"] < mr:
                return False
            if xr is not None and p["r_ref"] > xr:
                return False
        return True

    def _add_spef(self) -> None:
        path = filedialog.askopenfilename(title="Select SPEF", filetypes=[("SPEF", "*.spef"), ("All", "*.*")])
        if not path:
            return
        name = simpledialog.askstring("Name", "Enter unique name:")
        if not name or name in self.spefs:
            messagebox.showerror("Error", "Invalid or duplicate name")
            return
        try:
            self.root.config(cursor="watch")
            self.root.update_idletasks()
            spef = SpefFile(path)
            spef.parse()
        except Exception as exc:
            messagebox.showerror("Error", f"Parse failed:\n{exc}")
            return
        finally:
            self.root.config(cursor="")
            self.root.update_idletasks()
        self.spefs[name] = spef
        # Use get_net_count() for efficiency with 1M+ nets
        net_count = spef.get_net_count() if hasattr(spef, 'get_net_count') else len(spef.nets)
        self.tree.insert("", "end", iid=name, values=(name, net_count, path))
        self._refresh_choices()

    def _remove_selected(self) -> None:
        for item in self.tree.selection():
            name = self.tree.item(item, "values")[0]
            if name in self.spefs:
                del self.spefs[name]
            self.tree.delete(item)
        self._refresh_choices()

    def _preload_spefs(self, paths: Iterable[str]) -> None:
        for path in paths:
            try:
                self._load_path(path)
            except Exception as exc:
                messagebox.showwarning("Warning", f"Failed: {path}\n{exc}")
    def _load_path(self, path: str, name: Optional[str] = None) -> None:
        if not path:
            return
        if name is None:
            base = os.path.splitext(os.path.basename(path))[0] or "spef"
            name = base
            i = 2
            while name in self.spefs:
                name = f"{base}_{i}"
                i += 1
        self.root.config(cursor="watch")
        self.root.update_idletasks()
        try:
            spef = SpefFile(path)
            spef.parse()
        finally:
            self.root.config(cursor="")
            self.root.update_idletasks()
        self.spefs[name] = spef
        # Use get_net_count() for efficiency with 1M+ nets
        net_count = spef.get_net_count() if hasattr(spef, 'get_net_count') else len(spef.nets)
        self.tree.insert("", "end", iid=name, values=(name, net_count, path))
        self._refresh_choices()
    def _refresh_choices(self) -> None:
        names = list(self.spefs.keys())
        self.ref_combo["values"] = names
        self.fit_combo["values"] = names
        if names:
            if not self.ref_var.get() or self.ref_var.get() not in names:
                self.ref_var.set(names[0])
            if not self.fit_var.get() or self.fit_var.get() not in names:
                self.fit_var.set(names[1] if len(names) > 1 else names[0])
    def _add_cap_data(self) -> None:
        path = filedialog.askopenfilename(title="Cap Data", filetypes=[("Data", "*.data"), ("All", "*.*")])
        if path:
            self._load_cap_data(path)
    
    def _add_res_data(self) -> None:
        path = filedialog.askopenfilename(title="Res Data", filetypes=[("Data", "*.data"), ("All", "*.*")])
        if path:
            self._load_res_data(path)

    def _add_ccap_data(self) -> None:
        path = filedialog.askopenfilename(title="Coupling Cap Data", filetypes=[("Data", "*.data"), ("All", "*.*")])
        if path:
            self._load_ccap_data(path)
    
    def _load_cap_data(self, path: str) -> None:
        """Load cap data using C++ backend."""
        try:
            has_existing = (
                self._cpp_result is not None and
                (
                    getattr(self._cpp_result, "res_count", 0) > 0 or
                    getattr(self._cpp_result, "ccap_count", 0) > 0
                )
            )
            # If we already have non-cap data in _cpp_result, merge; otherwise create new
            if has_existing:
                # Merge with existing res data
                new_cap = spef_core.create_plot_data_from_files(path, "", "")
                # Copy cap data to current result
                self._cpp_result.cap_c1 = new_cap.cap_c1
                self._cpp_result.cap_c2 = new_cap.cap_c2
                self._cpp_result.cap_net_names = new_cap.cap_net_names
                self._cpp_result.cap_count = new_cap.cap_count
                self._cpp_result.cap_correlation = new_cap.cap_correlation
            else:
                # Create new PlotData from cap file
                self._cpp_result = spef_core.create_plot_data_from_files(path, "", "")
            self.cap_data_label.config(text=f"{os.path.basename(path)} ({self._cpp_result.cap_count} nets)", foreground="black")
        except Exception as exc:
            messagebox.showerror("Error", f"Load failed:\n{exc}")
    
    def _load_res_data(self, path: str) -> None:
        """Load res data using C++ backend."""
        try:
            has_existing = (
                self._cpp_result is not None and
                (
                    getattr(self._cpp_result, "cap_count", 0) > 0 or
                    getattr(self._cpp_result, "ccap_count", 0) > 0
                )
            )
            # If we already have non-res data in _cpp_result, merge; otherwise create new
            if has_existing:
                # Merge with existing cap/ccap data
                new_res = spef_core.create_plot_data_from_files("", path, "")
                # Copy res data to current result
                self._cpp_result.res_r1 = new_res.res_r1
                self._cpp_result.res_r2 = new_res.res_r2
                self._cpp_result.res_net_names = new_res.res_net_names
                self._cpp_result.res_sink_names = new_res.res_sink_names
                self._cpp_result.res_driver_names = new_res.res_driver_names
                self._cpp_result.res_count = new_res.res_count
                self._cpp_result.res_correlation = new_res.res_correlation
            else:
                # Create new PlotData from res file
                self._cpp_result = spef_core.create_plot_data_from_files("", path, "")
            self.res_data_label.config(text=f"{os.path.basename(path)} ({self._cpp_result.res_count} pairs)", foreground="black")
        except Exception as exc:
            messagebox.showerror("Error", f"Load failed:\n{exc}")

    def _load_ccap_data(self, path: str) -> None:
        """Load coupling cap data using C++ backend."""
        try:
            has_existing = (
                self._cpp_result is not None and
                (
                    getattr(self._cpp_result, "cap_count", 0) > 0 or
                    getattr(self._cpp_result, "res_count", 0) > 0
                )
            )

            new_ccap = spef_core.create_plot_data_from_files("", "", path)
            if has_existing:
                self._cpp_result.ccap_c1 = new_ccap.ccap_c1
                self._cpp_result.ccap_c2 = new_ccap.ccap_c2
                self._cpp_result.ccap_net1_names = new_ccap.ccap_net1_names
                self._cpp_result.ccap_net2_names = new_ccap.ccap_net2_names
                self._cpp_result.ccap_count = new_ccap.ccap_count
                self._cpp_result.ccap_correlation = new_ccap.ccap_correlation
            else:
                self._cpp_result = new_ccap

            net1_names = list(getattr(self._cpp_result, "ccap_net1_names", []))
            net2_names = list(getattr(self._cpp_result, "ccap_net2_names", []))
            c1_vals = list(getattr(self._cpp_result, "ccap_c1", []))
            c2_vals = list(getattr(self._cpp_result, "ccap_c2", []))
            n = min(len(net1_names), len(net2_names), len(c1_vals), len(c2_vals))
            self._data_coupling_caps = [
                CouplingCapComparison(net1_names[i], net2_names[i], float(c1_vals[i]), float(c2_vals[i]))
                for i in range(n)
            ]

            self.ccap_data_label.config(text=f"{os.path.basename(path)} ({len(self._data_coupling_caps)} pairs)", foreground="black")
            if self._cpp_result is None or (self._cpp_result.cap_count == 0 and self._cpp_result.res_count == 0):
                self.view_mode_var.set("coupling_cap")
        except Exception as exc:
            messagebox.showerror("Error", f"Load failed:\n{exc}")

    def _run_analysis(self) -> None:
        """Run analysis using C++ backend."""
        ref = self.ref_var.get()
        fit = self.fit_var.get()
        if not ref or not fit or ref not in self.spefs or fit not in self.spefs:
            messagebox.showwarning("Warning", "Select two SPEF files")
            return
        try:
            s1 = self.spefs[ref]
            s2 = self.spefs[fit]
            # Always use C++ export_plot_data
            if hasattr(s1, '_cpp_spef') and s1._cpp_spef is not None and hasattr(s2, '_cpp_spef') and s2._cpp_spef is not None:
                self._cpp_result = spef_core.export_plot_data(s1._cpp_spef, s2._cpp_spef, 0)
                self._fanout_cache = None  # Invalidate cache when analysis changes
                try:
                    self._data_coupling_caps = compare_coupling_caps_cpp(s1, s2)
                    print(f"Found {len(self._data_coupling_caps)} coupling cap pairs")
                except Exception as e:
                    print(f"[warn] Coupling cap analysis failed: {e}")
                    self._data_coupling_caps = []
                
                self._update_plot()
            else:
                messagebox.showerror("Error", "SPEF files not parsed with C++ backend")
        except Exception as exc:
            messagebox.showerror("Error", f"Analysis failed:\n{exc}")
    def _run_from_data(self) -> None:
        has_plot_data = self._cpp_result is not None and (self._cpp_result.cap_count > 0 or self._cpp_result.res_count > 0)
        has_coupling_data = bool(self._data_coupling_caps)
        if not has_plot_data and not has_coupling_data:
            messagebox.showwarning("Warning", "No data loaded")
            return
        self._update_plot()
    def _auto_run(self) -> None:
        # If preloaded C++ result (PlotData) exists, use it directly
        if self._cpp_result is not None:
            self._update_plot()
            return
        # Otherwise run analysis from SPEF objects
        names = list(self.spefs.keys())
        if len(names) >= 2:
            self.ref_var.set(names[0])
            self.fit_var.set(names[1])
            self._run_analysis()

    def _update_plot(self) -> None:
        """Update plots with filters - uses C++ PlotData result."""
        self.ax_c.clear()
        self.ax_r.clear()
        
        # Parse filters
        flt = self._parse_filters()
        if flt is None:
            return
        
        view_mode = self.view_mode_var.get()
        
        if view_mode == "coupling_cap":
            # Show coupling capacitance scatter plot
            self._update_coupling_cap_plot(flt)
        else:
            # Show total cap + resistance correlation (existing behavior)
            if self._cpp_result is not None:
                self._update_plot_from_plotdata(self._cpp_result, flt)
        
        self.fig.tight_layout()
        self.canvas.draw()

    def _update_coupling_cap_plot(self, flt=None) -> None:
        """Plot coupling capacitance correlation in the ax_c and ax_r axes."""
        try:
            import numpy as np
        except ImportError:
            self.ax_c.set_title("coupling cap: numpy required")
            return
        
        # If no data yet, try computing now
        ref_name = self.ref_var.get()
        fit_name = self.fit_var.get()
        
        if not self._data_coupling_caps and ref_name in self.spefs and fit_name in self.spefs:
            try:
                s1 = self.spefs[ref_name]
                s2 = self.spefs[fit_name]
                self._data_coupling_caps = compare_coupling_caps_cpp(s1, s2)
            except Exception as e:
                self.ax_c.set_title(f"coupling cap: failed - {e}")
                return
        
        if not self._data_coupling_caps:
            self.ax_c.set_title("No coupling capacitance data available\n(Run Analysis first)")
            return
        
        # Apply filters
        coupling_points = self._filter_coupling_cap_points(self._data_coupling_caps, flt)
        
        if not coupling_points:
            self.ax_c.set_title("No coupling cap data after filtering")
            return
        
        c1_arr = np.array([p["c_ref"] for p in coupling_points])
        c2_arr = np.array([p["c_fit"] for p in coupling_points])
        
        min_c = float(np.minimum(c1_arr.min(), c2_arr.min()))
        max_c = float(np.maximum(c1_arr.max(), c2_arr.max()))
        span = max_c - min_c or 1.0
        pad = 0.05 * span
        vmin, vmax = min_c - pad, max_c + pad
        
        self.ax_c.plot([vmin, vmax], [vmin, vmax], "k--", linewidth=1.0)
        
        red_mask = c2_arr > c1_arr
        if np.any(red_mask):
            self.ax_c.plot(c1_arr[red_mask], c2_arr[red_mask], "o", markersize=3,
                          markerfacecolor="none", markeredgecolor="red", alpha=0.6)
        if np.any(~red_mask):
            self.ax_c.plot(c1_arr[~red_mask], c2_arr[~red_mask], "o", markersize=3,
                          markerfacecolor="none", markeredgecolor="blue", alpha=0.6)
        
        self.ax_c.set_xlim(vmin, vmax)
        self.ax_c.set_ylim(vmin, vmax)
        
        corr = pearson_corr(c1_arr.tolist(), c2_arr.tolist())
        title = f"Coupling Cap: {ref_name} (X) vs {fit_name} (Y)  n={len(coupling_points)}"
        if corr:
            title += f"  (corr={corr:.4f})"
        self.ax_c.set_title(title)
        self.ax_c.set_xlabel(f"{ref_name} Coupling Cap")
        self.ax_c.set_ylabel(f"{fit_name} Coupling Cap")
        self.ax_c.grid(True, alpha=0.3)
        
        # Cache for hover tooltip
        self._xs_c = c1_arr
        self._ys_c = c2_arr
        self._cap_points = coupling_points
        self._xs_r = None
        self._ys_r = None
        self._res_points = []
        self._kd_r = None
        
        # Build kd-tree if numpy available
        try:
            from scipy.spatial import cKDTree
            if len(coupling_points) > 100000:
                self._kd_c = cKDTree(np.column_stack((c1_arr, c2_arr)))
            else:
                self._kd_c = None
        except ImportError:
            self._kd_c = None
        
        # Show delta histogram in ax_r
        diffs = np.abs(c1_arr - c2_arr)
        self._draw_diff_histogram_ax(self.ax_r, diffs.tolist(), 
                                     f"Coupling Cap |Δ| Distribution (n={len(diffs)})")
        
        # Update corr label
        self.corr_label.config(text=f"corr={corr:.4f}" if corr else "corr=N/A")

    def _filter_coupling_cap_points(self, coupling_caps: List[CouplingCapComparison], flt) -> list:
        """Filter coupling cap points and return list of dicts for plotting."""
        if not coupling_caps:
            return []
        
        # Build total_cap lookups for percentage filtering
        total_cap_map_ref = {}
        total_cap_map_fit = {}
        ref_name = self.ref_var.get()
        fit_name = self.fit_var.get()
        if ref_name and ref_name in self.spefs:
            spef = self.spefs[ref_name]
            if hasattr(spef, '_cpp_spef') and spef._cpp_spef is not None:
                try:
                    for net_name, net_data in spef._cpp_spef.nets.items():
                        total_cap_map_ref[net_name] = net_data.total_cap
                except Exception:
                    pass
        if fit_name and fit_name in self.spefs:
            spef = self.spefs[fit_name]
            if hasattr(spef, '_cpp_spef') and spef._cpp_spef is not None:
                try:
                    for net_name, net_data in spef._cpp_spef.nets.items():
                        total_cap_map_fit[net_name] = net_data.total_cap
                except Exception:
                    pass
        
        points = []
        for cc in coupling_caps:
            c_ref = cc.c1
            c_fit = cc.c2
            
            total_ref = max(total_cap_map_ref.get(cc.net1, 0.0), total_cap_map_ref.get(cc.net2, 0.0))
            total_fit = max(total_cap_map_fit.get(cc.net1, 0.0), total_cap_map_fit.get(cc.net2, 0.0))
            pct_ref = (c_ref / total_ref * 100.0) if total_ref > 0 else 0.0
            pct_fit = (c_fit / total_fit * 100.0) if total_fit > 0 else 0.0

            if flt:
                p = {
                    "cc_ref": c_ref,
                    "cc_fit": c_fit,
                    "cc_pct_ref": pct_ref,
                }
                if not self._passes_filters(p, flt):
                    continue
            
            points.append({
                "net1": cc.net1,
                "net2": cc.net2,
                "c_ref": c_ref,
                "c_fit": c_fit,
                "pct_ref": pct_ref,
                "pct_fit": pct_fit,
            })
        
        return points

    def _update_plot_from_cpp(self, flt=None) -> None:
        """Extract data directly from C++ PlotData result with optional filters."""
        self._update_plot_from_plotdata(self._cpp_result, flt)

    def _update_plot_from_plotdata(self, plot_data, flt=None) -> None:
        """Fast plotting using numpy arrays from C++ - optimized for 1M+ nets.
        
        Args:
            plot_data: C++ PlotData object with numpy arrays
            flt: Optional filter dict from _parse_filters()
        """
        try:
            import numpy as np
        except ImportError:
            messagebox.showerror("Error", "numpy is required for PlotData visualization")
            return
        
        # Get numpy arrays directly (no Python loop overhead!)
        cap_c1 = np.asarray(plot_data.cap_c1)
        cap_c2 = np.asarray(plot_data.cap_c2)
        res_r1 = np.asarray(plot_data.res_r1)
        res_r2 = np.asarray(plot_data.res_r2)
        cap_net_names = list(plot_data.cap_net_names)
        res_net_names = list(plot_data.res_net_names)
        res_sink_names = list(plot_data.res_sink_names)
        res_driver_names = list(plot_data.res_driver_names)
        
        # Apply filters if provided
        if flt:
            # Build fanout lookup when fanout filter is active
            fanout_active = flt.get("min_fanout") is not None or flt.get("max_fanout") is not None
            fanout_map = self._get_fanout_cache() if fanout_active else {}
            get_fanout = fanout_map.get if fanout_active else lambda _: None
            # Build point dicts for filtering
            cap_points = []
            for i, name in enumerate(cap_net_names):
                p = {"net": name, "c_ref": float(cap_c1[i]), "c_fit": float(cap_c2[i]), "fanout": get_fanout(name)}
                cap_points.append(p)
            res_points = []
            for i, name in enumerate(res_net_names):
                p = {"net": name, "r_ref": float(res_r1[i]), "r_fit": float(res_r2[i]), "fanout": get_fanout(name)}
                res_points.append(p)
            
            # Filter
            cap_indices = [i for i, p in enumerate(cap_points) if self._passes_filters(p, flt)]
            res_indices = [i for i, p in enumerate(res_points) if self._passes_filters(p, flt)]
            
            cap_c1 = cap_c1[cap_indices]
            cap_c2 = cap_c2[cap_indices]
            cap_net_names = [cap_net_names[i] for i in cap_indices]
            
            res_r1 = res_r1[res_indices]
            res_r2 = res_r2[res_indices]
            res_net_names = [res_net_names[i] for i in res_indices]
            res_sink_names = [res_sink_names[i] for i in res_indices]
            res_driver_names = [res_driver_names[i] for i in res_indices]
        
        ref_name = self.ref_var.get() if hasattr(self, "ref_var") and self.ref_var.get() else "tool1"
        fit_name = self.fit_var.get() if hasattr(self, "fit_var") and self.fit_var.get() else "tool2"
        
        # Capacitance plot - vectorized operations
        if len(cap_c1) > 0:
            # Compute min/max (vectorized)
            min_c = float(np.minimum(cap_c1.min(), cap_c2.min()))
            max_c = float(np.maximum(cap_c1.max(), cap_c2.max()))
            span_c = max_c - min_c or 1.0
            pad_c = 0.05 * span_c
            vmin_c = min_c - pad_c
            vmax_c = max_c + pad_c
            
            # Vectorized color computation
            red_mask = cap_c2 > cap_c1
            
            self.ax_c.plot([vmin_c, vmax_c], [vmin_c, vmax_c], "k--", linewidth=1.0)
            
            # Plot red points
            if np.any(red_mask):
                self.ax_c.plot(cap_c1[red_mask], cap_c2[red_mask], "o", markersize=2, 
                             markerfacecolor="none", markeredgecolor="red", alpha=0.6)
            
            # Plot blue points
            if np.any(~red_mask):
                self.ax_c.plot(cap_c1[~red_mask], cap_c2[~red_mask], "o", markersize=2,
                             markerfacecolor="none", markeredgecolor="blue", alpha=0.6)
            
            self.ax_c.set_xlim(vmin_c, vmax_c)
            self.ax_c.set_ylim(vmin_c, vmax_c)
            
            # Correlation from C++ (already computed)
            corr = plot_data.cap_correlation
            title_c = f"Total C: {ref_name} (X) vs {fit_name} (Y)"
            if corr:
                title_c += f"  (corr={corr:.4f})"
            self.ax_c.set_title(title_c)
            self.ax_c.set_xlabel(f"{ref_name} C")
            self.ax_c.set_ylabel(f"{fit_name} C")
            self.ax_c.grid(True, alpha=0.3)
            
            # Cache for kd-tree (if available)
            self._xs_c = cap_c1
            self._ys_c = cap_c2
            cap_c1_list = cap_c1.tolist()
            cap_c2_list = cap_c2.tolist()
            self._cap_points = [
                {"net": n, "c_ref": c1, "c_fit": c2}
                for n, c1, c2 in zip(cap_net_names, cap_c1_list, cap_c2_list)
            ]
        
        # Resistance plot - vectorized operations
        if len(res_r1) > 0:
            min_r = float(np.minimum(res_r1.min(), res_r2.min()))
            max_r = float(np.maximum(res_r1.max(), res_r2.max()))
            span_r = max_r - min_r or 1.0
            pad_r = 0.05 * span_r
            vmin_r = min_r - pad_r
            vmax_r = max_r + pad_r
            
            red_mask_r = res_r2 > res_r1
            
            self.ax_r.plot([vmin_r, vmax_r], [vmin_r, vmax_r], "k--", linewidth=1.0)
            
            if np.any(red_mask_r):
                self.ax_r.plot(res_r1[red_mask_r], res_r2[red_mask_r], "o", markersize=2,
                              markerfacecolor="none", markeredgecolor="red", alpha=0.6)
            
            if np.any(~red_mask_r):
                self.ax_r.plot(res_r1[~red_mask_r], res_r2[~red_mask_r], "o", markersize=2,
                              markerfacecolor="none", markeredgecolor="blue", alpha=0.6)
            
            self.ax_r.set_xlim(vmin_r, vmax_r)
            self.ax_r.set_ylim(vmin_r, vmax_r)
            
            corr_r = plot_data.res_correlation
            title_r = f"Driver->sink R: {ref_name} (X) vs {fit_name} (Y)"
            if corr_r:
                title_r += f"  (corr={corr_r:.4f})"
            self.ax_r.set_title(title_r)
            self.ax_r.set_xlabel(f"{ref_name} R")
            self.ax_r.set_ylabel(f"{fit_name} R")
            self.ax_r.grid(True, alpha=0.3)
            
            # Cache for kd-tree
            self._xs_r = res_r1
            self._ys_r = res_r2
            res_r1_list = res_r1.tolist()
            res_r2_list = res_r2.tolist()
            self._res_points = [
                {"net": n, "r_ref": r1, "r_fit": r2, "load": s, "driver": d}
                for n, r1, r2, s, d in zip(res_net_names, res_r1_list, res_r2_list, res_sink_names, res_driver_names)
            ]
        
        self.fig.tight_layout()
        self.canvas.draw()

    def _draw_diff_histogram_ax(self, ax, diffs, title, n_bins=100):
        try:
            import numpy as np
        except ImportError:
            ax.set_title(title + "  [numpy required]")
            return
        ax.set_title(title)
        if len(diffs) == 0:
            return
        mean = float(sum(diffs)) / len(diffs)
        variance = sum((x - mean) ** 2 for x in diffs) / max(len(diffs) - 1, 1)
        stddev = math.sqrt(variance)
        # RMSE: sqrt(sum(x^2) / n) - root mean square of the differences
        rmse = math.sqrt(sum(x ** 2 for x in diffs) / max(len(diffs), 1))
        dmin = min(diffs)
        dmax = max(diffs)
        counts, bin_edges = np.histogram(diffs, bins=n_bins)
        # Center the x-axis on zero, not on the data range
        xmin = float(bin_edges[0])
        xmax = float(bin_edges[-1])
        max_abs = max(abs(xmin), abs(xmax))
        if max_abs == 0:
            max_abs = 1.0  # Avoid zero range
        x_range = max_abs * 1.1  # Add 10% padding
        xmin = -x_range
        xmax = x_range
        # Choose metric based on GUI selection
        metric = self.stat_metric_var.get()
        if metric == "rmse":
            s1 = rmse
            s2 = 2.0 * rmse
        else:  # stddev (default)
            s1 = stddev
            s2 = 2.0 * stddev
        # Pink: beyond ±2σ
        ax.axvspan(xmin, mean - s2, color="#FFB3DE", zorder=0)
        ax.axvspan(mean + s2, xmax, color="#FFB3DE", zorder=0)
        # Purple/lavender: ±1σ to ±2σ
        ax.axvspan(mean - s2, mean - s1, color="#AAAAEE", zorder=0)
        ax.axvspan(mean + s1, mean + s2, color="#AAAAEE", zorder=0)
        # Yellow: within ±1σ
        ax.axvspan(mean - s1, mean + s1, color="yellow", zorder=0)
        s3_val = 3.0 * (rmse if metric == "rmse" else stddev)
        for count, left, right in zip(counts, bin_edges[:-1], bin_edges[1:]):
            bin_center = (float(left) + float(right)) / 2.0
            dist = abs(bin_center - mean)
            if metric == "rmse":
                ref_val = rmse
                if ref_val == 0.0:
                    color = "darkgreen"
                elif dist <= s1:
                    color = "darkgreen"
                elif dist <= s2:
                    color = "blue"
                elif dist <= s3_val:
                    color = "red"
                else:
                    color = "black"
            else:  # stddev
                if stddev == 0.0:
                    color = "darkgreen"
                elif dist <= s1:
                    color = "darkgreen"
                elif dist <= s2:
                    color = "blue"
                elif dist <= s3_val:
                    color = "red"
                else:
                    color = "black"
            ax.bar(float(left), int(count), width=float(right - left), align="edge", color=color, zorder=2)
        ax.set_xlim(xmin, xmax)
        def _fmt(v):
            if v == 0.0:
                return "0"
            if abs(v) >= 0.001:
                return f"{v:.4f}"
            return f"{v:.4e}"
        # Build stats text based on selected metric
        if metric == "rmse":
            stats_text = (
                "Difference Stats:\n\n"
                f"Mean: {_fmt(mean)}\n\n"
                f"RMSE: {_fmt(rmse)}\n\n"
                f"Min: {_fmt(dmin)}\n\n"
                f"Max: {_fmt(dmax)}"
            )
        else:  # stddev
            stats_text = (
                "Difference Stats:\n\n"
                f"Mean: {_fmt(mean)}\n\n"
                f"StdDev: {_fmt(stddev)}\n\n"
                f"Min: {_fmt(dmin)}\n\n"
                f"Max: {_fmt(dmax)}"
            )
        ax.text(
            0.98, 0.97,
            stats_text,
            transform=ax.transAxes,
            verticalalignment="top",
            horizontalalignment="right",
            fontsize=9,
            fontfamily="monospace",
            bbox=dict(boxstyle="square,pad=0.6", facecolor="white", edgecolor="black", linewidth=1.2),
            zorder=5,
        )
        ax.text(
            0.98, 0.97,
            "Difference Stats:",
            transform=ax.transAxes,
            verticalalignment="top",
            horizontalalignment="right",
            fontsize=9,
            fontfamily="monospace",
            fontweight="bold",
            zorder=6,
        )
    def _show_histogram(self) -> None:
        try:
            import numpy as np
        except ImportError:
            messagebox.showerror("Error", "numpy required")
            return

        # Parse filters from UI
        flt = self._parse_filters()
        if flt is None:
            return

        win = tk.Toplevel(self.root)
        win.title("Difference Histogram")
        fig = Figure(figsize=(8, 6))
        ax1 = fig.add_subplot(2, 1, 1)
        ax2 = fig.add_subplot(2, 1, 2)

        # Helper to apply filters to data
        fanout_active = flt.get("min_fanout") is not None or flt.get("max_fanout") is not None
        fanout_map = self._get_fanout_cache() if fanout_active else {}
        get_fanout = fanout_map.get if fanout_active else lambda _: None

        def filter_cap_data(c1, c2, names):
            if not flt.get("min_c") and not flt.get("max_c") and not flt.get("min_fanout") and not flt.get("max_fanout"):
                return c1, c2, names
            indices = []
            for i in range(len(c1)):
                p = {"net": names[i], "c_ref": float(c1[i]), "c_fit": float(c2[i]), "fanout": get_fanout(names[i])}
                if self._passes_filters(p, flt):
                    indices.append(i)
            return c1[indices], c2[indices], [names[i] for i in indices]

        def filter_res_data(r1, r2, net_names, sink_names):
            if not flt.get("min_r") and not flt.get("max_r") and not flt.get("min_fanout") and not flt.get("max_fanout"):
                return r1, r2, net_names, sink_names
            indices = []
            for i in range(len(r1)):
                p = {"net": net_names[i], "r_ref": float(r1[i]), "r_fit": float(r2[i]), "fanout": get_fanout(net_names[i])}
                if self._passes_filters(p, flt):
                    indices.append(i)
            return r1[indices], r2[indices], [net_names[i] for i in indices], [sink_names[i] for i in indices]

        if self.view_mode_var.get() == "coupling_cap":
            points = self._filter_coupling_cap_points(self._data_coupling_caps, flt)
            diffs = np.abs(np.asarray([p["c_ref"] for p in points]) - np.asarray([p["c_fit"] for p in points])) if points else np.asarray([])
            self._draw_diff_histogram_ax(ax1, diffs.tolist(), f"Coupling Cap Diff (n={len(diffs)}, filtered)")
            ax2.set_title("Coupling % (ref) Distribution")
            if points:
                pct = np.asarray([p.get("pct_ref", 0.0) for p in points])
                ax2.hist(pct, bins=80, color="#3b82f6", alpha=0.8)
                ax2.set_xlabel("Coupling % of total cap (ref)")
                ax2.set_ylabel("Count")
                ax2.grid(True, alpha=0.3)
            else:
                ax2.text(0.5, 0.5, "No data after filtering", transform=ax2.transAxes,
                         ha="center", va="center")
        else:
            # Capacitance histogram
            if self._cpp_result is not None and self._cpp_result.cap_count > 0:
                cap_c1 = np.asarray(self._cpp_result.cap_c1)
                cap_c2 = np.asarray(self._cpp_result.cap_c2)
                cap_names = list(self._cpp_result.cap_net_names)
                cap_c1_f, cap_c2_f, _ = filter_cap_data(cap_c1, cap_c2, cap_names)
                diffs = np.abs(cap_c1_f - cap_c2_f)
                self._draw_diff_histogram_ax(ax1, diffs, f"Cap Diff (n={len(diffs)}, filtered)")
            elif getattr(self, '_xs_c', None) is not None and len(self._xs_c):
                # Use cached data from plotting
                cap_c1 = np.asarray(self._xs_c)
                cap_c2 = np.asarray(self._ys_c)
                cap_names = [p["net"] for p in getattr(self, '_cap_points', [])]
                cap_c1_f, cap_c2_f, _ = filter_cap_data(cap_c1, cap_c2, cap_names)
                diffs = np.abs(cap_c1_f - cap_c2_f)
                self._draw_diff_histogram_ax(ax1, diffs, f"Cap Diff (n={len(diffs)}, filtered)")

            # Resistance histogram
            if self._cpp_result is not None and self._cpp_result.res_count > 0:
                res_r1 = np.asarray(self._cpp_result.res_r1)
                res_r2 = np.asarray(self._cpp_result.res_r2)
                res_net_names = list(self._cpp_result.res_net_names)
                res_sink_names = list(self._cpp_result.res_sink_names)
                res_r1_f, res_r2_f, _, _ = filter_res_data(res_r1, res_r2, res_net_names, res_sink_names)
                diffs = np.abs(res_r1_f - res_r2_f)
                self._draw_diff_histogram_ax(ax2, diffs, f"Res Diff (n={len(diffs)}, filtered)")
            elif getattr(self, '_xs_r', None) is not None and len(self._xs_r):
                res_r1 = np.asarray(self._xs_r)
                res_r2 = np.asarray(self._ys_r)
                res_net_names = [p["net"] for p in getattr(self, '_res_points', [])]
                res_sink_names = [p.get("load", "") for p in getattr(self, '_res_points', [])]
                res_r1_f, res_r2_f, _, _ = filter_res_data(res_r1, res_r2, res_net_names, res_sink_names)
                diffs = np.abs(res_r1_f - res_r2_f)
                self._draw_diff_histogram_ax(ax2, diffs, f"Res Diff (n={len(diffs)}, filtered)")

        fig.tight_layout()
        FigureCanvasTkAgg(fig, master=win).get_tk_widget().pack(fill="both", expand=True)


if __name__ == "__main__":
    main()
