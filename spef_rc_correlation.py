#!/usr/bin/env python3
# -*- coding: utf-8 -*-
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
@dataclass
class CapComparison:
    net: str
    c1: float
    c2: float

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

class SpefFile:
    """Simple SPEF wrapper that uses C++ backend."""
    def __init__(self, path: str):
        self.path = path
        self.name_map: Dict[str, str] = {}
        self.nets: Dict[str, NetRC] = {}
        self.t_unit = 'NS'
        self.c_unit = 'PF'
        self.r_unit = 'OHM'
        self.l_unit = 'HENRY'
        self._cpp_spef = None
    
    def parse(self) -> None:
        """Parse SPEF file using C++ backend."""
        if HAS_CPP:
            self._parse_cpp()
        else:
            raise RuntimeError("C++ extension required for parsing")
    
    def _parse_cpp(self) -> None:
        """Parse using C++ extension and convert to Python objects."""
        print(f"[{self.path}] parsing with C++ extension...")
        cpp_spef = spef_core.parse_spef(self.path)
        self._cpp_spef = cpp_spef
        self.name_map = dict(cpp_spef.name_map)
        nmap = self.name_map
        
        def _resolve_node(tok: str) -> str:
            if not tok or tok[0] != '*':
                return tok
            if ':' in tok:
                idx = tok.index(':')
                base = tok[:idx]
                pin = tok[idx + 1:]
                resolved = nmap.get(base, base)
                return f"{resolved}:{pin}" if pin else resolved
            return nmap.get(tok, tok)
        
        for net_name, cpp_net in cpp_spef.nets.items():
            net = NetRC(
                name=cpp_net.name,
                total_cap=cpp_net.total_cap,
                driver=cpp_net.driver,
                sinks=list(cpp_net.sinks)
            )
            for node, edges in cpp_net.res_graph.items():
                resolved_node = _resolve_node(node)
                net.res_graph[resolved_node] = [
                    (_resolve_node(e.to), e.weight) for e in edges
                ]
            self.nets[net_name] = net
        
        self.t_unit = cpp_spef.t_unit
        self.c_unit = cpp_spef.c_unit
        self.r_unit = cpp_spef.r_unit
        self.l_unit = cpp_spef.l_unit
        print(f"[{self.path}] parsed {len(self.nets)} nets... (C++)")


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


def compare_spef_cpp_objs(spef1: SpefFile, spef2: SpefFile, num_threads: int = 0):
    """Compare two already-parsed SPEF objects using C++ backend."""
    if not HAS_CPP:
        raise RuntimeError("C++ extension not available")
    result = spef_core.compare_spef_full(spef1._cpp_spef, spef2._cpp_spef, num_threads)
    caps = [CapComparison(c.net_name, c.c1, c.c2) for c in result.cap_rows]
    ress = [ResComparison(r.net_name, r.driver, r.sink, r.r1, r.r2) for r in result.res_rows]
    top_10_cap = [CapComparison(c.net_name, c.c1, c.c2) for c in result.top_10_cap]
    top_10_res = [ResComparison(r.net_name, r.driver, r.sink, r.r1, r.r2) for r in result.top_10_res]
    return caps, ress, top_10_cap, top_10_res


def compare_spef_cpp_to_files(spef1_path: str, spef2_path: str, cap_out: str, res_out: str, num_threads: int = 0) -> Tuple[List[CapComparison], List[ResComparison]]:
    """Compare and write data files."""
    caps, ress, _, _ = compare_spef_cpp(spef1_path, spef2_path, num_threads)
    
    with open(cap_out, 'w', encoding='utf-8') as fc, open(res_out, 'w', encoding='utf-8') as fr:
        for cap in caps:
            print(f"{cap.net} {cap.c1} {cap.c2}", file=fc)
        for res in ress:
            print(f"{res.net} {res.driver} {res.load} {res.r1} {res.r2}", file=fr)
    
    return caps, ress


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


def _parse_one(path: str) -> SpefFile:
    """Parse a single SPEF file."""
    sf = SpefFile(path)
    sf.parse()
    return sf


def parse_spefs_parallel(path1: str, path2: str) -> Tuple[SpefFile, SpefFile]:
    """Parse two SPEF files concurrently using C++ multithreaded backend."""
    if not HAS_CPP:
        raise RuntimeError("C++ extension not available")
    print(f"Parsing {path1} and {path2} in parallel (C++ threads)...")
    try:
        cpp_spefs = spef_core.parse_spef_parallel([path1, path2], 2)
        # Wrap in SpefFile for compatibility with rest of code
        s1 = SpefFile(path1)
        s1._cpp_spef = cpp_spefs[0]
        s1.name_map = dict(cpp_spefs[0].name_map)
        s1.t_unit = cpp_spefs[0].t_unit
        s1.c_unit = cpp_spefs[0].c_unit
        s1.r_unit = cpp_spefs[0].r_unit
        s1.l_unit = cpp_spefs[0].l_unit
        s1.nets = {}
        nmap = s1.name_map
        def _resolve_node(tok: str) -> str:
            if not tok or tok[0] != '*':
                return tok
            if ':' in tok:
                idx = tok.index(':')
                base = tok[:idx]
                pin = tok[idx + 1:]
                resolved = nmap.get(base, base)
                return f"{resolved}:{pin}" if pin else resolved
            return nmap.get(tok, tok)
        for net_name, cpp_net in cpp_spefs[0].nets.items():
            net = NetRC(
                name=cpp_net.name,
                total_cap=cpp_net.total_cap,
                driver=cpp_net.driver,
                sinks=list(cpp_net.sinks)
            )
            for node, edges in cpp_net.res_graph.items():
                resolved_node = _resolve_node(node)
                net.res_graph[resolved_node] = [
                    (_resolve_node(e.to), e.weight) for e in edges
                ]
            s1.nets[net_name] = net

        s2 = SpefFile(path2)
        s2._cpp_spef = cpp_spefs[1]
        s2.name_map = dict(cpp_spefs[1].name_map)
        s2.t_unit = cpp_spefs[1].t_unit
        s2.c_unit = cpp_spefs[1].c_unit
        s2.r_unit = cpp_spefs[1].r_unit
        s2.l_unit = cpp_spefs[1].l_unit
        s2.nets = {}
        nmap2 = s2.name_map
        def _resolve_node2(tok: str) -> str:
            if not tok or tok[0] != '*':
                return tok
            if ':' in tok:
                idx = tok.index(':')
                base = tok[:idx]
                pin = tok[idx + 1:]
                resolved = nmap2.get(base, base)
                return f"{resolved}:{pin}" if pin else resolved
            return nmap2.get(tok, tok)
        for net_name, cpp_net in cpp_spefs[1].nets.items():
            net = NetRC(
                name=cpp_net.name,
                total_cap=cpp_net.total_cap,
                driver=cpp_net.driver,
                sinks=list(cpp_net.sinks)
            )
            for node, edges in cpp_net.res_graph.items():
                resolved_node = _resolve_node2(node)
                net.res_graph[resolved_node] = [
                    (_resolve_node2(e.to), e.weight) for e in edges
                ]
            s2.nets[net_name] = net
        return s1, s2
    except Exception as exc:
        print(f"[warn] C++ parallel parse failed ({exc}), falling back to sequential")
        s1 = SpefFile(path1)
        s1.parse()
        s2 = SpefFile(path2)
        s2.parse()
        return s1, s2


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
            # 先解析 SPEF 并自动比对，结果传递给 GUI
            s1, s2 = parse_spefs_parallel(args.spef1, args.spef2)
            result = spef_core.compare_spef_full(s1._cpp_spef, s2._cpp_spef, args.threads)
            caps = [CapComparison(c.net_name, c.c1, c.c2) for c in result.cap_rows]
            ress = [ResComparison(r.net_name, r.driver, r.sink, r.r1, r.r2) for r in result.res_rows]
            launch_gui(
                preload_paths=None,
                auto_run=True,
                preload_caps=caps,
                preload_ress=ress,
                preload_spef_objs=[
                    (os.path.splitext(os.path.basename(args.spef1))[0], s1),
                    (os.path.splitext(os.path.basename(args.spef2))[0], s2)
                ]
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
            caps, ress, top_10_cap, top_10_res = compare_spef_cpp(args.spef1, args.spef2, args.threads)
            with open(cap_out, 'w') as fc, open(res_out, 'w') as fr:
                for cap in caps:
                    print(f"{cap.net} {cap.c1} {cap.c2}", file=fc)
                for res in ress:
                    print(f"{res.net} {res.driver} {res.load} {res.r1} {res.r2}", file=fr)
            print(f"Cap data written to: {cap_out}")
            print(f"Res data written to: {res_out}")
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
    if args.net_cap_data or args.net_res_data:
        if args.gui or args.gui_auto_run:
            launch_gui(
                collect_spef_paths([p for p in [args.spef1, args.spef2] if p]),
                auto_run=args.gui_auto_run,
                preload_cap_data=args.net_cap_data,
                preload_res_data=args.net_res_data,
            )
            return

        caps = parse_net_cap_data(args.net_cap_data) if args.net_cap_data else []
        ress = parse_net_res_data(args.net_res_data) if args.net_res_data else []

        print("=== Summary (from data files) ===")
        if caps:
            print(f"Cap entries: {len(caps)}")
            corr_c = pearson_corr([c.c1 for c in caps], [c.c2 for c in caps])
            print(f"Cap correlation: {corr_c:.6f}" if corr_c else "Cap correlation: N/A")
        if ress:
            print(f"Res entries: {len(ress)}")
            corr_r = pearson_corr([r.r1 for r in ress], [r.r2 for r in ress])
            print(f"Res correlation: {corr_r:.6f}" if corr_r else "Res correlation: N/A")

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
    preload_caps: Optional[List[CapComparison]] = None,
    preload_ress: Optional[List[ResComparison]] = None,
    preload_spef_objs: Optional[list] = None
) -> None:
    """Launch interactive GUI."""

    if tk is None or FigureCanvasTkAgg is None or Figure is None:
        print("GUI requires tkinter and matplotlib.")
        return

    root = tk.Tk()
    RcCorrApp(root, preload_paths, auto_run, preload_cap_data, preload_res_data, preload_caps, preload_ress, preload_spef_objs)
    root.mainloop()


class RcCorrApp:
    def __init__(self, root, preload_paths=None, auto_run=False,
                 preload_cap_data=None, preload_res_data=None,
                 preload_caps=None, preload_ress=None, preload_spef_objs=None):
        self.root = root
        self.root.title("SPEF RC Correlation")
        self.spefs: Dict[str, SpefFile] = {}
        self._data_caps: List[CapComparison] = []
        self._data_ress: List[ResComparison] = []
        self.ref_var = tk.StringVar()
        self.fit_var = tk.StringVar()
        self.r_agg_var = tk.StringVar(value="max")
        self._auto_run_requested = auto_run
        self._build_ui()

        if preload_spef_objs:
            for name, spef in preload_spef_objs:
                self.spefs[name] = spef
                self.tree.insert("", "end", iid=name, values=(name, len(spef.nets), spef.path))
            self._refresh_choices()
        elif preload_paths:
            self._preload_spefs(preload_paths)
        if preload_cap_data:
            self._load_cap_data(preload_cap_data)
        if preload_res_data:
            self._load_res_data(preload_res_data)
        if preload_caps:
            self._data_caps = list(preload_caps)
            self.cap_data_label.config(text=f"(preloaded) ({len(preload_caps)} nets)", foreground="black")
        if preload_ress:
            self._data_ress = list(preload_ress)
            self.res_data_label.config(text=f"(preloaded) ({len(preload_ress)} pairs)", foreground="black")

        # 自动分析逻辑：如果 auto_run=True 且 spef 文件数>=2，则自动运行分析
        if self._auto_run_requested:
            if len(self.spefs) >= 2:
                self.root.after(0, self._auto_run)
            # 也可根据需求支持 data file 自动分析
            elif self._data_caps or self._data_ress:
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

        ttk.Frame(dfrm, height=5).pack()
        ttk.Button(dfrm, text="Run from Data Files", command=self._run_from_data).pack(fill="x", padx=5, pady=(0, 4))

        # Settings & Filters frame
        srm = ttk.LabelFrame(self.root, text="Settings & Filters")
        srm.pack(fill="x", padx=5, pady=5)

        # Row 0: reference & fit selection
        ttk.Label(srm, text="Reference:").grid(row=0, column=0, sticky="w", padx=5, pady=2)
        self.ref_combo = ttk.Combobox(srm, textvariable=self.ref_var, state="readonly", width=15)
        self.ref_combo.grid(row=0, column=1, sticky="w", padx=5, pady=2)

        ttk.Label(srm, text="Fit:").grid(row=0, column=2, sticky="w", padx=5, pady=2)
        self.fit_combo = ttk.Combobox(srm, textvariable=self.fit_var, state="readonly", width=15)
        self.fit_combo.grid(row=0, column=3, sticky="w", padx=5, pady=2)

        # Row 1: fanout range
        self.min_fanout_var = tk.StringVar(value="1")
        self.max_fanout_var = tk.StringVar(value="99999")
        ttk.Label(srm, text="Fanout range:").grid(row=1, column=0, sticky="w", padx=5, pady=2)
        ttk.Entry(srm, textvariable=self.min_fanout_var, width=8).grid(row=1, column=1, sticky="w", padx=5, pady=2)
        ttk.Label(srm, text="to").grid(row=1, column=2, sticky="w", padx=2, pady=2)
        ttk.Entry(srm, textvariable=self.max_fanout_var, width=8).grid(row=1, column=3, sticky="w", padx=5, pady=2)

        # Row 2: cap range (reference)
        self.min_c_var = tk.StringVar(value="0.001")
        self.max_c_var = tk.StringVar(value="2")
        ttk.Label(srm, text="Cap range (ref C):").grid(row=2, column=0, sticky="w", padx=5, pady=2)
        ttk.Entry(srm, textvariable=self.min_c_var, width=10).grid(row=2, column=1, sticky="w", padx=5, pady=2)
        ttk.Label(srm, text="to").grid(row=2, column=2, sticky="w", padx=2, pady=2)
        ttk.Entry(srm, textvariable=self.max_c_var, width=10).grid(row=2, column=3, sticky="w", padx=5, pady=2)

        # Row 3: R range (reference, aggregated)
        self.min_r_var = tk.StringVar(value="1")
        self.max_r_var = tk.StringVar(value="1000000")
        ttk.Label(srm, text="R range (ref, agg):").grid(row=3, column=0, sticky="w", padx=5, pady=2)
        ttk.Entry(srm, textvariable=self.min_r_var, width=10).grid(row=3, column=1, sticky="w", padx=5, pady=2)
        ttk.Label(srm, text="to").grid(row=3, column=2, sticky="w", padx=2, pady=2)
        ttk.Entry(srm, textvariable=self.max_r_var, width=10).grid(row=3, column=3, sticky="w", padx=5, pady=2)

        # Row 4: R aggregation, run button, correlation label
        ttk.Label(srm, text="R aggregation:").grid(row=4, column=0, sticky="w", padx=5, pady=2)
        ttk.Combobox(srm, textvariable=self.r_agg_var, values=["max", "avg", "total"], state="readonly", width=8).grid(row=4, column=1, sticky="w", padx=5, pady=2)

        ttk.Button(srm, text="Run Analysis", command=self._run_analysis).grid(row=4, column=2, sticky="w", padx=5, pady=2)
        ttk.Button(srm, text="Diff Histogram", command=self._show_histogram).grid(row=4, column=4, sticky="w", padx=5, pady=2)

        self.corr_label = ttk.Label(srm, text="")
        self.corr_label.grid(row=4, column=3, sticky="w", padx=5, pady=2)

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
                "min_r": to_float(self.min_r_var.get()),
                "max_r": to_float(self.max_r_var.get()),
            }
        except Exception as exc:
            import tkinter.messagebox as messagebox
            messagebox.showerror("Invalid filter", f"Filter values must be numeric.\n{exc}")
            return None

    def _passes_filters(self, p, flt):
        mf = flt["min_fanout"]
        xf = flt["max_fanout"]
        if mf is not None and p.get("fanout", 0) < mf:
            return False
        if xf is not None and p.get("fanout", 0) > xf:
            return False
        if "c_ref" in p:
            mc = flt["min_c"]
            xc = flt["max_c"]
            if mc is not None and p["c_ref"] < mc:
                return False
            if xc is not None and p["c_ref"] > xc:
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
        self.tree.insert("", "end", iid=name, values=(name, len(spef.nets), path))
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
        self.tree.insert("", "end", iid=name, values=(name, len(spef.nets), path))
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
    def _load_cap_data(self, path: str) -> None:
        try:
            self._data_caps = parse_net_cap_data(path)
            self.cap_data_label.config(text=f"{os.path.basename(path)} ({len(self._data_caps)} nets)", foreground="black")
        except Exception as exc:
            messagebox.showerror("Error", f"Load failed:\n{exc}")
    def _add_res_data(self) -> None:
        path = filedialog.askopenfilename(title="Res Data", filetypes=[("Data", "*.data"), ("All", "*.*")])
        if path:
            self._load_res_data(path)
    def _load_res_data(self, path: str) -> None:
        try:
            self._data_ress = parse_net_res_data(path)
            self.res_data_label.config(text=f"{os.path.basename(path)} ({len(self._data_ress)} pairs)", foreground="black")
        except Exception as exc:
            messagebox.showerror("Error", f"Load failed:\n{exc}")
    def _run_analysis(self) -> None:
        ref = self.ref_var.get()
        fit = self.fit_var.get()
        if not ref or not fit or ref not in self.spefs or fit not in self.spefs:
            messagebox.showwarning("Warning", "Select two SPEF files")
            return
        try:
            s1 = self.spefs[ref]
            s2 = self.spefs[fit]
            if hasattr(s1, '_cpp_spef') and s1._cpp_spef is not None and hasattr(s2, '_cpp_spef') and s2._cpp_spef is not None:
                caps, ress, _, _ = compare_spef_cpp_objs(s1, s2)
            else:
                caps, ress, _, _ = compare_spef_cpp(s1.path, s2.path)
            self._data_caps = caps
            self._data_ress = ress
            self._update_plot()
        except Exception as exc:
            messagebox.showerror("Error", f"Analysis failed:\n{exc}")
    def _run_from_data(self) -> None:
        if not self._data_caps and not self._data_ress:
            messagebox.showwarning("Warning", "No data loaded")
            return
        self._update_plot()
    def _auto_run(self) -> None:
        names = list(self.spefs.keys())
        if len(names) >= 2:
            self.ref_var.set(names[0])
            self.fit_var.set(names[1])
            self._run_analysis()

    def _update_plot(self) -> None:
        self.ax_c.clear()
        self.ax_r.clear()
        # 过滤数据
        flt = self._parse_filters()
        points_cap = []
        for c in self._data_caps:
            p = {"net": c.net, "c_ref": c.c1, "c_fit": c.c2, "fanout": 0}
            points_cap.append(p)
        points_res = []
        for r in self._data_ress:
            p = {"net": r.net, "driver": r.driver, "load": r.load, "r_ref": r.r1, "r_fit": r.r2, "fanout": 0}
            points_res.append(p)
        # 若有spefs，补充fanout
        if hasattr(self, "spefs") and self.spefs:
            ref_name = self.ref_var.get()
            ref = self.spefs.get(ref_name)
            if ref:
                for p in points_cap:
                    net = p["net"]
                    if net in ref.nets:
                        p["fanout"] = len(ref.nets[net].sinks)
                for p in points_res:
                    net = p["net"]
                    if net in ref.nets:
                        p["fanout"] = len(ref.nets[net].sinks)
        # 过滤
        if flt:
            points_cap = [p for p in points_cap if self._passes_filters(p, flt)]
            points_res = [p for p in points_res if self._passes_filters(p, flt)]
        self._cache_plot_arrays(points_cap, points_res)
        xs = [p["c_ref"] for p in points_cap]
        ys = [p["c_fit"] for p in points_cap]
        ref_name = self.ref_var.get() if hasattr(self, "ref_var") else "tool1"
        fit_name = self.fit_var.get() if hasattr(self, "fit_var") else "tool2"
        if xs and ys:
            min_c = min(xs + ys)
            max_c = max(xs + ys)
            span_c = max_c - min_c or 1.0
            pad_c = 0.05 * span_c
            vmin_c = min_c - pad_c
            vmax_c = max_c + pad_c
            colors_c = ["red" if y > x else "blue" for x, y in zip(xs, ys)]
            self.ax_c.plot([vmin_c, vmax_c], [vmin_c, vmax_c], "k--", linewidth=1.0)
            # Plot red points first, then blue points
            red_idxs = [i for i, c in enumerate(colors_c) if c == "red"]
            blue_idxs = [i for i, c in enumerate(colors_c) if c == "blue"]
            if red_idxs:
                self.ax_c.plot([xs[i] for i in red_idxs], [ys[i] for i in red_idxs], "o", markersize=2, markerfacecolor="none", markeredgecolor="red", alpha=0.6)
            if blue_idxs:
                self.ax_c.plot([xs[i] for i in blue_idxs], [ys[i] for i in blue_idxs], "o", markersize=2, markerfacecolor="none", markeredgecolor="blue", alpha=0.6)
            self.ax_c.set_xlim(vmin_c, vmax_c)
            self.ax_c.set_ylim(vmin_c, vmax_c)
            corr = pearson_corr(xs, ys)
            title_c = f"Total C: {ref_name} (X) vs {fit_name} (Y)"
            if corr is not None:
                title_c += f"  (corr={corr:.4f})"
                self.corr_label.config(text=f"Cap corr: {corr:.4f}")
            self.ax_c.set_title(title_c)
            self.ax_c.set_xlabel(f"{ref_name} C")
            self.ax_c.set_ylabel(f"{fit_name} C")
            self.ax_c.grid(True, alpha=0.3)
        xs = [p["r_ref"] for p in points_res]
        ys = [p["r_fit"] for p in points_res]
        if xs and ys:
            min_r = min(xs + ys)
            max_r = max(xs + ys)
            span_r = max_r - min_r or 1.0
            pad_r = 0.05 * span_r
            vmin_r = min_r - pad_r
            vmax_r = max_r + pad_r
            colors_r = ["red" if y > x else "blue" for x, y in zip(xs, ys)]
            self.ax_r.plot([vmin_r, vmax_r], [vmin_r, vmax_r], "k--", linewidth=1.0)
            # Plot red points first, then blue points
            red_idxs = [i for i, c in enumerate(colors_r) if c == "red"]
            blue_idxs = [i for i, c in enumerate(colors_r) if c == "blue"]
            if red_idxs:
                self.ax_r.plot([xs[i] for i in red_idxs], [ys[i] for i in red_idxs], "o", markersize=2, markerfacecolor="none", markeredgecolor="red", alpha=0.6)
            if blue_idxs:
                self.ax_r.plot([xs[i] for i in blue_idxs], [ys[i] for i in blue_idxs], "o", markersize=2, markerfacecolor="none", markeredgecolor="blue", alpha=0.6)
            self.ax_r.set_xlim(vmin_r, vmax_r)
            self.ax_r.set_ylim(vmin_r, vmax_r)
            corr = pearson_corr(xs, ys)
            title_r = f"Driver->sink R: {ref_name} (X) vs {fit_name} (Y)"
            if corr is not None:
                title_r += f"  (corr={corr:.4f})"
            self.ax_r.set_title(title_r)
            self.ax_r.set_xlabel(f"{ref_name} R")
            self.ax_r.set_ylabel(f"{fit_name} R")
            self.ax_r.grid(True, alpha=0.3)
        self.fig.tight_layout()
        self.canvas.draw()

    def _draw_diff_histogram_ax(self, ax, diffs, title, n_bins=100):
        try:
            import numpy as np
        except ImportError:
            ax.set_title(title + "  [numpy required]")
            return
        ax.set_title(title)
        if not diffs:
            return
        mean = float(sum(diffs)) / len(diffs)
        variance = sum((x - mean) ** 2 for x in diffs) / max(len(diffs) - 1, 1)
        stddev = math.sqrt(variance)
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
        s3 = 3.0 * stddev
        for count, left, right in zip(counts, bin_edges[:-1], bin_edges[1:]):
            bin_center = (float(left) + float(right)) / 2.0
            dist = abs(bin_center - mean)
            if stddev == 0.0:
                color = "darkgreen"
            elif dist <= s1:
                color = "darkgreen"
            elif dist <= s2:
                color = "blue"
            elif dist <= s3:
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
        win = tk.Toplevel(self.root)
        win.title("Difference Histogram")
        fig = Figure(figsize=(8, 6))
        ax1 = fig.add_subplot(2, 1, 1)
        ax2 = fig.add_subplot(2, 1, 2)
        if self._data_caps:
            diffs = [abs(c.c1 - c.c2) for c in self._data_caps]
            if diffs:
                self._draw_diff_histogram_ax(ax1, diffs, f"Cap Diff (n={len(diffs)})")
        if self._data_ress:
            diffs = [abs(r.r1 - r.r2) for r in self._data_ress]
            if diffs:
                self._draw_diff_histogram_ax(ax2, diffs, f"Res Diff (n={len(diffs)})")
        fig.tight_layout()
        FigureCanvasTkAgg(fig, master=win).get_tk_widget().pack(fill="both", expand=True)


if __name__ == "__main__":
    main()
