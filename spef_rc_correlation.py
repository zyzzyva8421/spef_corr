#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simple SPEF RC correlation tool.

Given two SPEF files from different extraction tools, this script:
  - Parses nets and total capacitance
  - Identifies driver and sink pins per net
  - Builds per-net resistance graphs and computes driver-to-sink resistance
  - For overlapping nets between the two SPEFs, computes:
      * Correlation of total capacitance
      * Correlation of driver-to-sink resistance over common (net, sink) pairs

Usage (example):
    python spef_rc_correlation.py toolA.spef toolB.spef \
        --csv-prefix rc_corr

This will print a text summary and also write:
    rc_corr_caps.csv  - per-net total C comparison
    rc_corr_res.csv   - per (net, sink) driver-sink R comparison

The parser is intentionally conservative but should work for typical digital SPEF
from sign-off extraction tools. It understands:
  - *NAME_MAP
  - *D_NET, *CONN, *CAP, *RES sections

Limitations:
  - Assumes one main driver per net (direction O or B in *CONN)
  - For driver/sink nodes that do not appear exactly in *RES, it tries to match
    by prefix (e.g. pin, pin:1, etc.). If still not found, that sink is skipped.
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Optional, Iterable, Set
import heapq


@dataclass
class NetRC:
    name: str
    total_cap: float
    driver: Optional[str] = None
    sinks: List[str] = field(default_factory=list)
    # resistance graph: node -> list of (neighbor, R)
    res_graph: Dict[str, List[Tuple[str, float]]] = field(default_factory=dict)

    # cache for driver->sink resistance
    _driver_sink_res_cache: Optional[Dict[str, float]] = field(default=None, init=False, repr=False)
    # cache for mapping pin prefix (before ':') to an existing node name
    _node_prefix_map: Optional[Dict[str, str]] = field(default=None, init=False, repr=False)

    def _find_best_node_name(self, pin: str) -> Optional[str]:
        """Try to map a pin name from *CONN to a node name in *RES graph.

        Many SPEF writers use pin, pin:1, pin:2, etc. This function first
        checks exact match, then tries prefix matches.
        """
        if pin in self.res_graph:
            return pin
        # Build prefix map lazily for faster lookup on large nets
        if self._node_prefix_map is None:
            mp: Dict[str, str] = {}
            for node in self.res_graph.keys():
                base = node.split(":", 1)[0]
                if base not in mp:
                    mp[base] = node
            self._node_prefix_map = mp
        return self._node_prefix_map.get(pin)

    def _shortest_resistance(self, src: str, dst: str) -> Optional[float]:
        """Dijkstra shortest path on resistance graph between src and dst.

        Returns total resistance, or None if unreachable or src/dst missing.
        """
        if src not in self.res_graph or dst not in self.res_graph:
            return None

        if src == dst:
            return 0.0

        # Standard Dijkstra with a min-heap
        dist: Dict[str, float] = {src: 0.0}
        visited: Set[str] = set()
        heap: List[Tuple[float, str]] = [(0.0, src)]

        while heap:
            d, node = heapq.heappop(heap)
            if node in visited:
                continue
            visited.add(node)
            if node == dst:
                return d
            for neigh, r in self.res_graph.get(node, []):
                nd = d + r
                if nd < dist.get(neigh, math.inf):
                    dist[neigh] = nd
                    heapq.heappush(heap, (nd, neigh))
        return None

    def driver_sink_resistances(self) -> Dict[str, float]:
        """Return map: sink_pin -> R(driver->sink).

        Only includes sinks where a path from driver exists in the graph.
        Result is cached per net.
        """
        if self._driver_sink_res_cache is not None:
            return self._driver_sink_res_cache

        result: Dict[str, float] = {}
        if not self.driver or not self.sinks:
            self._driver_sink_res_cache = result
            return result

        driver_node = self._find_best_node_name(self.driver) or self.driver

        for sink in self.sinks:
            sink_node = self._find_best_node_name(sink) or sink
            r = self._shortest_resistance(driver_node, sink_node)
            if r is not None:
                result[sink] = r
        self._driver_sink_res_cache = result
        return result


class SpefFile:
    def __init__(self, path: str) -> None:
        self.path = path
        self.name_map: Dict[str, str] = {}
        self.nets: Dict[str, NetRC] = {}

    # -------------------------- parsing helpers --------------------------

    def _resolve_name(self, tok: str) -> str:
        """Resolve a token via *NAME_MAP if it starts with '*'."""
        if tok.startswith("\"") and tok.endswith("\"") and len(tok) >= 2:
            return tok[1:-1]
        if tok.startswith("*") and tok in self.name_map:
            return self.name_map[tok]
        return tok

    def parse(self) -> None:
        with open(self.path, "r", encoding="utf-8", errors="ignore") as f:
            lines = f.readlines()

        i = 0
        n = len(lines)
        in_name_map = False
        current_net: Optional[NetRC] = None
        section: Optional[str] = None  # None, "CONN", "CAP", "RES"

        while i < n:
            raw = lines[i].strip()
            i += 1
            if not raw or raw.startswith("//"):
                continue

            # NAME_MAP section
            if raw.startswith("*NAME_MAP"):
                in_name_map = True
                continue
            if in_name_map:
                if raw.startswith("*"):
                    # *<id> <name>
                    parts = raw.split()
                    if len(parts) >= 2:
                        key = parts[0]
                        name = parts[1].strip("\"")
                        self.name_map[key] = name
                    continue
                else:
                    # End of name map when a non-* line appears
                    in_name_map = False

            # New net
            if raw.startswith("*D_NET"):
                parts = raw.split()
                if len(parts) < 3:
                    continue
                net_name = self._resolve_name(parts[1])
                try:
                    total_cap = float(parts[2])
                except ValueError:
                    # Some tools may add units, handle basic case like 1.23e-12
                    num = parts[2]
                    # strip potential trailing chars (e.g., pF, fF)
                    num = ''.join(ch for ch in num if (ch.isdigit() or ch in ".-+eE"))
                    total_cap = float(num) if num else 0.0
                current_net = NetRC(name=net_name, total_cap=total_cap)
                self.nets[net_name] = current_net
                section = None
                continue

            if current_net is None:
                continue

            if raw.startswith("*CONN"):
                section = "CONN"
                continue
            if raw.startswith("*CAP"):
                section = "CAP"
                continue
            if raw.startswith("*RES"):
                section = "RES"
                continue
            if raw.startswith("*END"):
                current_net = None
                section = None
                continue

            # Inside sections
            if section == "CONN":
                # Example: *I U1/Z O ; *P A I
                parts = raw.split()
                if len(parts) < 3:
                    continue
                # parts[0] = *I or *P, parts[1] = pin, parts[2] = direction
                pin_tok = parts[1]
                dir_tok = parts[2]
                pin_name = self._resolve_name(pin_tok)
                if dir_tok.upper().startswith("O") or dir_tok.upper().startswith("B"):
                    # Assume first O/B is driver
                    if current_net.driver is None:
                        current_net.driver = pin_name
                elif dir_tok.upper().startswith("I"):
                    current_net.sinks.append(pin_name)

            elif section == "RES":
                # Example: *1 node1 node2 R
                parts = raw.split()
                if len(parts) < 4:
                    continue
                node1 = self._resolve_name(parts[1])
                node2 = self._resolve_name(parts[2])
                try:
                    rval = float(parts[3])
                except ValueError:
                    num = parts[3]
                    num = ''.join(ch for ch in num if (ch.isdigit() or ch in ".-+eE"))
                    try:
                        rval = float(num) if num else 0.0
                    except ValueError:
                        continue

                g = current_net.res_graph
                g.setdefault(node1, []).append((node2, rval))
                g.setdefault(node2, []).append((node1, rval))

            # CAP section is not needed for this tool beyond total_cap in *D_NET


# ------------------------ correlation & statistics ------------------------


def pearson_corr(xs: Iterable[float], ys: Iterable[float]) -> Optional[float]:
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


@dataclass
class CapComparison:
    net: str
    c1: float
    c2: float


@dataclass
class ResComparison:
    net: str
    r1: float
    r2: float


def _aggregate(values: List[float], mode: str) -> Optional[float]:
    """Aggregate a list of resistances into a single value per net.

    mode:
      - 'max':   maximum R among sinks
      - 'avg':   average R among sinks
      - 'total': sum of R over sinks
    """
    if not values:
        return None
    if mode == "max":
        return max(values)
    if mode == "avg":
        return sum(values) / len(values)
    if mode == "total":
        return sum(values)
    return None


def compare_spef(s1: SpefFile, s2: SpefFile, r_agg: str) -> Tuple[List[CapComparison], List[ResComparison]]:
    common_nets = sorted(set(s1.nets.keys()) & set(s2.nets.keys()))
    print("common_nets are sorted")
    cap_rows: List[CapComparison] = []
    res_rows: List[ResComparison] = []
 
    for net_name in common_nets:
        n1 = s1.nets[net_name]
        n2 = s2.nets[net_name]
        cap_rows.append(CapComparison(net=net_name, c1=n1.total_cap, c2=n2.total_cap))
 
        dr1 = n1.driver_sink_resistances()
        dr2 = n2.driver_sink_resistances()
        common_sinks = sorted(set(dr1.keys()) & set(dr2.keys()))
        if not common_sinks:
            continue
 
        for s in common_sinks:
            val1 = dr1[s]
            val2 = dr2[s]
            res_rows.append(ResComparison(net=net_name, driver=n1.driver, load=s, r1=val1, r2=val2))
 
        # 计算偏差并找出最大的10个
        cap_rows_with_deviation = [(abs(row.c1 - row.c2), row) for row in cap_rows]
        res_rows_with_deviation = [(abs(row.r1 - row.r2), row) for row in res_rows]
        top_10_cap = [row for _, row in nlargest(10, cap_rows_with_deviation, key=lambda x: x[0])]
        top_10_res = [row for _, row in nlargest(10, res_rows_with_deviation, key=lambda x: x[0])]
 
        return cap_rows, res_rows, top_10_cap, top_10_res


def write_caps_csv(path: str, caps: List[CapComparison]) -> None:
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["net", "C_tool1", "C_tool2", "ratio_tool2_over_tool1", "delta_tool2_minus_tool1"])
        for row in caps:
            ratio = (row.c2 / row.c1) if row.c1 != 0 else "inf"
            delta = row.c2 - row.c1
            w.writerow([row.net, row.c1, row.c2, ratio, delta])


def write_res_csv(path: str, ress: List[ResComparison], r_agg: str) -> None:
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow([
            "net",
            f"R_{r_agg}_tool1",
            f"R_{r_agg}_tool2",
            "ratio_tool2_over_tool1",
            "delta_tool2_minus_tool1",
        ])
        for row in ress:
            ratio = (row.r2 / row.r1) if row.r1 != 0 else "inf"
            delta = row.r2 - row.r1
            w.writerow([row.net, row.r1, row.r2, ratio, delta])


def summarize_and_print(caps: List[CapComparison], ress: List[ResComparison], 
                        spef1: SpefFile, spef2: SpefFile, r_agg: str) -> None:
    print("=== SPEF RC Correlation Summary ===")
    print(f"Tool1 SPEF: {spef1.path}")
    print(f"Tool2 SPEF: {spef2.path}")
    print(f"Nets in tool1: {len(spef1.nets)}")
    print(f"Nets in tool2: {len(spef2.nets)}")
    print(f"Common nets:  {len({c.net for c in caps})}")

    xs_c = [c.c1 for c in caps]
    ys_c = [c.c2 for c in caps]
    corr_c = pearson_corr(xs_c, ys_c)
    if corr_c is not None:
        print(f"Total C correlation (Pearson, per-net): {corr_c:.6f} over {len(caps)} nets")
    else:
        print("Total C correlation: N/A (not enough data or zero variance)")

    xs_r = [r.r1 for r in ress]
    ys_r = [r.r2 for r in ress]
    corr_r = pearson_corr(xs_r, ys_r)
    if corr_r is not None:
        print(
            f"Driver->sink R correlation (Pearson, per-net, agg={r_agg}): "
            f"{corr_r:.6f} over {len(ress)} nets"
        )
    else:
        print("Driver->sink R correlation: N/A (not enough data or zero variance)")


# ----------------------------- GUI (Tkinter) -----------------------------


def launch_gui() -> None:
    """Launch an interactive GUI for multi-SPEF correlation analysis."""

    try:
        import tkinter as tk
        from tkinter import filedialog, messagebox, ttk, simpledialog
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
        from matplotlib.figure import Figure
    except Exception as exc:
        print("GUI mode requires tkinter and matplotlib to be installed.")
        print(f"Error importing GUI dependencies: {exc}")
        return

    class RcCorrApp:
        def __init__(self, root: "tk.Tk") -> None:
            self.root = root
            self.root.title("SPEF RC Correlation")

            # name -> SpefFile
            self.spefs: Dict[str, SpefFile] = {}

            # selection & filter state
            self.ref_var = tk.StringVar()
            self.fit_var = tk.StringVar()
            self.r_agg_var = tk.StringVar(value="max")

            self.min_fanout_var = tk.StringVar()
            self.max_fanout_var = tk.StringVar()
            self.min_c_var = tk.StringVar()
            self.max_c_var = tk.StringVar()
            self.min_r_var = tk.StringVar()
            self.max_r_var = tk.StringVar()

            self.points: List[Dict[str, float]] = []  # per-net data after filtering

            self._build_ui()

        # ----------------------------- UI building -----------------------------

        def _build_ui(self) -> None:
            from math import isnan  # noqa: F401  # reserved if needed later

            # SPEF files frame
            files_frame = ttk.LabelFrame(self.root, text="SPEF Files")
            files_frame.pack(fill="x", padx=5, pady=5)

            self.tree = ttk.Treeview(
                files_frame,
                columns=("name", "nets", "path"),
                show="headings",
                height=4,
            )
            self.tree.heading("name", text="Name")
            self.tree.heading("nets", text="#Nets")
            self.tree.heading("path", text="Path")
            self.tree.column("name", width=80, anchor="center")
            self.tree.column("nets", width=70, anchor="center")
            self.tree.column("path", width=400, anchor="w")
            self.tree.pack(side="left", fill="x", expand=True, padx=(5, 0), pady=5)

            btn_frame = ttk.Frame(files_frame)
            btn_frame.pack(side="right", fill="y", padx=5, pady=5)

            ttk.Button(btn_frame, text="Add SPEF", command=self._add_spef).pack(fill="x", pady=2)
            ttk.Button(btn_frame, text="Remove", command=self._remove_selected).pack(fill="x", pady=2)

            # Settings & filters frame
            opts_frame = ttk.LabelFrame(self.root, text="Settings & Filters")
            opts_frame.pack(fill="x", padx=5, pady=5)

            # Row 0: reference & fit selection
            ttk.Label(opts_frame, text="Reference (X axis):").grid(row=0, column=0, sticky="w", padx=5, pady=2)
            self.ref_combo = ttk.Combobox(opts_frame, textvariable=self.ref_var, state="readonly", width=15)
            self.ref_combo.grid(row=0, column=1, sticky="w", padx=5, pady=2)

            ttk.Label(opts_frame, text="Fit (Y axis):").grid(row=0, column=2, sticky="w", padx=5, pady=2)
            self.fit_combo = ttk.Combobox(opts_frame, textvariable=self.fit_var, state="readonly", width=15)
            self.fit_combo.grid(row=0, column=3, sticky="w", padx=5, pady=2)

            # Row 1: fanout range
            ttk.Label(opts_frame, text="Fanout range:").grid(row=1, column=0, sticky="w", padx=5, pady=2)
            ttk.Entry(opts_frame, textvariable=self.min_fanout_var, width=8).grid(row=1, column=1, sticky="w", padx=5, pady=2)
            ttk.Label(opts_frame, text="to").grid(row=1, column=2, sticky="w", padx=2, pady=2)
            ttk.Entry(opts_frame, textvariable=self.max_fanout_var, width=8).grid(row=1, column=3, sticky="w", padx=5, pady=2)

            # Row 2: cap range (reference)
            ttk.Label(opts_frame, text="Cap range (ref C):").grid(row=2, column=0, sticky="w", padx=5, pady=2)
            ttk.Entry(opts_frame, textvariable=self.min_c_var, width=10).grid(row=2, column=1, sticky="w", padx=5, pady=2)
            ttk.Label(opts_frame, text="to").grid(row=2, column=2, sticky="w", padx=2, pady=2)
            ttk.Entry(opts_frame, textvariable=self.max_c_var, width=10).grid(row=2, column=3, sticky="w", padx=5, pady=2)

            # Row 3: R range (reference, aggregated)
            ttk.Label(opts_frame, text="R range (ref, agg):").grid(row=3, column=0, sticky="w", padx=5, pady=2)
            ttk.Entry(opts_frame, textvariable=self.min_r_var, width=10).grid(row=3, column=1, sticky="w", padx=5, pady=2)
            ttk.Label(opts_frame, text="to").grid(row=3, column=2, sticky="w", padx=2, pady=2)
            ttk.Entry(opts_frame, textvariable=self.max_r_var, width=10).grid(row=3, column=3, sticky="w", padx=5, pady=2)

            # Row 4: R aggregation, run button, correlation label
            ttk.Label(opts_frame, text="R aggregation:").grid(row=4, column=0, sticky="w", padx=5, pady=2)
            r_agg_combo = ttk.Combobox(
                opts_frame,
                textvariable=self.r_agg_var,
                values=["max", "avg", "total"],
                state="readonly",
                width=8,
            )
            r_agg_combo.grid(row=4, column=1, sticky="w", padx=5, pady=2)

            ttk.Button(opts_frame, text="Run Analysis", command=self._run_analysis).grid(
                row=4,
                column=2,
                sticky="w",
                padx=5,
                pady=2,
            )

            self.corr_label = ttk.Label(opts_frame, text="")
            self.corr_label.grid(row=4, column=3, sticky="w", padx=5, pady=2)

            # Plot frame
            plot_frame = ttk.LabelFrame(self.root, text="Correlation Plot")
            plot_frame.pack(fill="both", expand=True, padx=5, pady=5)

            self.fig = Figure(figsize=(7, 5))
            self.ax_c = self.fig.add_subplot(2, 1, 1)
            self.ax_r = self.fig.add_subplot(2, 1, 2)

            self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
            canvas_widget = self.canvas.get_tk_widget()
            canvas_widget.pack(fill="both", expand=True)

            self.annot_c = self._init_annot(self.ax_c)
            self.annot_r = self._init_annot(self.ax_r)

            self.fig.tight_layout()
            self.canvas.draw()
            self.canvas.mpl_connect("motion_notify_event", self._on_motion)

        # ----------------------------- SPEF loading -----------------------------

        def _add_spef(self) -> None:
            path = filedialog.askopenfilename(
                title="Select SPEF file",
                filetypes=[("SPEF files", "*.spef;*.SPEF"), ("All files", "*.*")],
            )
            if not path:
                return

            name = simpledialog.askstring("SPEF Name", "Enter a unique name for this SPEF:")
            if not name:
                return
            name = name.strip()
            if not name or name in self.spefs:
                messagebox.showerror("Invalid name", "Name is empty or already in use.")
                return

            try:
                self.root.config(cursor="watch")
                self.root.update_idletasks()
                spef = SpefFile(path)
                spef.parse()
            except Exception as exc:
                messagebox.showerror("Parse error", f"Failed to parse SPEF file:\n{exc}")
                return
            finally:
                self.root.config(cursor="")
                self.root.update_idletasks()

            self.spefs[name] = spef
            self.tree.insert("", "end", iid=name, values=(name, len(spef.nets), path))
            self._refresh_spef_choices()

        def _remove_selected(self) -> None:
            for item in self.tree.selection():
                self.tree.delete(item)
                self.spefs.pop(item, None)
            self._refresh_spef_choices()

        def _refresh_spef_choices(self) -> None:
            names = sorted(self.spefs.keys())
            self.ref_combo["values"] = names
            self.fit_combo["values"] = names

            # Try to keep selections valid
            if self.ref_var.get() not in names:
                self.ref_var.set(names[0] if names else "")
            if self.fit_var.get() not in names:
                self.fit_var.set(names[1] if len(names) > 1 else (names[0] if len(names) == 1 else ""))

        # -------------------------- filtering helpers --------------------------

        def _parse_filters(self) -> Optional[Dict[str, Optional[float]]]:
            def to_int(s: str) -> Optional[int]:
                s = s.strip()
                if not s:
                    return None
                return int(s)

            def to_float(s: str) -> Optional[float]:
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
            except ValueError as exc:
                messagebox.showerror("Invalid filter", f"Filter values must be numeric.\n{exc}")
                return None

        def _passes_filters(self, p: Dict[str, float], flt: Dict[str, Optional[float]]) -> bool:
            mf = flt["min_fanout"]
            xf = flt["max_fanout"]
            if mf is not None and p["fanout"] < mf:
                return False
            if xf is not None and p["fanout"] > xf:
                return False

            mc = flt["min_c"]
            xc = flt["max_c"]
            if mc is not None and p["c_ref"] < mc:
                return False
            if xc is not None and p["c_ref"] > xc:
                return False

            mr = flt["min_r"]
            xr = flt["max_r"]
            if mr is not None and p["r_ref"] < mr:
                return False
            if xr is not None and p["r_ref"] > xr:
                return False

            return True

        # ------------------------- analysis & plotting -------------------------

        def _run_analysis(self) -> None:
            if len(self.spefs) < 2:
                messagebox.showwarning("Need SPEFs", "Please load at least two SPEF files.")
                return

            ref_name = self.ref_var.get()
            fit_name = self.fit_var.get()
            if not ref_name or not fit_name:
                messagebox.showwarning("Select SPEFs", "Please select reference and fit SPEFs.")
                return
            if ref_name == fit_name:
                messagebox.showwarning("Select different SPEFs", "Reference and fit SPEFs must be different.")
                return

            ref = self.spefs.get(ref_name)
            fit = self.spefs.get(fit_name)
            if ref is None or fit is None:
                messagebox.showerror("Invalid selection", "Selected SPEF not found.")
                return

            flt = self._parse_filters()
            if flt is None:
                return

            r_agg = self.r_agg_var.get()
            caps, ress = compare_spef(ref, fit, r_agg)

            cap_map = {c.net: c for c in caps}
            res_map = {r.net: r for r in ress}
            nets = sorted(set(cap_map.keys()) & set(res_map.keys()))

            points: List[Dict[str, float]] = []
            for net in nets:
                c = cap_map[net]
                r = res_map[net]
                fanout = len(ref.nets[net].sinks) if net in ref.nets else 0
                p = {
                    "net": net,
                    "fanout": float(fanout),
                    "c_ref": c.c1,
                    "c_fit": c.c2,
                    "r_ref": r.r1,
                    "r_fit": r.r2,
                }
                if self._passes_filters(p, flt):
                    points.append(p)

            self.points = points

            if not points:
                self.corr_label.config(text="No nets after filtering")
                self._clear_axes()
                return

            xs_c = [p["c_ref"] for p in points]
            ys_c = [p["c_fit"] for p in points]
            xs_r = [p["r_ref"] for p in points]
            ys_r = [p["r_fit"] for p in points]

            corr_c = pearson_corr(xs_c, ys_c)
            corr_r = pearson_corr(xs_r, ys_r)

            c_text = "C corr: N/A"
            if corr_c is not None:
                c_text = f"C corr ({len(points)} nets): {corr_c:.4f}"
            r_text = "R corr: N/A"
            if corr_r is not None:
                r_text = f"R corr ({len(points)} nets, {r_agg}): {corr_r:.4f}"
            self.corr_label.config(text=f"{c_text} | {r_text}")

            self._update_plot(ref_name, fit_name, corr_c, corr_r)

        def _clear_axes(self) -> None:
            self.ax_c.clear()
            self.ax_r.clear()
            self.ax_c.set_title("No data")
            self.ax_r.set_title("")
            self.canvas.draw()

        def _update_plot(self, ref_name: str, fit_name: str,
                          corr_c: Optional[float], corr_r: Optional[float]) -> None:
            self.ax_c.clear()
            self.ax_r.clear()

            # Capacitance plot
            xs_c = [p["c_ref"] for p in self.points]
            ys_c = [p["c_fit"] for p in self.points]
            if xs_c and ys_c:
                min_c = min(xs_c + ys_c)
                max_c = max(xs_c + ys_c)
                span_c = max_c - min_c or 1.0
                pad_c = 0.05 * span_c
                vmin_c = min_c - pad_c
                vmax_c = max_c + pad_c

                colors_c = ["tab:red" if y > x else "tab:blue" for x, y in zip(xs_c, ys_c)]
                self.ax_c.plot([vmin_c, vmax_c], [vmin_c, vmax_c], "k--", linewidth=1.0)
                self.ax_c.scatter(xs_c, ys_c, c=colors_c, s=20, alpha=0.8)
                self.ax_c.set_xlim(vmin_c, vmax_c)
                self.ax_c.set_ylim(vmin_c, vmax_c)

            title_c = f"Total C: {ref_name} (X) vs {fit_name} (Y)"
            if corr_c is not None:
                title_c += f"  (corr={corr_c:.4f})"
            self.ax_c.set_title(title_c)
            self.ax_c.set_xlabel(f"{ref_name} C")
            self.ax_c.set_ylabel(f"{fit_name} C")

            # Resistance plot
            xs_r = [p["r_ref"] for p in self.points]
            ys_r = [p["r_fit"] for p in self.points]
            if xs_r and ys_r:
                min_r = min(xs_r + ys_r)
                max_r = max(xs_r + ys_r)
                span_r = max_r - min_r or 1.0
                pad_r = 0.05 * span_r
                vmin_r = min_r - pad_r
                vmax_r = max_r + pad_r

                colors_r = ["tab:red" if y > x else "tab:blue" for x, y in zip(xs_r, ys_r)]
                self.ax_r.plot([vmin_r, vmax_r], [vmin_r, vmax_r], "k--", linewidth=1.0)
                self.ax_r.scatter(xs_r, ys_r, c=colors_r, s=20, alpha=0.8)
                self.ax_r.set_xlim(vmin_r, vmax_r)
                self.ax_r.set_ylim(vmin_r, vmax_r)

            title_r = f"Driver->sink R ({self.r_agg_var.get()}): {ref_name} (X) vs {fit_name} (Y)"
            if corr_r is not None:
                title_r += f"  (corr={corr_r:.4f})"
            self.ax_r.set_title(title_r)
            self.ax_r.set_xlabel(f"{ref_name} R")
            self.ax_r.set_ylabel(f"{fit_name} R")

            self.annot_c = self._init_annot(self.ax_c)
            self.annot_r = self._init_annot(self.ax_r)

            self.fig.tight_layout()
            self.canvas.draw()

        # ------------------------------ hover logic -----------------------------

        def _init_annot(self, ax):
            annot = ax.annotate(
                "",
                xy=(0, 0),
                xytext=(10, 10),
                textcoords="offset points",
                bbox=dict(boxstyle="round", fc="w"),
                arrowprops=dict(arrowstyle="->"),
            )
            annot.set_visible(False)
            return annot

        def _on_motion(self, event) -> None:
            if not self.points:
                return

            ax = event.inaxes
            if ax not in (self.ax_c, self.ax_r):
                # hide annotations when leaving axes
                changed = False
                if self.annot_c.get_visible():
                    self.annot_c.set_visible(False)
                    changed = True
                if self.annot_r.get_visible():
                    self.annot_r.set_visible(False)
                    changed = True
                if changed:
                    self.canvas.draw_idle()
                return

            if event.xdata is None or event.ydata is None:
                return

            if ax is self.ax_c:
                xs = [p["c_ref"] for p in self.points]
                ys = [p["c_fit"] for p in self.points]
                annot = self.annot_c
            else:
                xs = [p["r_ref"] for p in self.points]
                ys = [p["r_fit"] for p in self.points]
                annot = self.annot_r

            x0, y0 = float(event.xdata), float(event.ydata)
            if not xs or not ys:
                return

            x_min, x_max = min(xs), max(xs)
            y_min, y_max = min(ys), max(ys)
            x_span = x_max - x_min or 1.0
            y_span = y_max - y_min or 1.0

            best_i = None
            best_d2 = None
            for i, (x, y) in enumerate(zip(xs, ys)):
                dx = (x - x0) / x_span
                dy = (y - y0) / y_span
                d2 = dx * dx + dy * dy
                if best_d2 is None or d2 < best_d2:
                    best_d2 = d2
                    best_i = i

            if best_i is None:
                return

            # Threshold in normalized space; tweak if needed
            if best_d2 is not None and best_d2 < 0.005:
                p = self.points[best_i]
                annot.xy = (xs[best_i], ys[best_i])
                text = (
                    f"net: {p['net']}\n"
                    f"C_ref={p['c_ref']:.3g}, C_fit={p['c_fit']:.3g}\n"
                    f"R_ref={p['r_ref']:.3g}, R_fit={p['r_fit']:.3g}"
                )
                annot.set_text(text)
                annot.get_bbox_patch().set_facecolor("#ffffcc")
                annot.set_visible(True)
                self.canvas.draw_idle()
            else:
                if annot.get_visible():
                    annot.set_visible(False)
                    self.canvas.draw_idle()

    root = tk.Tk()
    RcCorrApp(root)
    root.mainloop()


# ---------------------------- CLI entry point ----------------------------


def main() -> None:
    parser = argparse.ArgumentParser(description="SPEF RC correlation between two extraction tools")
    parser.add_argument("spef1", nargs="?", help="First SPEF file (tool 1)")
    parser.add_argument("spef2", nargs="?", help="Second SPEF file (tool 2)")
    parser.add_argument("--csv-prefix", help="If set, write CSVs with this prefix (e.g., prefix_caps.csv, prefix_res.csv)")
    parser.add_argument(
        "--r-agg",
        choices=["max", "avg", "total"],
        default="max",
        help="Aggregation mode for per-net driver-to-sink R: max, avg, or total over sinks",
    )
    parser.add_argument(
        "--gui",
        action="store_true",
        help="Launch GUI for interactive multi-SPEF correlation analysis",
    )

    args = parser.parse_args()

    if args.gui or (args.spef1 is None and args.spef2 is None):
        launch_gui()
        return

    if args.spef1 is None or args.spef2 is None:
        parser.error("spef1 and spef2 are required in CLI mode (or use --gui).")

    s1 = SpefFile(args.spef1)
    s2 = SpefFile(args.spef2)

    print(f"Parsing {args.spef1} ...")
    s1.parse()
    print(f"Parsing {args.spef2} ...")
    s2.parse()

    caps, ress = compare_spef(s1, s2, args.r_agg)
    summarize_and_print(caps, ress, s1, s2, args.r_agg)

    if args.csv_prefix:
        caps_path = f"{args.csv_prefix}_caps.csv"
        res_path = f"{args.csv_prefix}_res_{args.r_agg}.csv"
        write_caps_csv(caps_path, caps)
        write_res_csv(res_path, ress, args.r_agg)
        print("\nCSV written:")
        print(f"  {caps_path}")
        print(f"  {res_path}")


if __name__ == "__main__":
    main()
