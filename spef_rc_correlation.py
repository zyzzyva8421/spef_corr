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
import os
import random
import shutil
from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Optional, Iterable, Set
import heapq

import re
from collections import deque
from heapq import nlargest
import multiprocessing
from functools import partial

@dataclass
class NetRC:
    name: str
    total_cap: float
    driver: Optional[str] = None
    sinks: List[str] = field(default_factory=list)
    # resistance graph: node -> list of (neighbor, R)
    res_graph: Dict[str, List[Tuple[str, float]]] = field(default_factory=dict)

    node_levels: Dict[str, int] = field(default_factory=dict)
    node_layers: Dict[str, str] = field(default_factory=dict)
    # segment capacitances: list of (node1, node2, cap_value)
    # node2 is None for grounded caps, otherwise both nodes present for mutual caps
    segment_caps: List[Tuple[str, Optional[str], float]] = field(default_factory=list)
    # cache for driver->sink resistance
    _driver_sink_res_cache: Optional[Dict[str, float]] = field(default=None, init=False, repr=False)
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

        # print(f"Computing shortest resistance from {src} to {dst} in net {self.name}...")
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
        self.layer_map: Dict[str, str] = {}
        self.port_map: Dict[str, Dict] = {}
        self.level_to_layer: Dict[str, str] = {}
        self.nets: Dict[str, NetRC] = {}
        self.t_unit = 'NS'
        self.c_unit = 'PF'
        self.r_unit = 'OHM'
        self.l_unit = 'HENRY'

    # -------------------------- parsing helpers --------------------------
    @staticmethod
    def _parse_level_from_comment(comment: str) -> Tuple[Optional[int], Optional[int], Optional[int]]:
        lvl = from_lvl = to_lvl = None
        for tok in comment.split():
            if tok.startswith("$lvl="):
                try:
                    lvl = int(tok[len("$lvl="):])
                except ValueError:
                    pass
            elif tok.startswith("$from_lvl="):
                try:
                    from_lvl = int(tok[len("$from_lvl="):])
                except ValueError:
                    pass
            elif tok.startswith("$to_lvl="):
                try:
                    to_lvl = int(tok[len("$to_lvl="):])
                except ValueError:
                    pass
        return lvl, from_lvl, to_lvl

    def _assign_node_level(self, net: NetRC, node_name: str, level: Optional[int]) -> None:
        if level is None:
            return
        if node_name in net.node_levels:
            return
        net.node_levels[node_name] = level
        layer_name = self.level_to_layer.get(level)
        if layer_name is not None and node_name not in net.node_layers:
            net.node_layers[node_name] = layer_name


    def _resolve_name(self, tok: str) -> str:
        """Resolve a token via *NAME_MAP if it starts with '*'."""
        if tok.startswith("\"") and tok.endswith("\"") and len(tok) >= 2:
            return tok[1:-1]
        pin = ''
        if ":" in tok:
            parts = tok.split(':')
            pin = parts[1]
            tok = parts[0]
        if tok.startswith("*") and tok in self.name_map:
            result = self.name_map[tok] + f':{pin}' if pin else self.name_map[tok]
        else:
            result = tok
        # Unescape \[ → [ and \] → ] that some SPEF writers emit for bus nets
        if '\\' in result:
            result = result.replace('\\[', '[').replace('\\]', ']')
        return result

    def parse(self) -> None:
        # ---- bind frequently accessed attributes to locals ----
        # Python attribute lookup (self.x) is slower than reading a local variable.
        name_map     = self.name_map
        nets_dict    = self.nets
        layer_map    = self.layer_map
        port_map     = self.port_map
        path         = self.path
        assign_level = self._assign_node_level
        parse_level  = self._parse_level_from_comment

        # Pre-compile the float fallback regex once per parse call (not per line)
        _re_nonnum = re.compile(r'[^\d.\-+eE]')

        def _pf(s: str) -> float:
            """Float parse with graceful fallback for strings with unit suffixes."""
            try:
                return float(s)
            except ValueError:
                c = _re_nonnum.sub('', s)
                return float(c) if c else 0.0

        # Inline _resolve_name as a closure to avoid per-call method dispatch overhead
        def _resolve(tok: str) -> str:
            if not tok:
                return tok
            c = tok[0]
            if c == '"':
                return tok[1:-1] if tok[-1] == '"' else tok[1:]
            if ':' in tok:
                idx  = tok.index(':')
                base = tok[:idx]
                pin  = tok[idx + 1:]
                if base and base[0] == '*' and base in name_map:
                    r = name_map[base]
                    result = f"{r}:{pin}" if pin else r
                else:
                    result = base   # no name-map match: return base, dropping pin suffix
            elif c == '*':
                result = name_map.get(tok, tok)
            else:
                result = tok
            # Unescape \[ → [ and \] → ] that some SPEF writers emit for bus nets
            if '\\' in result:
                result = result.replace('\\[', '[').replace('\\]', ']')
            return result

        # Integer section constants — int comparison is faster than str comparison
        SEC_NONE = 0
        SEC_CONN = 1
        SEC_CAP  = 2
        SEC_RES  = 3

        in_name_map  = False
        in_layer_map = False
        in_port_map  = False
        current_net: Optional[NetRC] = None
        section      = SEC_NONE
        net_count    = 0
        r_is_kohm    = False   # set True when *R_UNIT KOHM is seen

        with open(path, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                # ---- split code / inline comment using find() instead of split() ----
                ci = line.find("//")
                if ci >= 0:
                    code_part    = line[:ci]
                    comment_part = line[ci + 2:].strip()
                else:
                    code_part    = line
                    comment_part = ""
                raw = code_part.strip()
                if not raw:
                    if comment_part:
                        body = comment_part.lstrip()
                        if in_layer_map:
                            if not body or body[0] != '*':
                                in_layer_map = False
                            else:
                                lp = body.split(None, 2)
                                if len(lp) >= 2:
                                    layer_map[lp[0]] = lp[1]
                        elif body.startswith("*LAYER_MAP"):
                            in_layer_map = True
                    continue

                c1 = raw[1] if len(raw) > 1 else '\x00'

                # ============================================================
                # HOT PATH — inside a net (handles the vast majority of lines)
                # ============================================================
                if current_net is not None:
                    if section == SEC_RES:
                        # Typical line: *idx node1 node2 R_value [...]
                        # split(None, 4): bounded split avoids splitting the tail
                        parts = raw.split(None, 4)
                        if len(parts) >= 4:
                            node1 = _resolve(parts[1])
                            node2 = _resolve(parts[2])
                            rval  = _pf(parts[3])
                            if r_is_kohm:
                                rval *= 1000.0
                            g = current_net.res_graph
                            g.setdefault(node1, []).append((node2, rval))
                            g.setdefault(node2, []).append((node1, rval))
                            if comment_part:
                                lvl, from_lvl, to_lvl = parse_level(comment_part)
                                if "via" not in comment_part:
                                    assign_level(current_net, node1, lvl)
                                    assign_level(current_net, node2, lvl)
                                else:
                                    assign_level(current_net, node1, from_lvl or lvl)
                                    assign_level(current_net, node2, to_lvl or lvl)
                        else:
                            # Single-token line → must be a section header
                            if   c1 == 'E' and raw[2:4] == 'ND':                     current_net = None; section = SEC_NONE
                            elif c1 == 'C' and raw[2:4] == 'AP': section = SEC_CAP
                            elif c1 == 'C' and raw[2:5] == 'ONN':                     section = SEC_CONN
                            elif c1 == 'R' and raw[2:4] == 'ES': 
                                #print(f"Warning: unexpected line in RES section of net {current_net.name} (missing END header): '{raw}'")
                                section = SEC_RES
                        continue

                    if section == SEC_CAP:
                        # Parse CAP entries: 
                        # - Format 1: idx node1 cap_value (node to ground)
                        # - Format 2: idx node1 node2 cap_value (mutual cap)
                        parts = raw.split(None, 4)
                        if len(parts) >= 3:
                            # Check if this is a numeric index (CAP entry) or section header
                            try:
                                idx = int(parts[0])
                                # This is a CAP entry
                                if len(parts) == 3:
                                    # Node to ground: idx node1 cap_value
                                    node1 = _resolve(parts[1])
                                    cap_val = _pf(parts[2])
                                    current_net.segment_caps.append((node1, None, cap_val))
                                elif len(parts) >= 4:
                                    # Mutual cap: idx node1 node2 cap_value
                                    node1 = _resolve(parts[1])
                                    node2 = _resolve(parts[2])
                                    cap_val = _pf(parts[3])
                                    current_net.segment_caps.append((node1, node2, cap_val))
                            except ValueError:
                                # Not a numeric index, must be a section header
                                if   c1 == 'C' and raw[2:4] == 'AP': section = SEC_CAP
                                elif c1 == 'C' and raw[2:5] == 'ONN': section = SEC_CONN
                                elif c1 == 'R' and raw[2:4] == 'ES': 
                                    #print(f"Warning: unexpected line in CAP section of net {current_net.name} (missing END header?): '{raw}'")
                                    section = SEC_RES
                                elif c1 == 'E' and raw[2:4] == 'ND': current_net = None; section = SEC_NONE
                        else:
                            # Single or double token line (section header)
                            if   c1 == 'E' and raw[2:4] == 'ND':                     current_net = None; section = SEC_NONE
                            elif c1 == 'C' and raw[2:4] == 'AP': section = SEC_CAP
                            elif c1 == 'C' and raw[2:5] == 'ONN':                     section = SEC_CONN
                            elif c1 == 'R' and raw[2:4] == 'ES': section = SEC_RES
                        continue

                    if section == SEC_CONN:
                        # Typical line: *I|*P pin direction [...]
                        # split(None, 3): we only need the first three tokens
                        parts = raw.split(None, 3)
                        if len(parts) >= 3:
                            pin_name = _resolve(parts[1])
                            # Use first-char set membership instead of upper().startswith()
                            d0 = parts[2][0]
                            if d0 in 'OoBb':
                                if current_net.driver is None:
                                    current_net.driver = pin_name
                            elif d0 in 'Ii':
                                current_net.sinks.append(pin_name)
                            if comment_part:
                                lvl, from_lvl, to_lvl = parse_level(comment_part)
                                assign_level(current_net, pin_name, lvl)
                        else:
                            if   c1 == 'C' and raw[2:4] == 'AP': section = SEC_CAP
                            elif c1 == 'C' and raw[2:5] == 'ONN': section = SEC_CONN
                            elif c1 == 'R' and raw[2:4] == 'ES': 
                                #print(f"Warning: unexpected line in CONN section of net {current_net.name} (missing END header): '{raw}'")
                                section = SEC_RES
                            elif c1 == 'E' and raw[2:4] == 'ND': current_net = None; section = SEC_NONE
                        continue

                    # section == SEC_NONE or SEC_CAP: scan for next section header
                    if   c1 == 'C' and raw[2:5] == 'ONN': section = SEC_CONN
                    elif c1 == 'C' and raw[2:4] == 'AP':  section = SEC_CAP
                    elif c1 == 'R' and raw[2:4] == 'ES': 
                        #print(f"Warning: unexpected line in net {current_net.name} outside of CONN/RES sections (missing CONN header?): '{raw}'")
                        section = SEC_RES
                    elif c1 == 'E' and raw[2:4] == 'ND':  current_net = None; section = SEC_NONE
                    continue

                # ============================================================
                # LOWER-FREQUENCY PATH — outside a net (headers, *D_NET)
                # ============================================================

                # *D_NET — most frequent outside-net line once headers are done
                if c1 == 'D' and raw.startswith("*D_NET"):
                    parts = raw.split(None, 3)
                    if len(parts) >= 3:
                        in_port_map = False
                        net_name    = _resolve(parts[1])
                        total_cap   = _pf(parts[2])
                        current_net = NetRC(name=net_name, total_cap=total_cap)
                        nets_dict[net_name] = current_net
                        section   = SEC_NONE
                        net_count += 1
                        if net_count % 5000 == 0:
                            print(f"[{path}] parsed {net_count} nets...")
                    continue

                # *NAME_MAP entries
                if in_name_map:
                    if c1 == 'P' and raw.startswith("*PORTS"):
                        in_name_map = False
                        in_port_map = True
                    else:
                        parts = raw.split(None, 2)
                        if len(parts) >= 2:
                            name_map[parts[0]] = parts[1].strip('"')
                    continue

                # *PORTS entries
                if in_port_map:
                    parts = raw.split(None, 5)
                    if len(parts) >= 5:
                        port_map[parts[0]] = {'I/O': parts[1], 'x': parts[3], 'y': parts[4]}
                    continue

                # File-level headers (encountered only a handful of times)
                if   c1 == 'N' and raw.startswith("*NAME_MAP"): in_name_map = True
                elif c1 == 'P' and raw.startswith("*PORTS"):    in_port_map = True
                elif '_UNIT 1 ' in raw:
                    # *R_UNIT 1 OHM / *C_UNIT 1 PF / *L_UNIT 1 HENRY / *T_UNIT 1 NS
                    # Avoid regex: split is enough since format is fixed
                    uparts = raw.split(None, 3)
                    if len(uparts) >= 3:
                        obj  = c1          # 'R', 'C', 'L', or 'T'
                        unit = uparts[2]
                        if obj == 'R':
                            self.r_unit = unit
                            r_is_kohm   = (unit == 'KOHM')
                        elif obj == 'C':
                            self.c_unit = unit
                        elif obj == 'L':
                            self.l_unit = unit
                        elif obj == 'T':
                            self.t_unit = unit

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
    driver: str
    load: str
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

def init_worker(s1_global, s2_global):
    global s1, s2
    s1 = s1_global
    s2 = s2_global

def process_net(net_name):
    """处理单个网络的对比"""
    n1 = s1.nets[net_name]
    n2 = s2.nets[net_name]
    
    # cap_rows 数据
    cap_row = CapComparison(net=net_name, c1=n1.total_cap, c2=n2.total_cap)
    
    # res_rows 数据
    res_rows = []
    dr1 = n1.driver_sink_resistances()
    dr2 = n2.driver_sink_resistances()
    common_sinks = sorted(set(dr1.keys()) & set(dr2.keys()))
    
    for s in common_sinks:
        res_rows.append(ResComparison(
            net=net_name, 
            driver=n1.driver, 
            load=s, 
            r1=dr1[s], 
            r2=dr2[s]
        ))
    
    return cap_row, res_rows

def process_net_batch(net_names_batch):
    """批量处理多个网络，减少对象访问开销"""
    results = []
    for net_name in net_names_batch:
        n1 = s1.nets[net_name]
        n2 = s2.nets[net_name]
        
        cap_row = CapComparison(net=net_name, c1=n1.total_cap, c2=n2.total_cap)
        
        dr1 = n1.driver_sink_resistances()
        dr2 = n2.driver_sink_resistances()
        common_sinks = sorted(set(dr1.keys()) & set(dr2.keys()))
        # print(f"Processing net: {net_name}, common sinks: {len(common_sinks)}")
        res_rows = [
            ResComparison(net=net_name, driver=n1.driver, load=s, r1=dr1[s], r2=dr2[s])
            for s in common_sinks
        ]
        
        results.append((cap_row, res_rows))
    
    return results

def compare_spef1(s1: SpefFile, s2: SpefFile, r_agg: str) -> Tuple[List[CapComparison], List[ResComparison]]:
    common_nets = sorted(set(s1.nets.keys()) & set(s2.nets.keys()))
    print("common_nets are sorted")
    cap_rows: List[CapComparison] = []
    res_rows: List[ResComparison] = []
    batch_size = 16
    batches = [common_nets[i:i+batch_size] for i in range(0, len(common_nets), batch_size)]
    with multiprocessing.Pool(
        # processes=num_processes,
        initializer=init_worker,
        initargs=(s1, s2)
    ) as pool:
        batch_results = pool.map(process_net_batch, batches)
    
    # 展平结果
    cap_rows = []
    res_rows = []
    for batch in batch_results:
        for cap_row, res_row_list in batch:
            cap_rows.append(cap_row)
            res_rows.extend(res_row_list)
    print("start to find top 10")
    print(f"total cap rows: {len(cap_rows)}, total res rows: {len(res_rows)}")
    # 计算偏差并找出最大的10个
    cap_rows_with_deviation = [(abs(row.c1 - row.c2), row) for row in cap_rows]
    res_rows_with_deviation = [(abs(row.r1 - row.r2), row) for row in res_rows]
    
    top_10_cap = [row for _, row in nlargest(10, cap_rows_with_deviation, key=lambda x: x[0])]
    top_10_res = [row for _, row in nlargest(10, res_rows_with_deviation, key=lambda x: x[0])]

    return cap_rows, res_rows, top_10_cap, top_10_res

def compare_spef(s1: SpefFile, s2: SpefFile, r_agg: str) -> Tuple[List[CapComparison], List[ResComparison]]:
    common_nets = sorted(set(s1.nets.keys()) & set(s2.nets.keys()))
    print("common_nets are sorted")
    cap_rows: List[CapComparison] = []
    res_rows: List[ResComparison] = []

    i = 0
    with open("net_cap.data", 'w', encoding='utf-8') as fc, \
         open("net_res.data", 'w', encoding='utf-8') as fr:
        for net_name in common_nets:
            n1 = s1.nets[net_name]
            n2 = s2.nets[net_name]
            cap_rows.append(CapComparison(net=net_name, c1=n1.total_cap, c2=n2.total_cap))
            print(f"{net_name} {n1.total_cap} {n2.total_cap}", file=fc)

            dr1 = n1.driver_sink_resistances()
            dr2 = n2.driver_sink_resistances()
            common_sinks = sorted(set(dr1.keys()) & set(dr2.keys()))
            if not common_sinks:
                continue

            for s in common_sinks:
                val1 = dr1[s]
                val2 = dr2[s]
                res_rows.append(ResComparison(net=net_name, driver=n1.driver, load=s, r1=val1, r2=val2))
                print(f"{net_name} {n1.driver} {s} {val1} {val2}", file=fr)
            i += 1
            if i % 1000 == 0:
                print(f"{i} nets compared")

    print("start to find top 10")
    # 计算偏差并找出最大的10个
    cap_rows_with_deviation = [(abs(row.c1 - row.c2), row) for row in cap_rows]
    res_rows_with_deviation = [(abs(row.r1 - row.r2), row) for row in res_rows]
    
    top_10_cap = [row for _, row in nlargest(10, cap_rows_with_deviation, key=lambda x: x[0])]
    top_10_res = [row for _, row in nlargest(10, res_rows_with_deviation, key=lambda x: x[0])]

    return cap_rows, res_rows, top_10_cap, top_10_res

def parse_net_cap_data(path: str) -> List[CapComparison]:
    """Parse a net_cap.data file into a list of CapComparison objects.

    File format (whitespace-separated, one entry per line):
        net_name  total_c_spef1  total_c_spef2

    Lines starting with '#' and blank lines are ignored.
    """
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
            net_name = parts[0]
            try:
                c1 = float(parts[1])
                c2 = float(parts[2])
            except ValueError:
                print(f"[warn] {path}:{lineno}: non-numeric capacitance value, skipping")
                continue
            caps.append(CapComparison(net=net_name, c1=c1, c2=c2))
    return caps


def parse_net_res_data(path: str) -> List[ResComparison]:
    """Parse a net_res.data file into a list of ResComparison objects.

    File format (whitespace-separated, one entry per line):
        net_name  driver_pin  sink_pin  r_spef1  r_spef2

    Lines starting with '#' and blank lines are ignored.
    """
    ress: List[ResComparison] = []
    with open(path, 'r', encoding='utf-8') as f:
        for lineno, line in enumerate(f, 1):
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) < 5:
                print(f"[warn] {path}:{lineno}: expected 5 fields, got {len(parts)}, skipping")
                continue
            net_name = parts[0]
            driver = parts[1]
            sink = parts[2]
            try:
                r1 = float(parts[3])
                r2 = float(parts[4])
            except ValueError:
                print(f"[warn] {path}:{lineno}: non-numeric resistance value, skipping")
                continue
            ress.append(ResComparison(net=net_name, driver=driver, load=sink, r1=r1, r2=r2))
    return ress


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


# -------------------- backmark: apply new RC values to SPEF --------------------

def _parse_backmark_cap_data(path: str) -> Dict[str, float]:
    """Parse net_cap.data → {net_identifier: new_total_cap}.

    First column: net_id (*NNN) or net_name.
    Third column: new total cap value.
    """
    cap_map: Dict[str, float] = {}
    with open(path, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) < 3:
                continue
            net_key = parts[0]          # *id or net_name
            try:
                new_cap = float(parts[2])
            except ValueError:
                continue
            cap_map[net_key] = new_cap
    return cap_map


def _parse_backmark_res_data(path: str) -> Dict[str, Dict[str, float]]:
    """Parse net_res.data → {net_name: {sink_pin: new_driver_to_sink_R}}.

    Format: net_name driver sink r_old r_new
    Last column (r_new) is the target value.
    Multiple rows per net (one per sink).
    """
    res_map: Dict[str, Dict[str, float]] = {}  # net -> {sink -> new_R}
    with open(path, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) < 5:
                continue
            net_key = parts[0]
            sink = parts[2]
            try:
                new_r = float(parts[4])  # last column
            except ValueError:
                continue
            res_map.setdefault(net_key, {})[sink] = new_r
    return res_map


def _resolve_spef_token(tok: str, name_map: Dict[str, str]) -> str:
    """Resolve a raw SPEF *RES/*CAP node token to its graph node name.

    Mirrors the inline ``_resolve`` closure used by SpefFile.parse so that
    backmark_spef can look up the per-segment scale factor by the same key
    that the resistance graph was built with.

    Args:
        tok:      A raw SPEF node token such as ``*3``, ``*3:1``, or a bare
                  node name.  Quoted strings have their surrounding quotes
                  stripped.
        name_map: The ``*NAME_MAP`` dict from :class:`SpefFile`
                  (``{*id: resolved_name}``).

    Returns:
        The resolved node name as it appears in :attr:`NetRC.res_graph`.
    """
    if not tok:
        return tok
    if tok.startswith('"') and tok.endswith('"') and len(tok) >= 2:
        return tok[1:-1]
    if ':' in tok:
        idx = tok.index(':')
        base = tok[:idx]
        pin = tok[idx + 1:]
        if base and base[0] == '*' and base in name_map:
            r = name_map[base]
            result = f"{r}:{pin}" if pin else r
        else:
            result = base
    elif tok.startswith('*'):
        result = name_map.get(tok, tok)
    else:
        result = tok
    if '\\' in result:
        result = result.replace('\\[', '[').replace('\\]', ']')
    return result


def _compute_res_segment_scales(
    net: 'NetRC',
    sink_ratios: Dict[str, float],
    avg_ratio: float,
) -> Dict[Tuple[str, str], float]:
    """Compute per-segment RES scale factors for backmarking.

    For a net with multiple sinks the resistance tree is analysed so that:

    * **Shared segments** — edges whose subtree (below the edge, away from the
      driver) contains paths to two or more sinks — are scaled by *avg_ratio*,
      the mean of all per-sink (new_R / old_R) ratios.
    * **Exclusive segments** — edges whose subtree reaches exactly one sink —
      are scaled by that particular sink's ratio.

    For a single-sink net every segment is exclusive, and its ratio equals
    *avg_ratio*, so the result is identical to the old uniform-scale logic.

    Segments not reachable from the driver, or not leading to any sink with a
    known ratio, are scaled by *avg_ratio*.

    Returns a dict ``{(node_a, node_b): scale}`` with **both** orderings of
    each edge stored so that the caller can look up either (n1, n2) or (n2, n1)
    without extra sorting.
    """
    if not net.driver or not sink_ratios:
        return {}

    driver_node = net._find_best_node_name(net.driver) or net.driver
    if driver_node not in net.res_graph:
        return {}

    # Map sink pin names → graph node names (only those with known ratios)
    sink_node_to_ratio: Dict[str, float] = {}
    for sink_pin, ratio in sink_ratios.items():
        sink_node = net._find_best_node_name(sink_pin) or sink_pin
        if sink_node in net.res_graph:
            sink_node_to_ratio[sink_node] = ratio

    if not sink_node_to_ratio:
        return {}

    # BFS from driver to build the spanning tree (children map + BFS order)
    children: Dict[str, List[str]] = {}
    parent: Dict[str, Optional[str]] = {driver_node: None}
    order: List[str] = [driver_node]
    queue: deque = deque([driver_node])
    while queue:
        node = queue.popleft()
        children[node] = []
        for neigh, _ in net.res_graph.get(node, []):
            if neigh not in parent:
                parent[neigh] = node
                children[node].append(neigh)
                order.append(neigh)
                queue.append(neigh)

    # Post-order traversal: compute the set of known sinks in each subtree
    sinks_below: Dict[str, Set[str]] = {}
    for node in reversed(order):
        s: Set[str] = set()
        if node in sink_node_to_ratio:
            s.add(node)
        for child in children.get(node, []):
            s |= sinks_below.get(child, set())
        sinks_below[node] = s

    # Assign a scale factor to every tree edge based on subtree sink membership
    edge_scales: Dict[Tuple[str, str], float] = {}
    for child_node, par_node in parent.items():
        if par_node is None:
            continue  # driver root has no incoming edge
        sinks = sinks_below.get(child_node, set())
        if len(sinks) == 1:
            sink_node = next(iter(sinks))
            scale = sink_node_to_ratio[sink_node]
        else:
            # 0 or ≥2 sinks below this edge → use average ratio
            scale = avg_ratio
        # Store both orderings so lookup works regardless of SPEF token order
        edge_scales[(par_node, child_node)] = scale
        edge_scales[(child_node, par_node)] = scale

    return edge_scales


def _fmt_float(val: float) -> str:
    """Format a float for SPEF output, keeping reasonable precision."""
    if val == 0.0:
        return "0"
    abs_val = abs(val)
    if abs_val < 1e-4:
        return f"{val:.6e}"
    return f"{val:.6g}"


def backmark_spef(
    spef_path: str,
    cap_data_path: Optional[str],
    res_data_path: Optional[str],
    output_path: str,
) -> None:
    """Rewrite a SPEF file with updated cap/res values from data files.

    For capacitance:
      - The new total cap replaces *D_NET total_cap.
      - Each *CAP segment value is scaled by (new_total / old_total).

    For resistance:
      - The data file gives new driver-to-sink R per sink.
      - The net's resistance tree is analysed to identify *shared* segments
        (edges whose subtree contains paths to ≥2 sinks) and *exclusive*
        segments (edges leading to exactly one sink).
      - Shared segments are scaled by the average ratio across all sinks
        (avg of new_R / old_R).
      - Exclusive segments are scaled by the ratio of the single sink they
        serve (new_R / old_R for that sink).
      - For single-sink nets all segments are exclusive, so the result is
        identical to a uniform scale by the one available ratio.

    The SPEF is first parsed to gather per-net information, then rewritten
    line-by-line to preserve formatting.
    """
    # ---- Step 1: parse the SPEF to get per-net structure ----
    sf = SpefFile(spef_path)
    sf.parse()

    # Build reverse name_map: net_name -> net_id (e.g. "clk" -> "*3")
    reverse_name_map: Dict[str, str] = {}
    for net_id, net_name in sf.name_map.items():
        reverse_name_map[net_name] = net_id

    # ---- Step 2: load data files and build scaling ratios ----
    cap_ratio: Dict[str, float] = {}           # net_name -> ratio
    new_total_caps: Dict[str, float] = {}      # net_name -> new_total_cap

    if cap_data_path:
        raw_cap = _parse_backmark_cap_data(cap_data_path)
        for key, new_cap in raw_cap.items():
            # Resolve key to net_name
            if key.startswith('*'):
                net_name = sf.name_map.get(key, key)
            else:
                net_name = key
            net = sf.nets.get(net_name)
            if net is None:
                continue
            old_cap = net.total_cap
            new_total_caps[net_name] = new_cap
            cap_ratio[net_name] = (new_cap / old_cap) if old_cap != 0.0 else 1.0

    res_segment_scales: Dict[str, Dict[Tuple[str, str], float]] = {}   # net_name -> edge -> scale
    res_avg_ratio: Dict[str, float] = {}                                # net_name -> fallback avg scale

    if res_data_path:
        raw_res = _parse_backmark_res_data(res_data_path)
        for key, sink_map in raw_res.items():
            if key.startswith('*'):
                net_name = sf.name_map.get(key, key)
            else:
                net_name = key
            net = sf.nets.get(net_name)
            if net is None:
                continue
            old_dr = net.driver_sink_resistances()
            if not old_dr:
                continue
            # Compute per-sink ratios for sinks present in both data file and SPEF
            sink_ratios: Dict[str, float] = {}
            for sink, new_r in sink_map.items():
                old_r = old_dr.get(sink)
                if old_r is not None and old_r > 0.0:
                    sink_ratios[sink] = new_r / old_r
            if not sink_ratios:
                continue
            avg = sum(sink_ratios.values()) / len(sink_ratios)
            res_avg_ratio[net_name] = avg
            res_segment_scales[net_name] = _compute_res_segment_scales(net, sink_ratios, avg)

    print(f"[backmark] Nets with cap update : {len(cap_ratio)}")
    print(f"[backmark] Nets with res update : {len(res_avg_ratio)}")

    # ---- Step 3: rewrite SPEF line-by-line ----
    # We also need the net_id -> net_name map to figure out which net_name
    # we're in when we see "*D_NET *1428 ...".

    SEC_NONE = 0
    SEC_CONN = 1
    SEC_CAP  = 2
    SEC_RES  = 3

    current_net_name: Optional[str] = None
    current_net_id: Optional[str]   = None
    section = SEC_NONE
    c_scale = 1.0
    r_avg_scale = 1.0
    r_edge_scales: Dict[Tuple[str, str], float] = {}
    lines_written = 0

    with open(spef_path, 'r', encoding='utf-8', errors='ignore') as fin, \
         open(output_path, 'w', encoding='utf-8') as fout:
        for line in fin:
            raw = line.strip()

            # --- *D_NET header ---
            if raw.startswith('*D_NET '):
                parts = raw.split(None, 3)
                if len(parts) >= 3:
                    net_id_tok = parts[1]
                    net_name_resolved = sf.name_map.get(net_id_tok, net_id_tok)
                    current_net_name = net_name_resolved
                    current_net_id = net_id_tok
                    section = SEC_NONE
                    c_scale = cap_ratio.get(net_name_resolved, 1.0)
                    r_avg_scale = res_avg_ratio.get(net_name_resolved, 1.0)
                    r_edge_scales = res_segment_scales.get(net_name_resolved, {})

                    if net_name_resolved in new_total_caps:
                        new_tc = new_total_caps[net_name_resolved]
                        # Rewrite the D_NET line with new total cap
                        fout.write(f"*D_NET {net_id_tok} {_fmt_float(new_tc)}\n")
                        lines_written += 1
                        continue
                # If no cap update, write original line
                fout.write(line)
                lines_written += 1
                continue

            # --- Inside a net: detect section headers ---
            if current_net_name is not None:
                if raw == '*CONN':
                    section = SEC_CONN
                    fout.write(line)
                    lines_written += 1
                    continue
                elif raw == '*CAP':
                    section = SEC_CAP
                    fout.write(line)
                    lines_written += 1
                    continue
                elif raw == '*RES':
                    section = SEC_RES
                    fout.write(line)
                    lines_written += 1
                    continue
                elif raw == '*END':
                    current_net_name = None
                    current_net_id = None
                    section = SEC_NONE
                    c_scale = 1.0
                    r_avg_scale = 1.0
                    r_edge_scales = {}
                    fout.write(line)
                    lines_written += 1
                    continue

                # --- *CAP segment: scale cap values ---
                if section == SEC_CAP and c_scale != 1.0:
                    # Strip inline comment before parsing so that parts[-1] is
                    # the numeric cap value, not a word from the comment.
                    ci = raw.find('//')
                    code = raw[:ci].rstrip() if ci >= 0 else raw
                    comment_suffix = (' ' + raw[ci:]) if ci >= 0 else ''
                    parts = code.split()
                    if len(parts) >= 3:
                        try:
                            int(parts[0])  # index
                            # Last token is cap value, second-to-last might be a node
                            # Format: idx node1 cap_val  OR  idx node1 node2 cap_val
                            old_val = float(parts[-1])
                            new_val = old_val * c_scale
                            # Reconstruct line preserving leading whitespace
                            lead = line[:len(line) - len(line.lstrip())]
                            parts[-1] = _fmt_float(new_val)
                            fout.write(lead + ' '.join(parts) + comment_suffix + '\n')
                            lines_written += 1
                            continue
                        except ValueError:
                            pass  # Not a cap data line, write as-is

                # --- *RES segment: scale res values ---
                if section == SEC_RES and (r_edge_scales or r_avg_scale != 1.0):
                    parts = raw.split()
                    if len(parts) >= 4:
                        try:
                            int(parts[0])  # index
                            # Format: idx node1 node2 res_val [trailing space]
                            old_val = float(parts[3])
                            # Resolve node tokens to graph names for edge lookup
                            n1 = _resolve_spef_token(parts[1], sf.name_map)
                            n2 = _resolve_spef_token(parts[2], sf.name_map)
                            seg_scale = r_edge_scales.get((n1, n2))
                            if seg_scale is None:
                                seg_scale = r_edge_scales.get((n2, n1))
                            if seg_scale is None:
                                seg_scale = r_avg_scale
                            if seg_scale == 1.0:
                                pass  # write unchanged below
                            else:
                                new_val = old_val * seg_scale
                                lead = line[:len(line) - len(line.lstrip())]
                                parts[3] = _fmt_float(new_val)
                                fout.write(lead + ' '.join(parts) + ' \n')
                                lines_written += 1
                                continue
                        except ValueError:
                            pass

            # Default: write line unchanged
            fout.write(line)
            lines_written += 1

    print(f"[backmark] Written {lines_written} lines to {output_path}")


# -------------------- shuffle: randomise net_id ↔ net_name mapping --------------------

def shuffle_net_mapping(
    spef_path: str,
    output_path: str,
    seed: Optional[int] = None,
) -> None:
    """Generate a new SPEF with a randomly shuffled net_id ↔ net_name mapping.

    The algorithm applies a *global token substitution* on every ``*N`` token
    in the file where N is a net-type NAME_MAP entry.  Concretely:

    1. Parse the ``*NAME_MAP`` section to find all ``*N → name`` entries.
    2. Identify which ``*N`` IDs appear as the first argument of ``*D_NET``
       lines (these are the "net IDs").
    3. Randomly permute the net_names among those net IDs to produce a
       bijective substitution ``old_net_id → new_net_id``.
    4. Replace every ``*N`` token (where N is a net ID) throughout the file
       with the new ID from step 3.

    The effect is:

    * The ``*NAME_MAP`` entries for nets are re-labelled (mapping shuffled).
    * Every ``*D_NET`` header is updated to carry the new ID of that net.
    * All internal node references (``*N:pin``) inside each ``*D_NET`` block
      are updated consistently, so the resolved pin names are unchanged.

    Because each net_name continues to own exactly its original RC data, a
    comparison of the shuffled SPEF against the original produces identical
    ``net_cap.data`` and ``net_res.data`` files.

    Args:
        spef_path:   Path to the input SPEF file.
        output_path: Path to write the shuffled SPEF.
        seed:        Optional random seed for reproducibility.
    """
    rng = random.Random(seed)

    _re_nid = re.compile(r'^\*\d+$')

    # ---- Pass 1: collect NAME_MAP and net_ids ----
    name_map: Dict[str, str] = {}   # *N -> resolved_name (all entries)
    net_ids: List[str] = []         # *N tokens from *D_NET lines (first-seen order)
    net_ids_set: Set[str] = set()   # fast membership test

    in_name_map = False
    with open(spef_path, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            raw = line.strip()
            if not raw:
                continue
            if raw.startswith('*NAME_MAP'):
                in_name_map = True
                continue
            if in_name_map:
                parts = raw.split(None, 1)
                if len(parts) == 2 and _re_nid.match(parts[0]):
                    name_map[parts[0]] = parts[1].strip('"')
                    continue
                # Non-matching line ends the NAME_MAP section
                in_name_map = False
            if raw.startswith('*D_NET '):
                parts = raw.split(None, 2)
                if len(parts) >= 2:
                    nid = parts[1]
                    if _re_nid.match(nid) and nid not in net_ids_set:
                        net_ids.append(nid)
                        net_ids_set.add(nid)

    if len(net_ids) < 2:
        shutil.copy2(spef_path, output_path)
        print(f"[shuffle] Only {len(net_ids)} net(s) found; file copied unchanged.")
        return

    # ---- Build shuffled assignment ----
    net_names: List[str] = [name_map.get(nid, nid) for nid in net_ids]
    shuffled_names: List[str] = list(net_names)
    # Retry up to 20 times to avoid the identity permutation.  The probability
    # of landing on the identity for N≥2 elements is 1/N!, so 20 tries gives a
    # failure probability < (1/2)^20 ≈ 1e-6.
    for _ in range(20):
        rng.shuffle(shuffled_names)
        if shuffled_names != net_names:
            break

    # new_nid_for_name[net_name] = net_id that net_name will occupy in new SPEF
    new_nid_for_name: Dict[str, str] = {
        shuffled_names[i]: net_ids[i] for i in range(len(net_ids))
    }

    # Global substitution map: old_nid -> new_nid
    # For a given old_nid with net_names[i], the name moves to new_nid_for_name[net_names[i]]
    subst: Dict[str, str] = {
        net_ids[i]: new_nid_for_name[net_names[i]] for i in range(len(net_ids))
    }

    # ---- Pass 2: apply global substitution and write ----
    net_nums: Set[str] = {nid[1:] for nid in net_ids}   # digit strings only

    _pat = re.compile(r'\*(\d+)')

    def _replace(m: re.Match) -> str:
        num = m.group(1)
        if num in net_nums:
            old_nid = '*' + num
            return subst.get(old_nid, old_nid)
        return m.group(0)

    with open(spef_path, 'r', encoding='utf-8', errors='ignore') as fin, \
         open(output_path, 'w', encoding='utf-8') as fout:
        for line in fin:
            if '*' in line:
                line = _pat.sub(_replace, line)
            fout.write(line)

    n_changed = sum(1 for nid in net_ids if subst[nid] != nid)
    print(f"[shuffle] Written shuffled SPEF to {output_path}")
    print(f"[shuffle] {len(net_ids)} nets total; {n_changed} net_ids reassigned")


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


def _draw_diff_histogram_ax(ax, diffs: List[float], title: str, n_bins: int = 100) -> None:
    """Draw an absolute difference histogram with coloured sigma-band backgrounds.

    Background bands:
        yellow       – within ±1σ of the mean
        purple/lavender – ±1σ to ±2σ
        pink         – beyond ±2σ

    Bar colours (based on which band the bin centre falls in):
        dark green – within ±1σ
        blue       – ±1σ to ±2σ
        red        – ±2σ to ±3σ
        black      – beyond ±3σ

    A text box in the upper-right corner shows Mean, StdDev, Min, Max.
    """
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

    xmin = float(bin_edges[0])
    xmax = float(bin_edges[-1])

    # ------------------------------------------------------------------
    # Coloured background bands
    # ------------------------------------------------------------------
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

    # ------------------------------------------------------------------
    # Histogram bars
    # ------------------------------------------------------------------
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
        ax.bar(float(left), int(count), width=float(right - left),
               align="edge", color=color, zorder=2)

    ax.set_xlim(xmin, xmax)

    # ------------------------------------------------------------------
    # Stats text box
    # ------------------------------------------------------------------
    def _fmt(v: float) -> str:
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
    # Underline the header line by annotating separately
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


def collect_spef_paths(inputs: Iterable[str]) -> List[str]:
    """Expand file and directory inputs into a SPEF path list.

    Duplicates are preserved so the same SPEF can be loaded twice for testing.
    """
    spef_paths: List[str] = []

    for raw_path in inputs:
        if not raw_path:
            continue
        path = os.path.abspath(raw_path)
        if os.path.isdir(path):
            try:
                names = sorted(os.listdir(path))
            except OSError:
                continue
            for name in names:
                child = os.path.join(path, name)
                if not os.path.isfile(child):
                    continue
                if not name.lower().endswith(".spef"):
                    continue
                spef_paths.append(child)
            continue

        if os.path.isfile(path) and path.lower().endswith(".spef"):
            spef_paths.append(path)

    return spef_paths


def launch_gui(preload_paths: Optional[Iterable[str]] = None, auto_run: bool = False,
               preload_cap_data: Optional[str] = None,
               preload_res_data: Optional[str] = None) -> None:
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
        def __init__(self, root: "tk.Tk", preload_paths: Optional[Iterable[str]] = None,
                     auto_run: bool = False,
                     preload_cap_data: Optional[str] = None,
                     preload_res_data: Optional[str] = None) -> None:
            self.root = root
            self.root.title("SPEF RC Correlation")

            # name -> SpefFile
            self.spefs: Dict[str, SpefFile] = {}

            # data loaded from .data files
            self._data_caps: List[CapComparison] = []
            self._data_ress: List[ResComparison] = []

            # selection & filter state
            self.ref_var = tk.StringVar()
            self.fit_var = tk.StringVar()
            self.r_agg_var = tk.StringVar(value="max")

            self.min_fanout_var = tk.StringVar(value="1")
            self.max_fanout_var = tk.StringVar(value="99999")
            self.min_c_var = tk.StringVar(value="0.001")
            self.max_c_var = tk.StringVar(value="2")
            self.min_r_var = tk.StringVar(value="1")
            self.max_r_var = tk.StringVar(value="1000000")

            self.points_cap: List[Dict[str, float]] = []  # per-net data after filtering
            self.points_res: List[Dict[str, float]] = []  # per-pin-pair data after filtering
            self._auto_run_requested = auto_run

            self._build_ui()

            if preload_paths:
                self._preload_spefs(preload_paths)
            if preload_cap_data:
                self._load_cap_data_file(preload_cap_data)
            if preload_res_data:
                self._load_res_data_file(preload_res_data)

            if self._auto_run_requested:
                if self._data_caps or self._data_ress:
                    self.root.after(0, self._run_analysis_from_data)
                elif len(self.spefs) >= 2:
                    self.root.after(0, self._auto_select_and_run)
                self._auto_run_requested = False

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

            # Data Files frame
            data_frame = ttk.LabelFrame(self.root, text="Data Files")
            data_frame.pack(fill="x", padx=5, pady=5)

            cap_row = ttk.Frame(data_frame)
            cap_row.pack(fill="x", padx=5, pady=2)
            ttk.Label(cap_row, text="Cap Data File:").pack(side="left")
            self.cap_data_label = ttk.Label(cap_row, text="(none)", foreground="gray", anchor="w")
            self.cap_data_label.pack(side="left", padx=5, fill="x", expand=True)
            ttk.Button(cap_row, text="Load Cap Data...", command=self._add_cap_data).pack(side="right")

            res_row = ttk.Frame(data_frame)
            res_row.pack(fill="x", padx=5, pady=2)
            ttk.Label(res_row, text="Res Data File:").pack(side="left")
            self.res_data_label = ttk.Label(res_row, text="(none)", foreground="gray", anchor="w")
            self.res_data_label.pack(side="left", padx=5, fill="x", expand=True)
            ttk.Button(res_row, text="Load Res Data...", command=self._add_res_data).pack(side="right")

            data_btn_row = ttk.Frame(data_frame)
            data_btn_row.pack(fill="x", padx=5, pady=(0, 4))
            ttk.Button(data_btn_row, text="Run from Data Files",
                       command=self._run_analysis_from_data).pack(side="left")

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

            ttk.Button(opts_frame, text="Diff Histogram", command=self._show_diff_histogram).grid(
                row=4,
                column=4,
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

        def _load_spef_path(self, path: str, name: Optional[str] = None) -> None:
            if not path:
                return

            if name is None:
                base_name = os.path.splitext(os.path.basename(path))[0] or "spef"
                name = base_name
                suffix = 2
                while name in self.spefs:
                    name = f"{base_name}_{suffix}"
                    suffix += 1

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
            self._refresh_spef_choices()

        def _preload_spefs(self, preload_paths: Iterable[str]) -> None:
            paths = list(preload_paths)
            errors: List[str] = []

            # When there are 2+ paths, parse them in parallel to halve the wait time
            if len(paths) >= 2:
                from concurrent.futures import ProcessPoolExecutor
                try:
                    self.root.config(cursor="watch")
                    self.root.update_idletasks()
                    with ProcessPoolExecutor(max_workers=len(paths)) as ex:
                        futures = {ex.submit(_parse_one, p): p for p in paths}
                    spefs_loaded: List[Tuple[str, "SpefFile"]] = []
                    for fut, p in futures.items():
                        try:
                            spefs_loaded.append((p, fut.result()))
                        except Exception as exc:
                            errors.append(f"{p}: {exc}")
                except Exception as exc:
                    errors.append(f"parallel preload failed: {exc}; retrying sequentially")
                    spefs_loaded = []
                    for p in paths:
                        try:
                            spef = SpefFile(p)
                            spef.parse()
                            spefs_loaded.append((p, spef))
                        except Exception as e2:
                            errors.append(f"{p}: {e2}")
                finally:
                    self.root.config(cursor="")
                    self.root.update_idletasks()

                for p, spef in spefs_loaded:
                    base_name = os.path.splitext(os.path.basename(p))[0] or "spef"
                    name = base_name
                    suffix = 2
                    while name in self.spefs:
                        name = f"{base_name}_{suffix}"
                        suffix += 1
                    self.spefs[name] = spef
                    self.tree.insert("", "end", iid=name, values=(name, len(spef.nets), p))
                    self._refresh_spef_choices()
            else:
                for path in paths:
                    try:
                        self._load_spef_path(path)
                    except Exception as exc:
                        errors.append(f"{path}: {exc}")

            if errors:
                messagebox.showwarning(
                    "Preload warning",
                    "Some SPEF files could not be loaded at startup:\n\n" + "\n".join(errors),
                )

        # ----------------------------- Data file loading --------------------------

        def _add_cap_data(self) -> None:
            path = filedialog.askopenfilename(
                title="Select net_cap.data file",
                filetypes=[("Data files", "*.data"), ("All files", "*.*")],
            )
            if path:
                self._load_cap_data_file(path)

        def _add_res_data(self) -> None:
            path = filedialog.askopenfilename(
                title="Select net_res.data file",
                filetypes=[("Data files", "*.data"), ("All files", "*.*")],
            )
            if path:
                self._load_res_data_file(path)

        def _load_cap_data_file(self, path: str) -> None:
            try:
                self._data_caps = parse_net_cap_data(path)
            except Exception as exc:
                messagebox.showerror("Parse error", f"Failed to parse cap data file:\n{exc}")
                return
            label = os.path.basename(path)
            self.cap_data_label.config(text=f"{label}  ({len(self._data_caps)} nets)", foreground="black")
            print(f"Cap data loaded: {path} ({len(self._data_caps)} entries)")

        def _load_res_data_file(self, path: str) -> None:
            try:
                self._data_ress = parse_net_res_data(path)
            except Exception as exc:
                messagebox.showerror("Parse error", f"Failed to parse res data file:\n{exc}")
                return
            label = os.path.basename(path)
            self.res_data_label.config(text=f"{label}  ({len(self._data_ress)} pairs)", foreground="black")
            print(f"Res data loaded: {path} ({len(self._data_ress)} entries)")

        def _run_analysis_from_data(self) -> None:
            if not self._data_caps and not self._data_ress:
                messagebox.showwarning("No Data", "Please load at least one data file (cap or res).")
                return

            flt = self._parse_filters()
            if flt is None:
                return

            mc = flt["min_c"]
            xc = flt["max_c"]
            mr = flt["min_r"]
            xr = flt["max_r"]

            points_cap: List[Dict[str, float]] = []
            for c in self._data_caps:
                if mc is not None and c.c1 < mc:
                    continue
                if xc is not None and c.c1 > xc:
                    continue
                points_cap.append({"net": c.net, "c_ref": c.c1, "c_fit": c.c2})

            points_res: List[Dict[str, float]] = []
            for r in self._data_ress:
                if mr is not None and r.r1 < mr:
                    continue
                if xr is not None and r.r1 > xr:
                    continue
                points_res.append({"net": r.net, "r_ref": r.r1, "r_fit": r.r2})

            self.points_cap = points_cap
            self.points_res = points_res

            if not points_cap and not points_res:
                self.corr_label.config(text="No data after filtering")
                self._clear_axes()
                return

            xs_c = [p["c_ref"] for p in points_cap]
            ys_c = [p["c_fit"] for p in points_cap]
            xs_r = [p["r_ref"] for p in points_res]
            ys_r = [p["r_fit"] for p in points_res]

            corr_c = pearson_corr(xs_c, ys_c)
            corr_r = pearson_corr(xs_r, ys_r)

            c_text = "C corr: N/A"
            if corr_c is not None:
                c_text = f"C corr ({len(points_cap)} nets): {corr_c:.4f}"
            r_text = "R corr: N/A"
            if corr_r is not None:
                r_text = f"R corr ({len(points_res)} pin pairs): {corr_r:.4f}"
            self.corr_label.config(text=f"{c_text} | {r_text}")

            self._update_plot("tool1", "tool2", corr_c, corr_r)

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

        def _auto_select_and_run(self) -> None:
            names = list(self.tree.get_children())
            if len(names) < 2:
                return
            self.ref_var.set(names[0])
            self.fit_var.set(names[1])
            self._auto_run_requested = False
            self._run_analysis()

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
            caps, ress, _top_10_cap, _top_10_res = compare_spef(ref, fit, r_agg)

            print(f"{len(caps)} nets caps compared")
            print(f"{len(ress)} pin pair res compared")
            print('cap deviation top 10 (ref : fit)')
            for cap_com in _top_10_cap:
                print(f"net:{cap_com.net} ({cap_com.c1} : {cap_com.c2})")
            print('res deviation top 10 (ref : fit)')
            for res_com in _top_10_res:
                print(f"net:{res_com.net} driver:{res_com.driver} load:{res_com.load} ({res_com.r1} : {res_com.r2})")

            points_cap: List[Dict[str, float]] = []
            for c in caps:
                # r = res_map[net]
                net = c.net
                fanout = len(ref.nets[net].sinks) if net in ref.nets else 0
                p = {
                    "net": net,
                    "fanout": float(fanout),
                    "c_ref": c.c1,
                    "c_fit": c.c2,
                }
                if self._passes_filters(p, flt):
                    points_cap.append(p)
            

            points_res: List[Dict[str, float]] = []
            for pin_pair in ress:
                net = pin_pair.net
                fanout = len(ref.nets[net].sinks) if net in ref.nets else 0
                p = {
                    "net": net,
                    "fanout": float(fanout),
                    "r_ref": pin_pair.r1,
                    "r_fit": pin_pair.r2,
                }
                if self._passes_filters(p, flt):
                    points_res.append(p)


            self.points_cap = points_cap
            self.points_res = points_res

            if not points_cap:
                self.corr_label.config(text="No nets after filtering")
                self._clear_axes()
                return

            xs_c = [p["c_ref"] for p in points_cap]
            ys_c = [p["c_fit"] for p in points_cap]
            xs_r = [p["r_ref"] for p in points_res]
            ys_r = [p["r_fit"] for p in points_res]

            corr_c = pearson_corr(xs_c, ys_c)
            corr_r = pearson_corr(xs_r, ys_r)

            c_text = "C corr: N/A"
            if corr_c is not None:
                c_text = f"C corr ({len(points_cap)} nets): {corr_c:.4f}"
            r_text = "R corr: N/A"
            if corr_r is not None:
                r_text = f"R corr ({len(points_res)} pin pairs): {corr_r:.4f}"
            self.corr_label.config(text=f"{c_text} | {r_text}")

            self._update_plot(ref_name, fit_name, corr_c, corr_r)

        def _clear_axes(self) -> None:
            self.ax_c.clear()
            self.ax_r.clear()
            self.ax_c.set_title("No data")
            self.ax_r.set_title("")
            self.canvas.draw()

        def _show_diff_histogram(self) -> None:
            """Open a new window with absolute difference histograms for C and R."""
            if not self.points_cap and not self.points_res:
                messagebox.showwarning("No Data", "Please run the analysis first.")
                return

            ref_name = self.ref_var.get() or "ref"
            fit_name = self.fit_var.get() or "fit"

            diffs_c = [p["c_fit"] - p["c_ref"] for p in self.points_cap]
            diffs_r = [p["r_fit"] - p["r_ref"] for p in self.points_res]

            nrows = (1 if diffs_c else 0) + (1 if diffs_r else 0)
            if nrows == 0:
                messagebox.showwarning("No Data", "No difference data available.")
                return

            win = tk.Toplevel(self.root)
            win.title(f"Absolute Difference Histogram  {ref_name} – {fit_name}")

            from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg as _FCA
            from matplotlib.figure import Figure as _Figure

            fig = _Figure(figsize=(10, 5 * nrows))
            idx = 1
            if diffs_c:
                ax_c = fig.add_subplot(nrows, 1, idx)
                _draw_diff_histogram_ax(
                    ax_c, diffs_c,
                    f"Absolute Difference Histogram {ref_name} - {fit_name}  (C)"
                )
                idx += 1
            if diffs_r:
                ax_r = fig.add_subplot(nrows, 1, idx)
                _draw_diff_histogram_ax(
                    ax_r, diffs_r,
                    f"Absolute Difference Histogram {ref_name} - {fit_name}  (R)"
                )

            fig.tight_layout()
            canvas = _FCA(fig, master=win)
            canvas.get_tk_widget().pack(fill="both", expand=True)
            canvas.draw()

        def _update_plot(self, ref_name: str, fit_name: str,
                          corr_c: Optional[float], corr_r: Optional[float]) -> None:
            self.ax_c.clear()
            self.ax_r.clear()

            # Capacitance plot
            xs_c = [p["c_ref"] for p in self.points_cap]
            ys_c = [p["c_fit"] for p in self.points_cap]
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
            xs_r = [p["r_ref"] for p in self.points_res]
            ys_r = [p["r_fit"] for p in self.points_res]
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

            title_r = f"Driver->sink R: {ref_name} (X) vs {fit_name} (Y)"
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
            if not self.points_cap and not self.points_res:
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
                xs = [p["c_ref"] for p in self.points_cap]
                ys = [p["c_fit"] for p in self.points_cap]
                annot = self.annot_c
            else:
                xs = [p["r_ref"] for p in self.points_res]
                ys = [p["r_fit"] for p in self.points_res]
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
                if ax is self.ax_c:
                    p = self.points_cap[best_i]
                    text = (
                        f"net: {p['net']}\n"
                        f"C_ref={p['c_ref']:.3g}, C_fit={p['c_fit']:.3g}"
                    )
                else:
                    p = self.points_res[best_i]
                    text = (
                        f"net: {p['net']}\n"
                        f"R_ref={p['r_ref']:.3g}, R_fit={p['r_fit']:.3g}"
                    )
                annot.xy = (xs[best_i], ys[best_i])
                annot.set_text(text)
                annot.get_bbox_patch().set_facecolor("#ffffcc")
                annot.set_visible(True)
                self.canvas.draw_idle()
            else:
                if annot.get_visible():
                    annot.set_visible(False)
                    self.canvas.draw_idle()

    root = tk.Tk()
    RcCorrApp(root, preload_paths=preload_paths, auto_run=auto_run,
              preload_cap_data=preload_cap_data, preload_res_data=preload_res_data)
    root.mainloop()


# ---------------------------- parallel parse helpers --------------------


def _parse_one(path: str) -> "SpefFile":
    """Top-level worker: parse a single SPEF and return the SpefFile.

    Must be a module-level function so multiprocessing can pickle it.
    """
    spef = SpefFile(path)
    spef.parse()
    return spef


def parse_spefs_parallel(path1: str, path2: str) -> Tuple["SpefFile", "SpefFile"]:
    """Parse two SPEF files concurrently in separate processes.

    Returns (spef1, spef2) after both have finished.
    Falls back to sequential parsing if the subprocess pool fails for any reason.
    """
    from concurrent.futures import ProcessPoolExecutor

    print(f"Parsing {path1} and {path2} in parallel ...")
    try:
        with ProcessPoolExecutor(max_workers=2) as ex:
            f1 = ex.submit(_parse_one, path1)
            f2 = ex.submit(_parse_one, path2)
            s1 = f1.result()
            s2 = f2.result()
        return s1, s2
    except Exception as exc:  # pragma: no cover
        print(f"[warn] parallel parse failed ({exc}), falling back to sequential")
        s1 = SpefFile(path1)
        s1.parse()
        s2 = SpefFile(path2)
        s2.parse()
        return s1, s2


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
    parser.add_argument(
        "--gui-auto-run",
        action="store_true",
        help="In GUI mode, auto-select the first two loaded SPEFs and run analysis once",
    )
    parser.add_argument(
        "--net-cap-data",
        metavar="FILE",
        help=(
            "Path to a pre-computed net_cap.data file. "
            "Each line: net_name  c_tool1  c_tool2. "
            "When supplied, capacitance correlation is computed from this file "
            "instead of parsing two SPEF files."
        ),
    )
    parser.add_argument(
        "--net-res-data",
        metavar="FILE",
        help=(
            "Path to a pre-computed net_res.data file. "
            "Each line: net_name  driver_pin  sink_pin  r_tool1  r_tool2. "
            "When supplied, resistance correlation is computed from this file "
            "instead of parsing two SPEF files."
        ),
    )

    parser.add_argument(
        "--backmark",
        action="store_true",
        help=(
            "Backmark mode: apply new cap/res values from --net-cap-data "
            "and --net-res-data to the SPEF file given as spef1, "
            "and write the updated SPEF. Use --output to specify output path."
        ),
    )
    parser.add_argument(
        "--shuffle",
        action="store_true",
        help=(
            "Shuffle mode: randomly permute the net_id ↔ net_name mapping "
            "in the SPEF given as spef1, while preserving RC data per net_name. "
            "Comparing the output against the original produces identical "
            "net_cap.data and net_res.data. "
            "Use --output for the output path and --seed for reproducibility."
        ),
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        metavar="INT",
        help="Random seed for --shuffle mode (default: random).",
    )
    parser.add_argument(
        "--output",
        "-o",
        metavar="FILE",
        help="Output path for backmarked or shuffled SPEF (default: <spef1>_backmarked.spef or <spef1>_shuffled.spef)",
    )

    args = parser.parse_args()

    # ------------------------------------------------------------------
    # Backmark mode
    # ------------------------------------------------------------------
    if args.backmark:
        if not args.spef1:
            parser.error("--backmark requires spef1 (the SPEF file to update).")
        if not args.net_cap_data and not args.net_res_data:
            parser.error("--backmark requires at least one of --net-cap-data or --net-res-data.")
        out = args.output
        if not out:
            base, ext = os.path.splitext(args.spef1)
            out = f"{base}_backmarked{ext}"
        backmark_spef(args.spef1, args.net_cap_data, args.net_res_data, out)
        return

    # ------------------------------------------------------------------
    # Shuffle mode
    # ------------------------------------------------------------------
    if args.shuffle:
        if not args.spef1:
            parser.error("--shuffle requires spef1 (the SPEF file to shuffle).")
        out = args.output
        if not out:
            base, ext = os.path.splitext(args.spef1)
            out = f"{base}_shuffled{ext}"
        shuffle_net_mapping(args.spef1, out, seed=args.seed)
        return

    # ------------------------------------------------------------------
    # Data-file mode: at least one of --net-cap-data / --net-res-data
    # ------------------------------------------------------------------
    if args.net_cap_data or args.net_res_data:
        if args.gui:
            # GUI mode with data files: pass data paths to the GUI
            gui_inputs = collect_spef_paths([p for p in [args.spef1, args.spef2] if p])
            launch_gui(
                gui_inputs,
                auto_run=args.gui_auto_run,
                preload_cap_data=args.net_cap_data,
                preload_res_data=args.net_res_data,
            )
            return

        caps: List[CapComparison] = []
        ress: List[ResComparison] = []

        if args.net_cap_data:
            print(f"Parsing cap data from {args.net_cap_data} ...")
            caps = parse_net_cap_data(args.net_cap_data)
            print(f"  {len(caps)} cap entries loaded")

        if args.net_res_data:
            print(f"Parsing res data from {args.net_res_data} ...")
            ress = parse_net_res_data(args.net_res_data)
            print(f"  {len(ress)} res entries loaded")

        print("=== SPEF RC Correlation Summary (from data files) ===")
        if args.net_cap_data:
            print(f"Cap data file: {args.net_cap_data}")
        if args.net_res_data:
            print(f"Res data file: {args.net_res_data}")

        xs_c = [c.c1 for c in caps]
        ys_c = [c.c2 for c in caps]
        corr_c = pearson_corr(xs_c, ys_c)
        if caps:
            if corr_c is not None:
                print(f"Total C correlation (Pearson, per-net): {corr_c:.6f} over {len(caps)} nets")
            else:
                print("Total C correlation: N/A (not enough data or zero variance)")

        xs_r = [r.r1 for r in ress]
        ys_r = [r.r2 for r in ress]
        corr_r = pearson_corr(xs_r, ys_r)
        if ress:
            if corr_r is not None:
                print(
                    f"Driver->sink R correlation (Pearson): "
                    f"{corr_r:.6f} over {len(ress)} (net, sink) pairs"
                )
            else:
                print("Driver->sink R correlation: N/A (not enough data or zero variance)")

        cap_devs = [(abs(row.c1 - row.c2), row) for row in caps]
        res_devs = [(abs(row.r1 - row.r2), row) for row in ress]
        top_10_cap = [row for _, row in nlargest(10, cap_devs, key=lambda x: x[0])]
        top_10_res = [row for _, row in nlargest(10, res_devs, key=lambda x: x[0])]

        if top_10_cap:
            print("Cap deviation top 10 (tool1 : tool2):")
            for cap_com in top_10_cap:
                print(f"  net:{cap_com.net} ({cap_com.c1} : {cap_com.c2})")
        if top_10_res:
            print("Res deviation top 10 (tool1 : tool2):")
            for res_com in top_10_res:
                print(
                    f"  net:{res_com.net} driver:{res_com.driver} "
                    f"load:{res_com.load} ({res_com.r1} : {res_com.r2})"
                )

        if args.csv_prefix:
            if caps:
                caps_path = f"{args.csv_prefix}_caps.csv"
                write_caps_csv(caps_path, caps)
                print(f"\nCSV written: {caps_path}")
            if ress:
                res_path = f"{args.csv_prefix}_res_{args.r_agg}.csv"
                write_res_csv(res_path, ress, args.r_agg)
                print(f"CSV written: {res_path}")
        return

    # ------------------------------------------------------------------
    # Normal SPEF mode
    # ------------------------------------------------------------------
    gui_inputs = collect_spef_paths([p for p in [args.spef1, args.spef2] if p])

    if args.gui or (args.spef1 is None and args.spef2 is None):
        launch_gui(gui_inputs, auto_run=args.gui_auto_run)
        return

    if args.spef1 is None or args.spef2 is None:
        parser.error("spef1 and spef2 are required in CLI mode (or use --gui).")

    s1, s2 = parse_spefs_parallel(args.spef1, args.spef2)

    caps, ress, _top_10_cap, _top_10_res = compare_spef(s1, s2, args.r_agg)
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