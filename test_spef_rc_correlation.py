"""
Regression tests for spef_rc_correlation.py

Coverage:
  - pearson_corr: perfect correlation, anti-correlation, constant series, mismatched lengths
  - NetRC._dijkstra_all: direct edge, multi-hop, unreachable node
  - NetRC.driver_sink_resistances: result caching, prefix-match fallback
  - SpefFile.parse: NAME_MAP, *D_NET/*CONN/*RES/*END, KOHM unit scaling,
                    quoted names, comment stripping, unknown-direction lines
  - compare_spef: same-file → correlation 1.0, disjoint nets → empty result
  - write_caps_csv / write_res_csv: round-trip CSV content check
  - collect_spef_paths: directory expansion, direct file path, non-spef file filtering
  - parse_spefs_parallel: returns two SpefFile objects with the same nets
  - shuffle_net_mapping: RC data preserved per net_name, mapping actually shuffled,
                         seed reproducibility, single-net file copied unchanged
  - backmark_spef: basic cap/res updates, inline comments on *CAP lines,
                   complete shuffle→compare→backmark round-trip, edge cases
"""

import csv
import io
import os
import sys
import tempfile
import textwrap

import pytest

import spef_rc_correlation as spef_mod

from spef_rc_correlation import (
    CapComparison,
    NetRC,
    ResComparison,
    SpefFile,
    backmark_spef,
    collect_spef_paths,
    compare_spef,
    parse_net_cap_data,
    parse_net_res_data,
    parse_spefs_parallel,
    pearson_corr,
    shuffle_net_mapping,
    write_caps_csv,
    write_res_csv,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

# A minimal but complete SPEF snippet used across multiple tests.
MINIMAL_SPEF = textwrap.dedent("""\
    *SPEF "IEEE 1481-1999"
    *DESIGN "test"
    *DATE "2026:01:01:00:00:00"
    *VENDOR "test"
    *PROGRAM "test"
    *VERSION "1.0"
    *DESIGN_FLOW "NETLIST"
    *DIVIDER /
    *DELIMITER :
    *BUS_DELIMITER [ ]
    *T_UNIT 1 NS
    *C_UNIT 1 PF
    *R_UNIT 1 OHM
    *L_UNIT 1 HENRY

    *NAME_MAP
    *1 net_A
    *2 drv_u1:Z
    *3 sink_u2:A
    *4 sink_u3:B

    *D_NET *1 1.5
    *CONN
    *I *2 O *C 0.0 0.0
    *I *3 I *C 0.0 0.0
    *I *4 I *C 0.0 0.0
    *CAP
    1 *2 0.5
    2 *3 0.5
    3 *4 0.5
    *RES
    *1 *2 *3 10.0
    *2 *3 *4 5.0
    *END
""")


def _write_temp_spef(content: str) -> str:
    """Write *content* to a named temp file and return its path."""
    f = tempfile.NamedTemporaryFile(
        suffix=".spef", mode="w", delete=False, encoding="utf-8"
    )
    f.write(content)
    f.close()
    return f.name


def _make_spef_object(content: str = MINIMAL_SPEF) -> SpefFile:
    """Parse *content* from a temp file and return the SpefFile."""
    path = _write_temp_spef(content)
    try:
        sf = SpefFile(path)
        sf.parse()
    finally:
        os.unlink(path)
    return sf


# ---------------------------------------------------------------------------
# pearson_corr
# ---------------------------------------------------------------------------

class TestPearsonCorr:
    def test_perfect_positive(self):
        xs = [1.0, 2.0, 3.0, 4.0]
        assert pearson_corr(xs, xs) == pytest.approx(1.0)

    def test_perfect_negative(self):
        xs = [1.0, 2.0, 3.0]
        ys = [3.0, 2.0, 1.0]
        assert pearson_corr(xs, ys) == pytest.approx(-1.0)

    def test_constant_series_returns_none(self):
        # Zero variance → undefined correlation
        assert pearson_corr([5.0, 5.0, 5.0], [1.0, 2.0, 3.0]) is None

    def test_empty_returns_none(self):
        assert pearson_corr([], []) is None

    def test_length_mismatch_returns_none(self):
        assert pearson_corr([1.0, 2.0], [1.0]) is None

    def test_uncorrelated(self):
        # [1, -1, 1, -1] vs [1, 1, -1, -1] are uncorrelated
        xs = [1.0, -1.0, 1.0, -1.0]
        ys = [1.0, 1.0, -1.0, -1.0]
        result = pearson_corr(xs, ys)
        assert result is not None
        assert abs(result) < 0.01


# ---------------------------------------------------------------------------
# NetRC – shortest resistance / driver-sink resistances
# ---------------------------------------------------------------------------

class TestNetRC:
    def _simple_net(self) -> NetRC:
        """
        driver -- 10 Ω -- mid -- 5 Ω -- sink
        """
        net = NetRC(name="n1", total_cap=1.0, driver="drv", sinks=["sink"])
        net.res_graph = {
            "drv":  [("mid",  10.0)],
            "mid":  [("drv",  10.0), ("sink", 5.0)],
            "sink": [("mid",   5.0)],
        }
        return net

    def test_direct_edge(self):
        net = self._simple_net()
        # drv → mid is a single hop of 10 Ω
        dist = net._dijkstra_all("drv")
        assert dist["mid"] == pytest.approx(10.0)

    def test_multi_hop(self):
        net = self._simple_net()
        dist = net._dijkstra_all("drv")
        assert dist["sink"] == pytest.approx(15.0)

    def test_same_node_is_zero(self):
        net = self._simple_net()
        dist = net._dijkstra_all("drv")
        assert dist["drv"] == pytest.approx(0.0)

    def test_unreachable_returns_none(self):
        net = self._simple_net()
        dist = net._dijkstra_all("drv")
        assert "ghost" not in dist

    def test_missing_src_returns_none(self):
        net = self._simple_net()
        dist = net._dijkstra_all("ghost")
        assert dist == {}

    def test_driver_sink_resistances(self):
        net = self._simple_net()
        result = net.driver_sink_resistances()
        assert result == {"sink": pytest.approx(15.0)}

    def test_result_is_cached(self):
        net = self._simple_net()
        r1 = net.driver_sink_resistances()
        # Mutate graph to verify the cache is returned, not recomputed
        net.res_graph["drv"].append(("sink", 1.0))
        r2 = net.driver_sink_resistances()
        assert r1 is r2

    def test_prefix_match_fallback(self):
        """
        Pin name in *CONN is 'drv', but *RES node is 'drv:1'.
        _find_best_node_name must map drv → drv:1.
        """
        net = NetRC(name="n2", total_cap=0.5, driver="drv", sinks=["sink"])
        net.res_graph = {
            "drv:1":  [("sink:1", 7.0)],
            "sink:1": [("drv:1",  7.0)],
        }
        result = net.driver_sink_resistances()
        assert result == {"sink": pytest.approx(7.0)}

    def test_no_driver_returns_empty(self):
        net = NetRC(name="n3", total_cap=1.0, driver=None, sinks=["sink"])
        assert net.driver_sink_resistances() == {}

    def test_no_sinks_returns_empty(self):
        net = NetRC(name="n4", total_cap=1.0, driver="drv", sinks=[])
        net.res_graph = {"drv": []}
        assert net.driver_sink_resistances() == {}


# ---------------------------------------------------------------------------
# SpefFile.parse
# ---------------------------------------------------------------------------

class TestSpefFileParse:
    def test_basic_parse(self):
        sf = _make_spef_object()
        assert "net_A" in sf.nets

    def test_net_total_cap(self):
        sf = _make_spef_object()
        assert sf.nets["net_A"].total_cap == pytest.approx(1.5)

    def test_driver_resolved(self):
        sf = _make_spef_object()
        assert sf.nets["net_A"].driver == "drv_u1:Z"

    def test_sinks_resolved(self):
        sf = _make_spef_object()
        sinks = set(sf.nets["net_A"].sinks)
        assert "sink_u2:A" in sinks
        assert "sink_u3:B" in sinks

    def test_resistance_graph_built(self):
        sf = _make_spef_object()
        g = sf.nets["net_A"].res_graph
        # NAME_MAP resolves *2→drv_u1:Z, *3→sink_u2:A, *4→sink_u3:B
        assert "drv_u1:Z" in g
        assert "sink_u2:A" in g

    def test_resistance_values(self):
        sf = _make_spef_object()
        g = sf.nets["net_A"].res_graph
        neighbors = dict(g["drv_u1:Z"])
        assert neighbors["sink_u2:A"] == pytest.approx(10.0)

    def test_segment_caps_parsed(self):
        """Test that segment capacitances are parsed correctly."""
        sf = _make_spef_object()
        net = sf.nets["net_A"]
        # Should have 3 segment cap entries (indices 1, 2, 3)
        assert len(net.segment_caps) == 3
        # Entry 1: node *2 (drv_u1:Z) to ground, 0.5 pF
        assert net.segment_caps[0] == ("drv_u1:Z", None, 0.5)
        # Entry 2: node *3 (sink_u2:A) to ground, 0.5 pF
        assert net.segment_caps[1] == ("sink_u2:A", None, 0.5)
        # Entry 3: node *4 (sink_u3:B) to ground, 0.5 pF
        assert net.segment_caps[2] == ("sink_u3:B", None, 0.5)

    def test_segment_caps_mutual(self):
        """Test parsing of mutual capacitances between nodes."""
        spef = textwrap.dedent("""\
            *SPEF "IEEE 1481-1999"
            *DESIGN "test"
            *DATE "2026:01:01:00:00:00"
            *VENDOR "test"
            *PROGRAM "test"
            *VERSION "1.0"
            *DIVIDER /
            *DELIMITER :
            *BUS_DELIMITER [ ]
            *T_UNIT 1 NS
            *C_UNIT 1 PF
            *R_UNIT 1 OHM
            *L_UNIT 1 HENRY

            *NAME_MAP
            *1 net_A
            *2 drv_u1:Z
            *3 sink_u2:A

            *D_NET *1 2.0
            *CONN
            *I *2 O *C 0.0 0.0
            *I *3 I *C 0.0 0.0
            *CAP
            1 *2 0.8
            2 *3 0.8
            3 *2 *3 0.4
            *RES
            *1 *2 *3 10.0
            *END
        """)
        sf = _make_spef_object(spef)
        net = sf.nets["net_A"]
        # Should have 3 entries: 2 grounded, 1 mutual
        assert len(net.segment_caps) == 3
        # Entry 1: node to ground
        assert net.segment_caps[0] == ("drv_u1:Z", None, 0.8)
        # Entry 2: node to ground  
        assert net.segment_caps[1] == ("sink_u2:A", None, 0.8)
        # Entry 3: mutual capacitance between two nodes
        assert net.segment_caps[2] == ("drv_u1:Z", "sink_u2:A", 0.4)

    def test_units_ohm(self):

        assert _make_spef_object().r_unit == "OHM"

    def test_kohm_scaling(self):
        """*R_UNIT KOHM should multiply resistance values by 1000."""
        spef = MINIMAL_SPEF.replace("*R_UNIT 1 OHM", "*R_UNIT 1 KOHM")
        sf = _make_spef_object(spef)
        g = sf.nets["net_A"].res_graph
        neighbors = dict(g["drv_u1:Z"])
        assert neighbors["sink_u2:A"] == pytest.approx(10_000.0)

    def test_multiple_nets(self):
        extra_net = textwrap.dedent("""\
            *D_NET *10 0.25
            *CONN
            *I net_B_drv:Q O *C 0.0 0.0
            *I net_B_sink:D I *C 0.0 0.0
            *RES
            *1 net_B_drv:Q net_B_sink:D 2.0
            *END
        """)
        content = MINIMAL_SPEF + "\n" + extra_net.replace(
            "*NAME_MAP", ""
        )
        path = _write_temp_spef(content)
        try:
            sf = SpefFile(path)
            sf.parse()
        finally:
            os.unlink(path)
        assert "net_A" in sf.nets

    def test_inline_comment_stripped(self):
        """Lines with // comments should still parse the data part."""
        spef = MINIMAL_SPEF.replace(
            "*RES\n*1 *2 *3 10.0",
            "*RES\n*1 *2 *3 10.0 // $lvl=3",
        )
        sf = _make_spef_object(spef)
        g = sf.nets["net_A"].res_graph
        neighbors = dict(g["drv_u1:Z"])
        assert neighbors["sink_u2:A"] == pytest.approx(10.0)

    def test_quoted_name_in_name_map(self):
        spef = MINIMAL_SPEF.replace(
            '*1 net_A', '*1 "net/A[0]"'
        )
        sf = _make_spef_object(spef)
        assert "net/A[0]" in sf.nets

    def test_escaped_brackets_in_name_map(self):
        """Net names with \\[ \\] in NAME_MAP should be unescaped to [ ]."""
        spef = MINIMAL_SPEF.replace(
            '*1 net_A',   r'*1 bus\[1\]'
        ).replace(
            '*2 drv_u1:Z', r'*2 drv\[0\]:Z'
        )
        sf = _make_spef_object(spef)
        assert "bus[1]" in sf.nets
        assert sf.nets["bus[1]"].driver == "drv[0]:Z"

    def test_escaped_brackets_in_d_net_direct(self):
        """Net names with \\[ \\] written directly in *D_NET (no NAME_MAP) are unescaped."""
        spef = textwrap.dedent(r"""
            *SPEF "IEEE 1481-1999"
            *DESIGN "test"
            *DATE "2026:01:01:00:00:00"
            *VENDOR "test"
            *PROGRAM "test"
            *VERSION "1.0"
            *DESIGN_FLOW "NETLIST"
            *DIVIDER /
            *DELIMITER :
            *BUS_DELIMITER [ ]
            *T_UNIT 1 NS
            *C_UNIT 1 PF
            *R_UNIT 1 OHM
            *L_UNIT 1 HENRY

            *D_NET a\[1\] 1.0
            *CONN
            *I drv_Z O
            *I snk_A I
            *RES
            1 drv_Z snk_A 5.0
            *END
        """)
        sf = _make_spef_object(spef)
        assert "a[1]" in sf.nets
        net = sf.nets["a[1]"]
        assert net.driver == "drv_Z"
        assert "snk_A" in net.sinks
        assert "drv_Z" in net.res_graph

    def test_empty_file_no_crash(self):
        sf = _make_spef_object("")
        assert sf.nets == {}

    def test_unknown_direction_ignored(self):
        """A direction char other than I/O/B should not be added as driver or sink."""
        spef = MINIMAL_SPEF.replace(
            "*I *4 I *C 0.0 0.0",
            "*I *4 X *C 0.0 0.0",
        )
        sf = _make_spef_object(spef)
        net = sf.nets["net_A"]
        sinks = set(net.sinks)
        assert "sink_u3:B" not in sinks
        assert net.driver == "drv_u1:Z"   # driver unchanged


# ---------------------------------------------------------------------------
# compare_spef
# ---------------------------------------------------------------------------

class TestCompareSpef:
    def test_same_spef_correlation_is_one(self):
        """Comparing a SpefFile against itself → Pearson C corr == 1.0."""
        sf = _make_spef_object()
        # compare_spef writes to net_cap.data / net_res.data in cwd;
        # use a tmpdir to avoid polluting the workspace
        orig_cwd = os.getcwd()
        with tempfile.TemporaryDirectory() as td:
            os.chdir(td)
            try:
                caps, ress, top_c, top_r = compare_spef(sf, sf, "max")
            finally:
                os.chdir(orig_cwd)

        assert len(caps) == 1
        assert caps[0].net == "net_A"
        assert caps[0].c1 == pytest.approx(caps[0].c2)

    def test_disjoint_nets_empty_result(self):
        """Two SpefFiles with no common nets → empty cap_rows and res_rows."""
        sf1 = _make_spef_object()
        spef2_content = MINIMAL_SPEF.replace("*1 net_A", "*1 net_B")
        sf2 = _make_spef_object(spef2_content)

        orig_cwd = os.getcwd()
        with tempfile.TemporaryDirectory() as td:
            os.chdir(td)
            try:
                caps, ress, top_c, top_r = compare_spef(sf1, sf2, "max")
            finally:
                os.chdir(orig_cwd)

        assert caps == []
        assert ress == []

    def test_top10_length(self):
        """top_10_cap and top_10_res should contain at most 10 items."""
        sf = _make_spef_object()
        orig_cwd = os.getcwd()
        with tempfile.TemporaryDirectory() as td:
            os.chdir(td)
            try:
                caps, ress, top_c, top_r = compare_spef(sf, sf, "avg")
            finally:
                os.chdir(orig_cwd)
        assert len(top_c) <= 10
        assert len(top_r) <= 10


# ---------------------------------------------------------------------------
# write_caps_csv / write_res_csv
# ---------------------------------------------------------------------------

class TestWriteCsv:
    def test_write_caps_csv_roundtrip(self):
        caps = [CapComparison("net_X", 1.0, 2.0)]
        with tempfile.NamedTemporaryFile(
            suffix=".csv", mode="w", delete=False, encoding="utf-8"
        ) as f:
            path = f.name
        try:
            write_caps_csv(path, caps)
            with open(path, newline="", encoding="utf-8") as f:
                rows = list(csv.reader(f))
        finally:
            os.unlink(path)

        assert rows[0] == ["net", "C_tool1", "C_tool2",
                            "ratio_tool2_over_tool1", "delta_tool2_minus_tool1"]
        assert rows[1][0] == "net_X"
        assert float(rows[1][1]) == pytest.approx(1.0)
        assert float(rows[1][2]) == pytest.approx(2.0)
        assert float(rows[1][3]) == pytest.approx(2.0)   # ratio
        assert float(rows[1][4]) == pytest.approx(1.0)   # delta

    def test_write_caps_csv_zero_c1(self):
        """c1 == 0 → ratio should be the string 'inf'."""
        caps = [CapComparison("net_Z", 0.0, 3.0)]
        with tempfile.NamedTemporaryFile(
            suffix=".csv", mode="w", delete=False, encoding="utf-8"
        ) as f:
            path = f.name
        try:
            write_caps_csv(path, caps)
            with open(path, newline="", encoding="utf-8") as f:
                rows = list(csv.reader(f))
        finally:
            os.unlink(path)
        assert rows[1][3] == "inf"

    def test_write_res_csv_roundtrip(self):
        ress = [ResComparison("net_X", "drv:Z", "sink:A", 10.0, 15.0)]
        with tempfile.NamedTemporaryFile(
            suffix=".csv", mode="w", delete=False, encoding="utf-8"
        ) as f:
            path = f.name
        try:
            write_res_csv(path, ress, "max")
            with open(path, newline="", encoding="utf-8") as f:
                rows = list(csv.reader(f))
        finally:
            os.unlink(path)

        assert rows[0][0] == "net"
        assert rows[1][0] == "net_X"
        assert float(rows[1][1]) == pytest.approx(10.0)
        assert float(rows[1][2]) == pytest.approx(15.0)
        assert float(rows[1][3]) == pytest.approx(1.5)   # ratio 15/10
        assert float(rows[1][4]) == pytest.approx(5.0)   # delta


# ---------------------------------------------------------------------------
# collect_spef_paths
# ---------------------------------------------------------------------------

class TestCollectSpefPaths:
    def test_direct_spef_path(self):
        with tempfile.NamedTemporaryFile(suffix=".spef", delete=False) as f:
            path = f.name
        try:
            result = collect_spef_paths([path])
            assert result == [path]
        finally:
            os.unlink(path)

    def test_non_spef_file_ignored(self):
        with tempfile.NamedTemporaryFile(suffix=".txt", delete=False) as f:
            path = f.name
        try:
            result = collect_spef_paths([path])
            assert result == []
        finally:
            os.unlink(path)

    def test_directory_expansion(self):
        with tempfile.TemporaryDirectory() as td:
            p1 = os.path.join(td, "a.spef")
            p2 = os.path.join(td, "b.spef")
            p3 = os.path.join(td, "c.txt")
            for p in (p1, p2, p3):
                open(p, "w").close()
            result = collect_spef_paths([td])
            assert sorted(result) == sorted([p1, p2])

    def test_empty_input(self):
        assert collect_spef_paths([]) == []

    def test_duplicate_paths_preserved(self):
        """Same path twice should appear twice (for same-file testing)."""
        with tempfile.NamedTemporaryFile(suffix=".spef", delete=False) as f:
            path = f.name
        try:
            result = collect_spef_paths([path, path])
            assert result == [path, path]
        finally:
            os.unlink(path)


# ---------------------------------------------------------------------------
# parse_spefs_parallel
# ---------------------------------------------------------------------------

class TestParseSpefParallel:
    def test_returns_two_spef_objects(self):
        path = _write_temp_spef(MINIMAL_SPEF)
        try:
            s1, s2 = parse_spefs_parallel(path, path)
        finally:
            os.unlink(path)
        assert isinstance(s1, SpefFile)
        assert isinstance(s2, SpefFile)

    def test_both_have_same_nets(self):
        path = _write_temp_spef(MINIMAL_SPEF)
        try:
            s1, s2 = parse_spefs_parallel(path, path)
        finally:
            os.unlink(path)
        assert set(s1.nets.keys()) == set(s2.nets.keys())

    def test_parallel_same_result_as_sequential(self):
        path = _write_temp_spef(MINIMAL_SPEF)
        try:
            s_seq = SpefFile(path)
            s_seq.parse()
            s1, _ = parse_spefs_parallel(path, path)
        finally:
            os.unlink(path)
        assert set(s1.nets.keys()) == set(s_seq.nets.keys())
        for net_name, net_seq in s_seq.nets.items():
            net_par = s1.nets[net_name]
            assert net_par.total_cap == pytest.approx(net_seq.total_cap)
            assert net_par.driver == net_seq.driver
            assert sorted(net_par.sinks) == sorted(net_seq.sinks)


# ---------------------------------------------------------------------------
# parse_net_cap_data / parse_net_res_data
# ---------------------------------------------------------------------------

class TestParseNetCapData:
    def _write_cap_data(self, content: str) -> str:
        f = tempfile.NamedTemporaryFile(
            suffix=".data", mode="w", delete=False, encoding="utf-8"
        )
        f.write(content)
        f.close()
        return f.name

    def test_basic_parse(self):
        content = "net_A 1.5 2.0\nnet_B 0.5 0.7\n"
        path = self._write_cap_data(content)
        try:
            caps = parse_net_cap_data(path)
        finally:
            os.unlink(path)
        assert len(caps) == 2
        assert caps[0].net == "net_A"
        assert caps[0].c1 == pytest.approx(1.5)
        assert caps[0].c2 == pytest.approx(2.0)
        assert caps[1].net == "net_B"

    def test_comments_and_blank_lines_skipped(self):
        content = "# this is a comment\n\nnet_A 1.0 2.0\n\n"
        path = self._write_cap_data(content)
        try:
            caps = parse_net_cap_data(path)
        finally:
            os.unlink(path)
        assert len(caps) == 1
        assert caps[0].net == "net_A"

    def test_insufficient_fields_skipped(self):
        content = "net_A 1.0\nnet_B 0.5 0.7\n"
        path = self._write_cap_data(content)
        try:
            caps = parse_net_cap_data(path)
        finally:
            os.unlink(path)
        assert len(caps) == 1
        assert caps[0].net == "net_B"

    def test_non_numeric_skipped(self):
        content = "net_A abc 2.0\nnet_B 0.5 0.7\n"
        path = self._write_cap_data(content)
        try:
            caps = parse_net_cap_data(path)
        finally:
            os.unlink(path)
        assert len(caps) == 1
        assert caps[0].net == "net_B"

    def test_empty_file(self):
        path = self._write_cap_data("")
        try:
            caps = parse_net_cap_data(path)
        finally:
            os.unlink(path)
        assert caps == []

    def test_correlation_from_cap_data(self):
        """Data parsed from file should yield perfect correlation when c1==c2."""
        content = "net_A 1.0 1.0\nnet_B 2.0 2.0\nnet_C 3.0 3.0\n"
        path = self._write_cap_data(content)
        try:
            caps = parse_net_cap_data(path)
        finally:
            os.unlink(path)
        corr = pearson_corr([c.c1 for c in caps], [c.c2 for c in caps])
        assert corr == pytest.approx(1.0)


class TestParseNetResData:
    def _write_res_data(self, content: str) -> str:
        f = tempfile.NamedTemporaryFile(
            suffix=".data", mode="w", delete=False, encoding="utf-8"
        )
        f.write(content)
        f.close()
        return f.name

    def test_basic_parse(self):
        content = "net_A drv:Z sink:A 10.0 15.0\nnet_A drv:Z sink:B 5.0 8.0\n"
        path = self._write_res_data(content)
        try:
            ress = parse_net_res_data(path)
        finally:
            os.unlink(path)
        assert len(ress) == 2
        assert ress[0].net == "net_A"
        assert ress[0].driver == "drv:Z"
        assert ress[0].load == "sink:A"
        assert ress[0].r1 == pytest.approx(10.0)
        assert ress[0].r2 == pytest.approx(15.0)

    def test_comments_and_blank_lines_skipped(self):
        content = "# comment\n\nnet_A drv:Z sink:A 10.0 15.0\n"
        path = self._write_res_data(content)
        try:
            ress = parse_net_res_data(path)
        finally:
            os.unlink(path)
        assert len(ress) == 1

    def test_insufficient_fields_skipped(self):
        content = "net_A drv:Z sink:A 10.0\nnet_B drv:Q sink:D 5.0 8.0\n"
        path = self._write_res_data(content)
        try:
            ress = parse_net_res_data(path)
        finally:
            os.unlink(path)
        assert len(ress) == 1
        assert ress[0].net == "net_B"

    def test_non_numeric_skipped(self):
        content = "net_A drv:Z sink:A abc 15.0\nnet_B drv:Q sink:D 5.0 8.0\n"
        path = self._write_res_data(content)
        try:
            ress = parse_net_res_data(path)
        finally:
            os.unlink(path)
        assert len(ress) == 1
        assert ress[0].net == "net_B"

    def test_empty_file(self):
        path = self._write_res_data("")
        try:
            ress = parse_net_res_data(path)
        finally:
            os.unlink(path)
        assert ress == []

    def test_correlation_from_res_data(self):
        """Data parsed from file should yield perfect correlation when r1==r2."""
        content = "net_A drv:Z sink:A 10.0 10.0\nnet_B drv:Q sink:D 5.0 5.0\nnet_C drv:X sink:Y 20.0 20.0\n"
        path = self._write_res_data(content)
        try:
            ress = parse_net_res_data(path)
        finally:
            os.unlink(path)
        corr = pearson_corr([r.r1 for r in ress], [r.r2 for r in ress])
        assert corr == pytest.approx(1.0)


# ---------------------------------------------------------------------------
# GUI CLI wiring (lightweight, no real Tk window)
# ---------------------------------------------------------------------------

class TestGuiCliWiring:
    def test_main_gui_forwards_two_explicit_spefs_and_auto_run(self, monkeypatch):
        path1 = _write_temp_spef(MINIMAL_SPEF)
        path2 = _write_temp_spef(MINIMAL_SPEF.replace("*1 net_A", "*1 net_B"))
        calls = []

        def fake_launch_gui(preload_paths=None, auto_run=False):
            calls.append((list(preload_paths or []), auto_run))

        monkeypatch.setattr(spef_mod, "launch_gui", fake_launch_gui)
        monkeypatch.setattr(
            sys,
            "argv",
            [
                "spef_rc_correlation.py",
                path1,
                path2,
                "--gui",
                "--gui-auto-run",
            ],
        )

        try:
            spef_mod.main()
        finally:
            os.unlink(path1)
            os.unlink(path2)

        assert calls == [([os.path.abspath(path1), os.path.abspath(path2)], True)]

    def test_main_gui_directory_expands_spef_files(self, monkeypatch):
        calls = []

        def fake_launch_gui(preload_paths=None, auto_run=False):
            calls.append((list(preload_paths or []), auto_run))

        monkeypatch.setattr(spef_mod, "launch_gui", fake_launch_gui)

        with tempfile.TemporaryDirectory() as td:
            a_path = os.path.join(td, "a.spef")
            b_path = os.path.join(td, "b.spef")
            txt_path = os.path.join(td, "notes.txt")
            with open(a_path, "w", encoding="utf-8") as f:
                f.write(MINIMAL_SPEF)
            with open(b_path, "w", encoding="utf-8") as f:
                f.write(MINIMAL_SPEF)
            with open(txt_path, "w", encoding="utf-8") as f:
                f.write("ignore")

            monkeypatch.setattr(sys, "argv", ["spef_rc_correlation.py", td, "--gui"])
            spef_mod.main()

        assert calls == [([os.path.abspath(a_path), os.path.abspath(b_path)], False)]

    def test_main_gui_preserves_duplicate_same_file_inputs(self, monkeypatch):
        path = _write_temp_spef(MINIMAL_SPEF)
        calls = []

        def fake_launch_gui(preload_paths=None, auto_run=False):
            calls.append((list(preload_paths or []), auto_run))

        monkeypatch.setattr(spef_mod, "launch_gui", fake_launch_gui)
        monkeypatch.setattr(
            sys,
            "argv",
            ["spef_rc_correlation.py", path, path, "--gui", "--gui-auto-run"],
        )

        try:
            spef_mod.main()
        finally:
            os.unlink(path)

        assert calls == [([os.path.abspath(path), os.path.abspath(path)], True)]

    def test_main_without_args_launches_gui_with_empty_preload(self, monkeypatch):
        calls = []

        def fake_launch_gui(preload_paths=None, auto_run=False, **kwargs):
            calls.append((list(preload_paths or []), auto_run))

        monkeypatch.setattr(spef_mod, "launch_gui", fake_launch_gui)
        monkeypatch.setattr(sys, "argv", ["spef_rc_correlation.py"])

        spef_mod.main()

        assert calls == [([], False)]

    # ---- data-file + GUI wiring ----

    def test_main_data_gui_forwards_cap_data(self, monkeypatch):
        """--net-cap-data with --gui should call launch_gui with preload_cap_data."""
        calls = []

        def fake_launch_gui(preload_paths=None, auto_run=False,
                            preload_cap_data=None, preload_res_data=None):
            calls.append({
                "preload_paths": list(preload_paths or []),
                "auto_run": auto_run,
                "preload_cap_data": preload_cap_data,
                "preload_res_data": preload_res_data,
            })

        monkeypatch.setattr(spef_mod, "launch_gui", fake_launch_gui)

        with tempfile.NamedTemporaryFile(suffix=".data", mode="w",
                                         delete=False, encoding="utf-8") as f:
            f.write("net_A 1.0 2.0\n")
            cap_path = f.name

        try:
            monkeypatch.setattr(sys, "argv", [
                "spef_rc_correlation.py",
                "--net-cap-data", cap_path,
                "--gui",
            ])
            spef_mod.main()
        finally:
            os.unlink(cap_path)

        assert len(calls) == 1
        assert calls[0]["preload_cap_data"] == cap_path
        assert calls[0]["preload_res_data"] is None
        assert calls[0]["auto_run"] is False

    def test_main_data_gui_forwards_res_data(self, monkeypatch):
        """--net-res-data with --gui should call launch_gui with preload_res_data."""
        calls = []

        def fake_launch_gui(preload_paths=None, auto_run=False,
                            preload_cap_data=None, preload_res_data=None):
            calls.append({
                "preload_paths": list(preload_paths or []),
                "auto_run": auto_run,
                "preload_cap_data": preload_cap_data,
                "preload_res_data": preload_res_data,
            })

        monkeypatch.setattr(spef_mod, "launch_gui", fake_launch_gui)

        with tempfile.NamedTemporaryFile(suffix=".data", mode="w",
                                         delete=False, encoding="utf-8") as f:
            f.write("net_A drv:Z sink:A 10.0 15.0\n")
            res_path = f.name

        try:
            monkeypatch.setattr(sys, "argv", [
                "spef_rc_correlation.py",
                "--net-res-data", res_path,
                "--gui",
            ])
            spef_mod.main()
        finally:
            os.unlink(res_path)

        assert len(calls) == 1
        assert calls[0]["preload_cap_data"] is None
        assert calls[0]["preload_res_data"] == res_path

    def test_main_data_gui_both_files_with_auto_run(self, monkeypatch):
        """Both data files + --gui --gui-auto-run passes all args correctly."""
        calls = []

        def fake_launch_gui(preload_paths=None, auto_run=False,
                            preload_cap_data=None, preload_res_data=None):
            calls.append({
                "preload_paths": list(preload_paths or []),
                "auto_run": auto_run,
                "preload_cap_data": preload_cap_data,
                "preload_res_data": preload_res_data,
            })

        monkeypatch.setattr(spef_mod, "launch_gui", fake_launch_gui)

        with tempfile.NamedTemporaryFile(suffix=".data", mode="w",
                                         delete=False, encoding="utf-8") as fc:
            fc.write("net_A 1.0 2.0\n")
            cap_path = fc.name
        with tempfile.NamedTemporaryFile(suffix=".data", mode="w",
                                         delete=False, encoding="utf-8") as fr:
            fr.write("net_A drv:Z sink:A 10.0 15.0\n")
            res_path = fr.name

        try:
            monkeypatch.setattr(sys, "argv", [
                "spef_rc_correlation.py",
                "--net-cap-data", cap_path,
                "--net-res-data", res_path,
                "--gui",
                "--gui-auto-run",
            ])
            spef_mod.main()
        finally:
            os.unlink(cap_path)
            os.unlink(res_path)

        assert len(calls) == 1
        assert calls[0]["preload_cap_data"] == cap_path
        assert calls[0]["preload_res_data"] == res_path
        assert calls[0]["auto_run"] is True

    def test_main_data_no_gui_still_does_cli(self, monkeypatch, capsys):
        """--net-cap-data without --gui should NOT call launch_gui (CLI mode)."""
        calls = []

        def fake_launch_gui(**kwargs):
            calls.append(kwargs)

        monkeypatch.setattr(spef_mod, "launch_gui", fake_launch_gui)

        with tempfile.NamedTemporaryFile(suffix=".data", mode="w",
                                         delete=False, encoding="utf-8") as f:
            f.write("net_A 1.0 2.0\nnet_B 0.5 0.8\n")
            cap_path = f.name

        try:
            monkeypatch.setattr(sys, "argv", [
                "spef_rc_correlation.py",
                "--net-cap-data", cap_path,
            ])
            spef_mod.main()
        finally:
            os.unlink(cap_path)

        # GUI must NOT have been launched
        assert calls == []
        # CLI output should mention the correlation
        out = capsys.readouterr().out
        assert "correlation" in out.lower() or "cap" in out.lower()
# ---------------------------------------------------------------------------
# shuffle_net_mapping
# ---------------------------------------------------------------------------

# A two-net SPEF used across the shuffle tests.  Uses plain-integer indices
# in *CAP and *RES sections (standard commercial format).
TWO_NET_SPEF = textwrap.dedent("""\
    *SPEF "IEEE 1481-1999"
    *DESIGN "test2"
    *DATE "2026:01:01:00:00:00"
    *VENDOR "test"
    *PROGRAM "test"
    *VERSION "1.0"
    *DESIGN_FLOW "NETLIST"
    *DIVIDER /
    *DELIMITER :
    *BUS_DELIMITER [ ]
    *T_UNIT 1 NS
    *C_UNIT 1 PF
    *R_UNIT 1 OHM
    *L_UNIT 1 HENRY

    *NAME_MAP
    *1 net_A
    *2 net_B
    *3 drv_u1:Z
    *4 sink_u2:A
    *5 drv_u3:Y
    *6 sink_u4:B

    *D_NET *1 1.5
    *CONN
    *I *3 O *C 0.0 0.0
    *I *4 I *C 0.0 0.0
    *CAP
    1 *3 0.8
    2 *4 0.7
    *RES
    1 *3 *4 10.0
    *END

    *D_NET *2 0.8
    *CONN
    *I *5 O *C 0.0 0.0
    *I *6 I *C 0.0 0.0
    *CAP
    1 *5 0.4
    2 *6 0.4
    *RES
    1 *5 *6 5.0
    *END
""")


def _parse_spef_from_string(content: str) -> SpefFile:
    """Write *content* to a temp file, parse it, delete it, and return the SpefFile."""
    path = _write_temp_spef(content)
    try:
        sf = SpefFile(path)
        sf.parse()
    finally:
        os.unlink(path)
    return sf


class TestShuffleNetMapping:
    # ------------------------------------------------------------------ helpers

    def _shuffle_tmp(self, content: str, seed: int = 0) -> str:
        """Write *content* to a temp SPEF, shuffle it, return path of shuffled file."""
        src = _write_temp_spef(content)
        dst = src + ".shuffled.spef"
        try:
            shuffle_net_mapping(src, dst, seed=seed)
        finally:
            os.unlink(src)
        return dst  # caller must unlink

    # ------------------------------------------------------------------ core correctness

    def test_rc_data_preserved_per_net_name(self):
        """After shuffling, each net_name has the same total_cap and resistance."""
        orig_sf = _parse_spef_from_string(TWO_NET_SPEF)

        shuffled_path = self._shuffle_tmp(TWO_NET_SPEF, seed=0)
        try:
            shuffled_sf = SpefFile(shuffled_path)
            shuffled_sf.parse()
        finally:
            os.unlink(shuffled_path)

        for net_name, orig_net in orig_sf.nets.items():
            shuf_net = shuffled_sf.nets.get(net_name)
            assert shuf_net is not None, f"{net_name} missing from shuffled SPEF"
            assert shuf_net.total_cap == pytest.approx(orig_net.total_cap), (
                f"{net_name}: cap changed"
            )
            orig_r = orig_net.driver_sink_resistances()
            shuf_r = shuf_net.driver_sink_resistances()
            for sink, r_val in orig_r.items():
                assert sink in shuf_r, f"{net_name}: sink {sink} missing after shuffle"
                assert shuf_r[sink] == pytest.approx(r_val), (
                    f"{net_name}/{sink}: resistance changed"
                )

    def test_compare_spef_produces_same_data(self):
        """compare_spef(original, shuffled) → c1==c2 and r1==r2 for every entry."""
        orig_path = _write_temp_spef(TWO_NET_SPEF)
        shuffled_path = orig_path + ".shuffled.spef"
        try:
            shuffle_net_mapping(orig_path, shuffled_path, seed=1)
            orig_sf = SpefFile(orig_path)
            orig_sf.parse()
            shuf_sf = SpefFile(shuffled_path)
            shuf_sf.parse()

            with tempfile.TemporaryDirectory() as td:
                os.chdir(td)
                try:
                    caps, ress, _, _ = compare_spef(orig_sf, shuf_sf, "max")
                finally:
                    os.chdir(os.path.dirname(orig_path))

            assert len(caps) == len(orig_sf.nets), "All nets must be in common"
            for c in caps:
                assert c.c1 == pytest.approx(c.c2), (
                    f"Cap mismatch for {c.net}: {c.c1} vs {c.c2}"
                )
            for r in ress:
                assert r.r1 == pytest.approx(r.r2), (
                    f"Res mismatch for {r.net}/{r.load}: {r.r1} vs {r.r2}"
                )
        finally:
            os.unlink(orig_path)
            if os.path.exists(shuffled_path):
                os.unlink(shuffled_path)

    def test_name_map_is_actually_shuffled(self):
        """The net_id → net_name entries in NAME_MAP must differ from the original."""
        orig_sf = _parse_spef_from_string(TWO_NET_SPEF)
        orig_mapping = {v: k for k, v in orig_sf.name_map.items()}  # name→id in original

        shuffled_path = self._shuffle_tmp(TWO_NET_SPEF, seed=0)
        try:
            shuf_sf = SpefFile(shuffled_path)
            shuf_sf.parse()
        finally:
            os.unlink(shuffled_path)

        new_mapping = {v: k for k, v in shuf_sf.name_map.items()}   # name→id in shuffled

        # At least one net must have a different ID
        net_names = list(orig_sf.nets.keys())
        assert any(
            orig_mapping.get(n) != new_mapping.get(n) for n in net_names
        ), "Expected at least one net to have a different ID after shuffling"

    def test_same_net_names_present(self):
        """The shuffled SPEF must contain exactly the same net names as the original."""
        orig_sf = _parse_spef_from_string(TWO_NET_SPEF)

        shuffled_path = self._shuffle_tmp(TWO_NET_SPEF, seed=2)
        try:
            shuf_sf = SpefFile(shuffled_path)
            shuf_sf.parse()
        finally:
            os.unlink(shuffled_path)

        assert set(shuf_sf.nets.keys()) == set(orig_sf.nets.keys())

    # ------------------------------------------------------------------ seed reproducibility

    def test_seed_gives_reproducible_output(self):
        """Two calls with the same seed must produce byte-for-byte identical output."""
        src = _write_temp_spef(TWO_NET_SPEF)
        dst1 = src + ".s1.spef"
        dst2 = src + ".s2.spef"
        try:
            shuffle_net_mapping(src, dst1, seed=99)
            shuffle_net_mapping(src, dst2, seed=99)
            with open(dst1, 'r', encoding='utf-8') as f1, \
                 open(dst2, 'r', encoding='utf-8') as f2:
                assert f1.read() == f2.read()
        finally:
            os.unlink(src)
            for p in (dst1, dst2):
                if os.path.exists(p):
                    os.unlink(p)

    def test_different_seeds_may_differ(self):
        """Two different seeds should (with overwhelming probability) differ."""
        # With only 2 nets there are only 2 permutations and both seeds might
        # land on the same one, so we use a larger synthetic SPEF here.
        nets = []
        name_map_lines = []
        for i in range(1, 11):
            net_id = f"*{i}"
            pin_id = f"*{i + 100}"
            name_map_lines.append(f"{net_id} net_{i:02d}")
            name_map_lines.append(f"{pin_id} drv_{i:02d}:Z")
            nets.append(
                f"*D_NET {net_id} {i * 0.1:.1f}\n"
                f"*CONN\n"
                f"*I {pin_id} O *C 0.0 0.0\n"
                f"*RES\n"
                f"1 {pin_id} {pin_id} {i * 1.0:.1f}\n"
                f"*END"
            )
        header = textwrap.dedent("""\
            *SPEF "IEEE 1481-1999"
            *DESIGN "big"
            *DATE "2026:01:01:00:00:00"
            *VENDOR "t"
            *PROGRAM "t"
            *VERSION "1.0"
            *DIVIDER /
            *DELIMITER :
            *BUS_DELIMITER [ ]
            *T_UNIT 1 NS
            *C_UNIT 1 PF
            *R_UNIT 1 OHM
            *L_UNIT 1 HENRY
        """)
        content = header + "\n*NAME_MAP\n" + "\n".join(name_map_lines) + "\n\n" + "\n\n".join(nets) + "\n"

        src = _write_temp_spef(content)
        dst_a = src + ".a.spef"
        dst_b = src + ".b.spef"
        try:
            shuffle_net_mapping(src, dst_a, seed=1)
            shuffle_net_mapping(src, dst_b, seed=2)
            with open(dst_a) as fa, open(dst_b) as fb:
                assert fa.read() != fb.read(), "Different seeds should produce different results"
        finally:
            os.unlink(src)
            for p in (dst_a, dst_b):
                if os.path.exists(p):
                    os.unlink(p)

    # ------------------------------------------------------------------ edge cases

    def test_single_net_file_copied_unchanged(self):
        """A SPEF with only one net (nothing to shuffle) is copied verbatim."""
        src = _write_temp_spef(MINIMAL_SPEF)
        dst = src + ".shuffled.spef"
        try:
            shuffle_net_mapping(src, dst, seed=0)
            with open(src, 'r', encoding='utf-8') as f1, \
                 open(dst, 'r', encoding='utf-8') as f2:
                assert f1.read() == f2.read()
        finally:
            os.unlink(src)
            if os.path.exists(dst):
                os.unlink(dst)

    def test_no_name_map_file_copied_unchanged(self):
        """A SPEF with no *NAME_MAP / literal net names is copied verbatim."""
        content = textwrap.dedent("""\
            *SPEF "IEEE 1481-1999"
            *DESIGN "t"
            *DATE "2026:01:01:00:00:00"
            *VENDOR "t"
            *PROGRAM "t"
            *VERSION "1.0"
            *DIVIDER /
            *DELIMITER :
            *BUS_DELIMITER [ ]
            *T_UNIT 1 NS
            *C_UNIT 1 PF
            *R_UNIT 1 OHM
            *L_UNIT 1 HENRY

            *D_NET net_X 1.0
            *CONN
            *I drv_Z O
            *RES
            1 drv_Z snk_A 5.0
            *END
        """)
        src = _write_temp_spef(content)
        dst = src + ".shuffled.spef"
        try:
            shuffle_net_mapping(src, dst, seed=0)
            with open(src, 'r', encoding='utf-8') as f1, \
                 open(dst, 'r', encoding='utf-8') as f2:
                assert f1.read() == f2.read()
        finally:
            os.unlink(src)
            if os.path.exists(dst):
                os.unlink(dst)

    def test_pin_ids_are_not_substituted(self):
        """Instance/pin NAME_MAP entries (*3, *4, *5, *6) must remain unchanged."""
        shuffled_path = self._shuffle_tmp(TWO_NET_SPEF, seed=0)
        try:
            shuf_sf = SpefFile(shuffled_path)
            shuf_sf.parse()
        finally:
            os.unlink(shuffled_path)

        # Pin/instance entries *3, *4, *5, *6 must still map to the same names
        assert shuf_sf.name_map.get("*3") == "drv_u1:Z"
        assert shuf_sf.name_map.get("*4") == "sink_u2:A"
        assert shuf_sf.name_map.get("*5") == "drv_u3:Y"
        assert shuf_sf.name_map.get("*6") == "sink_u4:B"

    def test_driver_and_sinks_unchanged(self):
        """Driver and sink pin names per net must be identical after shuffling."""
        orig_sf = _parse_spef_from_string(TWO_NET_SPEF)

        shuffled_path = self._shuffle_tmp(TWO_NET_SPEF, seed=3)
        try:
            shuf_sf = SpefFile(shuffled_path)
            shuf_sf.parse()
        finally:
            os.unlink(shuffled_path)

        for net_name in orig_sf.nets:
            orig_net = orig_sf.nets[net_name]
            shuf_net = shuf_sf.nets[net_name]
            assert shuf_net.driver == orig_net.driver, (
                f"{net_name}: driver changed from {orig_net.driver} to {shuf_net.driver}"
            )
            assert sorted(shuf_net.sinks) == sorted(orig_net.sinks), (
                f"{net_name}: sinks changed"
            )

    # ------------------------------------------------------------------ CLI wiring

    def test_cli_shuffle_creates_output_file(self, monkeypatch, capsys):
        """--shuffle should create a shuffled SPEF and print a summary."""
        src = _write_temp_spef(TWO_NET_SPEF)
        dst = src + ".out.spef"
        try:
            monkeypatch.setattr(sys, "argv", [
                "spef_rc_correlation.py",
                src,
                "--shuffle",
                "--output", dst,
                "--seed", "7",
            ])
            spef_mod.main()
            assert os.path.exists(dst), "Output file was not created"
            out = capsys.readouterr().out
            assert "shuffle" in out.lower()
        finally:
            os.unlink(src)
            if os.path.exists(dst):
                os.unlink(dst)

    def test_cli_shuffle_default_output_name(self, monkeypatch):
        """Without --output, shuffled SPEF is written to <input>_shuffled.spef."""
        src = _write_temp_spef(TWO_NET_SPEF)
        base, ext = os.path.splitext(src)
        dst = f"{base}_shuffled{ext}"
        try:
            monkeypatch.setattr(sys, "argv", [
                "spef_rc_correlation.py",
                src,
                "--shuffle",
                "--seed", "0",
            ])
            spef_mod.main()
            assert os.path.exists(dst)
        finally:
            os.unlink(src)
            if os.path.exists(dst):
                os.unlink(dst)


# ---------------------------------------------------------------------------
# backmark_spef
# ---------------------------------------------------------------------------

# A two-net SPEF with inline comments on both *CAP and *RES lines.
_BACKMARK_SPEF = textwrap.dedent("""\
    *SPEF "IEEE 1481-1999"
    *DESIGN "bm"
    *DATE "2026:01:01:00:00:00"
    *VENDOR "test"
    *PROGRAM "test"
    *VERSION "1.0"
    *DESIGN_FLOW "NETLIST"
    *DIVIDER /
    *DELIMITER :
    *BUS_DELIMITER [ ]
    *T_UNIT 1 NS
    *C_UNIT 1 PF
    *R_UNIT 1 OHM
    *L_UNIT 1 HENRY

    *NAME_MAP
    *1 net_A
    *2 net_B
    *3 drv_u1:Z
    *4 sink_u2:A
    *5 drv_u3:Y
    *6 sink_u4:B

    *D_NET *1 1.5
    *CONN
    *I *3 O *C 0.0 0.0
    *I *4 I *C 0.0 0.0
    *CAP
    1 *3 0.8
    2 *4 0.7
    *RES
    1 *3 *4 10.0
    *END

    *D_NET *2 0.8
    *CONN
    *I *5 O *C 0.0 0.0
    *I *6 I *C 0.0 0.0
    *CAP
    1 *5 0.4
    2 *6 0.4
    *RES
    1 *5 *6 5.0
    *END
""")

# Same design with doubled RC values (simulates a reference tool).
_REF_SPEF = textwrap.dedent("""\
    *SPEF "IEEE 1481-1999"
    *DESIGN "bm"
    *DATE "2026:01:01:00:00:00"
    *VENDOR "ref"
    *PROGRAM "ref"
    *VERSION "1.0"
    *DESIGN_FLOW "NETLIST"
    *DIVIDER /
    *DELIMITER :
    *BUS_DELIMITER [ ]
    *T_UNIT 1 NS
    *C_UNIT 1 PF
    *R_UNIT 1 OHM
    *L_UNIT 1 HENRY

    *NAME_MAP
    *1 net_A
    *2 net_B
    *3 drv_u1:Z
    *4 sink_u2:A
    *5 drv_u3:Y
    *6 sink_u4:B

    *D_NET *1 3.0
    *CONN
    *I *3 O *C 0.0 0.0
    *I *4 I *C 0.0 0.0
    *CAP
    1 *3 1.5
    2 *4 1.5
    *RES
    1 *3 *4 20.0
    *END

    *D_NET *2 1.6
    *CONN
    *I *5 O *C 0.0 0.0
    *I *6 I *C 0.0 0.0
    *CAP
    1 *5 0.8
    2 *6 0.8
    *RES
    1 *5 *6 10.0
    *END
""")


class TestBackmarkSpef:
    """Tests for backmark_spef and its helper parsers."""

    # ------------------------------------------------------------------ helpers

    def _write(self, td: str, name: str, content: str) -> str:
        path = os.path.join(td, name)
        with open(path, "w", encoding="utf-8") as f:
            f.write(content)
        return path

    def _parse(self, path: str) -> SpefFile:
        sf = SpefFile(path)
        sf.parse()
        return sf

    # ------------------------------------------------------------------ basic cap update

    def test_basic_cap_update(self):
        """*D_NET total cap and *CAP segment values are scaled correctly."""
        with tempfile.TemporaryDirectory() as td:
            spef = self._write(td, "orig.spef", _BACKMARK_SPEF)
            cap_data = self._write(td, "cap.data", "net_A 1.5 3.0\n")
            out = os.path.join(td, "out.spef")

            backmark_spef(spef, cap_data, None, out)

            sf = self._parse(out)
            net = sf.nets["net_A"]
            assert net.total_cap == pytest.approx(3.0)
            # Individual cap segments scaled by factor 2 (3.0/1.5)
            caps = {node: c for node, _, c in net.segment_caps}
            assert caps["drv_u1:Z"] == pytest.approx(0.8 * 2)
            assert caps["sink_u2:A"] == pytest.approx(0.7 * 2)
            # net_B unchanged
            assert sf.nets["net_B"].total_cap == pytest.approx(0.8)

    # ------------------------------------------------------------------ basic res update

    def test_basic_res_update(self):
        """*RES segment values are scaled uniformly by the average sink ratio."""
        with tempfile.TemporaryDirectory() as td:
            spef = self._write(td, "orig.spef", _BACKMARK_SPEF)
            res_data = self._write(
                td, "res.data", "net_A drv_u1:Z sink_u2:A 10.0 20.0\n"
            )
            out = os.path.join(td, "out.spef")

            backmark_spef(spef, None, res_data, out)

            sf = self._parse(out)
            net = sf.nets["net_A"]
            r = net.driver_sink_resistances()
            assert r["sink_u2:A"] == pytest.approx(20.0)
            # net_B unchanged
            assert sf.nets["net_B"].driver_sink_resistances()["sink_u4:B"] == pytest.approx(5.0)

    # ------------------------------------------------------------------ inline comment bug fix

    def test_cap_inline_comment_preserved_and_scaled(self):
        """*CAP lines with inline comments must still have their values scaled.

        This is a regression test for a bug where backmark_spef used parts[-1]
        to find the cap value, causing the comment text to be parsed instead of
        the numeric value, and silently leaving segment caps unscaled.
        """
        spef_content = textwrap.dedent("""\
            *SPEF "IEEE 1481-1999"
            *DESIGN "t"
            *DATE "2026:01:01:00:00:00"
            *VENDOR "t"
            *PROGRAM "t"
            *VERSION "1.0"
            *DIVIDER /
            *DELIMITER :
            *BUS_DELIMITER [ ]
            *T_UNIT 1 NS
            *C_UNIT 1 PF
            *R_UNIT 1 OHM
            *L_UNIT 1 HENRY

            *NAME_MAP
            *1 net_X
            *2 drv:Z
            *3 snk:A

            *D_NET *1 1.0
            *CONN
            *I *2 O *C 0.0 0.0
            *I *3 I *C 0.0 0.0
            *CAP
            1 *2 0.6 // ground cap at driver
            2 *3 0.4 // ground cap at sink
            *RES
            1 *2 *3 5.0 // M3
            *END
        """)
        with tempfile.TemporaryDirectory() as td:
            spef = self._write(td, "spef.spef", spef_content)
            # Double all values
            cap_data = self._write(td, "cap.data", "net_X 1.0 2.0\n")
            res_data = self._write(td, "res.data", "net_X drv:Z snk:A 5.0 10.0\n")
            out = os.path.join(td, "out.spef")

            backmark_spef(spef, cap_data, res_data, out)

            sf = self._parse(out)
            net = sf.nets["net_X"]

            # Total cap updated
            assert net.total_cap == pytest.approx(2.0)
            # Segment caps scaled by 2 (bug fix: comments must not interfere)
            caps = {node: c for node, _, c in net.segment_caps}
            assert caps["drv:Z"] == pytest.approx(1.2), (
                "Segment cap at driver not scaled — inline comment bug not fixed"
            )
            assert caps["snk:A"] == pytest.approx(0.8), (
                "Segment cap at sink not scaled — inline comment bug not fixed"
            )
            # Resistance scaled by 2
            r = net.driver_sink_resistances()
            assert r["snk:A"] == pytest.approx(10.0)

            # Inline comment text must be preserved in the output file
            with open(out, encoding="utf-8") as f:
                out_text = f.read()
            assert "// ground cap at driver" in out_text
            assert "// ground cap at sink" in out_text

    def test_cap_mutual_inline_comment_scaled(self):
        """Mutual *CAP entries (idx node1 node2 value // comment) are scaled correctly."""
        spef_content = textwrap.dedent("""\
            *SPEF "IEEE 1481-1999"
            *DESIGN "t"
            *DATE "2026:01:01:00:00:00"
            *VENDOR "t"
            *PROGRAM "t"
            *VERSION "1.0"
            *DIVIDER /
            *DELIMITER :
            *BUS_DELIMITER [ ]
            *T_UNIT 1 NS
            *C_UNIT 1 PF
            *R_UNIT 1 OHM
            *L_UNIT 1 HENRY

            *NAME_MAP
            *1 net_Y
            *2 drv:Z
            *3 snk:A

            *D_NET *1 1.0
            *CONN
            *I *2 O *C 0.0 0.0
            *I *3 I *C 0.0 0.0
            *CAP
            1 *2 *3 1.0 // mutual coupling
            *RES
            1 *2 *3 5.0
            *END
        """)
        with tempfile.TemporaryDirectory() as td:
            spef = self._write(td, "spef.spef", spef_content)
            cap_data = self._write(td, "cap.data", "net_Y 1.0 2.0\n")
            out = os.path.join(td, "out.spef")

            backmark_spef(spef, cap_data, None, out)

            sf = self._parse(out)
            net = sf.nets["net_Y"]
            assert net.total_cap == pytest.approx(2.0)
            # Mutual cap entry scaled by 2
            assert net.segment_caps[0][2] == pytest.approx(2.0), (
                "Mutual cap not scaled — inline comment bug not fixed"
            )

    # ------------------------------------------------------------------ round-trip with shuffled SPEF

    def test_backmark_from_shuffled_spef_data(self):
        """Data files generated from a shuffled SPEF can be applied to the original.

        Workflow:
          1. Generate a shuffled copy of the original SPEF.
          2. Compare the shuffled SPEF against a reference SPEF → net_cap.data,
             net_res.data (written to a temp dir by compare_spef).
          3. Apply those data files to the *original* (unshuffled) SPEF.
          4. The back-annotated SPEF must have the reference RC values for every net.
        """
        with tempfile.TemporaryDirectory() as td:
            orig_path = self._write(td, "orig.spef", _BACKMARK_SPEF)
            ref_path  = self._write(td, "ref.spef",  _REF_SPEF)
            shuf_path = os.path.join(td, "shuffled.spef")
            out_path  = os.path.join(td, "backmarked.spef")

            # Step 1: shuffle
            shuffle_net_mapping(orig_path, shuf_path, seed=7)

            # Step 2: compare shuffled vs reference
            shuf_sf = SpefFile(shuf_path); shuf_sf.parse()
            ref_sf  = SpefFile(ref_path);  ref_sf.parse()

            old_dir = os.getcwd()
            os.chdir(td)
            try:
                compare_spef(shuf_sf, ref_sf, "max")
            finally:
                os.chdir(old_dir)

            cap_data = os.path.join(td, "net_cap.data")
            res_data = os.path.join(td, "net_res.data")

            # Step 3: back-annotate the *original* SPEF
            backmark_spef(orig_path, cap_data, res_data, out_path)

            # Step 4: verify the output matches the reference values
            out_sf = self._parse(out_path)
            for net_name, ref_net in ref_sf.nets.items():
                out_net = out_sf.nets.get(net_name)
                assert out_net is not None, f"{net_name} missing from back-annotated SPEF"
                assert out_net.total_cap == pytest.approx(ref_net.total_cap, rel=1e-5), (
                    f"{net_name}: total_cap {out_net.total_cap} != ref {ref_net.total_cap}"
                )
                ref_r = ref_net.driver_sink_resistances()
                out_r = out_net.driver_sink_resistances()
                for sink, r_ref in ref_r.items():
                    assert sink in out_r, f"{net_name}/{sink} missing from output"
                    assert out_r[sink] == pytest.approx(r_ref, rel=1e-5), (
                        f"{net_name}/{sink}: R {out_r[sink]} != ref {r_ref}"
                    )

    # ------------------------------------------------------------------ edge cases

    def test_net_absent_from_data_file_unchanged(self):
        """Nets not mentioned in the data files must be written unchanged."""
        with tempfile.TemporaryDirectory() as td:
            spef = self._write(td, "orig.spef", _BACKMARK_SPEF)
            # Only update net_A; net_B should be untouched
            cap_data = self._write(td, "cap.data", "net_A 1.5 3.0\n")
            out = os.path.join(td, "out.spef")

            backmark_spef(spef, cap_data, None, out)

            sf = self._parse(out)
            assert sf.nets["net_B"].total_cap == pytest.approx(0.8)
            caps_b = {n: c for n, _, c in sf.nets["net_B"].segment_caps}
            assert caps_b["drv_u3:Y"] == pytest.approx(0.4)
            assert caps_b["sink_u4:B"] == pytest.approx(0.4)

    def test_cap_only_data(self):
        """Providing only cap data does not modify resistance values."""
        with tempfile.TemporaryDirectory() as td:
            spef = self._write(td, "orig.spef", _BACKMARK_SPEF)
            cap_data = self._write(td, "cap.data", "net_A 1.5 3.0\n")
            out = os.path.join(td, "out.spef")

            backmark_spef(spef, cap_data, None, out)

            sf = self._parse(out)
            # Cap updated
            assert sf.nets["net_A"].total_cap == pytest.approx(3.0)
            # Resistance unchanged
            r = sf.nets["net_A"].driver_sink_resistances()
            assert r["sink_u2:A"] == pytest.approx(10.0)

    def test_res_only_data(self):
        """Providing only res data does not modify capacitance values."""
        with tempfile.TemporaryDirectory() as td:
            spef = self._write(td, "orig.spef", _BACKMARK_SPEF)
            res_data = self._write(
                td, "res.data", "net_A drv_u1:Z sink_u2:A 10.0 20.0\n"
            )
            out = os.path.join(td, "out.spef")

            backmark_spef(spef, None, res_data, out)

            sf = self._parse(out)
            # Total cap unchanged
            assert sf.nets["net_A"].total_cap == pytest.approx(1.5)
            # Resistance scaled
            r = sf.nets["net_A"].driver_sink_resistances()
            assert r["sink_u2:A"] == pytest.approx(20.0)

    def test_zero_old_cap_no_crash(self):
        """A net with zero total cap in the SPEF must not crash (ratio defaults to 1)."""
        spef_content = textwrap.dedent("""\
            *SPEF "IEEE 1481-1999"
            *DESIGN "t"
            *DATE "2026:01:01:00:00:00"
            *VENDOR "t"
            *PROGRAM "t"
            *VERSION "1.0"
            *DIVIDER /
            *DELIMITER :
            *BUS_DELIMITER [ ]
            *T_UNIT 1 NS
            *C_UNIT 1 PF
            *R_UNIT 1 OHM
            *L_UNIT 1 HENRY

            *D_NET net_Z 0.0
            *CONN
            *I drv:Z O
            *RES
            1 drv:Z snk:A 5.0
            *END
        """)
        with tempfile.TemporaryDirectory() as td:
            spef = self._write(td, "spef.spef", spef_content)
            cap_data = self._write(td, "cap.data", "net_Z 0.0 2.0\n")
            out = os.path.join(td, "out.spef")

            backmark_spef(spef, cap_data, None, out)  # must not raise

            sf = self._parse(out)
            # *D_NET line updated to new total cap
            assert sf.nets["net_Z"].total_cap == pytest.approx(2.0)

    # ------------------------------------------------------------------ multi-sink per-segment scaling

    def test_multi_sink_res_per_segment_scaling(self):
        """Shared *RES segments scale by avg ratio; exclusive segments by their sink's ratio.

        Net topology (all R in Ohm):
          drv:Z --[10]-- mid --[6]-- snk_a:A
                               |
                              [4]-- snk_b:B

        Old driver-to-sink R:
          snk_a:A = 10 + 6 = 16
          snk_b:B = 10 + 4 = 14

        Data file targets:
          snk_a:A = 32  → ratio_a = 32/16 = 2.0
          snk_b:B = 14  → ratio_b = 14/14 = 1.0
          avg_ratio = (2.0 + 1.0) / 2 = 1.5

        Expected segment scales:
          drv:Z - mid    (shared, leads to both sinks) → 1.5
          mid - snk_a:A  (exclusive to snk_a)          → 2.0
          mid - snk_b:B  (exclusive to snk_b)          → 1.0

        Expected new segment R values:
          drv:Z - mid    = 10 * 1.5 = 15
          mid - snk_a:A  =  6 * 2.0 = 12
          mid - snk_b:B  =  4 * 1.0 =  4
        """
        spef_content = textwrap.dedent("""\
            *SPEF "IEEE 1481-1999"
            *DESIGN "ms"
            *DATE "2026:01:01:00:00:00"
            *VENDOR "t"
            *PROGRAM "t"
            *VERSION "1.0"
            *DIVIDER /
            *DELIMITER :
            *BUS_DELIMITER [ ]
            *T_UNIT 1 NS
            *C_UNIT 1 PF
            *R_UNIT 1 OHM
            *L_UNIT 1 HENRY

            *NAME_MAP
            *1 net_M
            *2 drv:Z
            *3 mid
            *4 snk_a:A
            *5 snk_b:B

            *D_NET *1 3.0
            *CONN
            *I *2 O *C 0.0 0.0
            *I *4 I *C 0.0 0.0
            *I *5 I *C 0.0 0.0
            *CAP
            1 *2 1.0
            2 *4 1.0
            3 *5 1.0
            *RES
            1 *2 *3 10.0
            2 *3 *4 6.0
            3 *3 *5 4.0
            *END
        """)
        with tempfile.TemporaryDirectory() as td:
            spef = self._write(td, "spef.spef", spef_content)
            res_data = self._write(
                td, "res.data",
                "net_M drv:Z snk_a:A 16.0 32.0\n"
                "net_M drv:Z snk_b:B 14.0 14.0\n",
            )
            out = os.path.join(td, "out.spef")

            backmark_spef(spef, None, res_data, out)

            sf = self._parse(out)
            net = sf.nets["net_M"]

            # Check raw segment resistances in the rewritten graph
            # drv:Z - mid should be scaled by avg_ratio = 1.5 → 15.0
            # mid - snk_a:A should be scaled by ratio_a = 2.0  → 12.0
            # mid - snk_b:B should be scaled by ratio_b = 1.0  →  4.0
            def _edge_r(g: dict, a: str, b: str) -> float:
                for neigh, r in g.get(a, []):
                    if neigh == b:
                        return r
                return float('nan')

            assert _edge_r(net.res_graph, "drv:Z", "mid") == pytest.approx(15.0), (
                "Shared segment not scaled by avg_ratio"
            )
            assert _edge_r(net.res_graph, "mid", "snk_a:A") == pytest.approx(12.0), (
                "Exclusive segment for snk_a not scaled by ratio_a"
            )
            assert _edge_r(net.res_graph, "mid", "snk_b:B") == pytest.approx(4.0), (
                "Exclusive segment for snk_b not scaled by ratio_b"
            )

            # Also verify driver-sink R values
            dr = net.driver_sink_resistances()
            assert dr["snk_a:A"] == pytest.approx(15.0 + 12.0)   # 27.0
            assert dr["snk_b:B"] == pytest.approx(15.0 + 4.0)    # 19.0
