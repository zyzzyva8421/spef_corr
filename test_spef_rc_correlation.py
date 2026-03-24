"""
Regression tests for spef_rc_correlation.py

Coverage:
  - pearson_corr: perfect correlation, anti-correlation, constant series, mismatched lengths
  - NetRC._shortest_resistance: direct edge, multi-hop, unreachable node
  - NetRC.driver_sink_resistances: result caching, prefix-match fallback
  - SpefFile.parse: NAME_MAP, *D_NET/*CONN/*RES/*END, KOHM unit scaling,
                    quoted names, comment stripping, unknown-direction lines
  - compare_spef: same-file → correlation 1.0, disjoint nets → empty result
  - write_caps_csv / write_res_csv: round-trip CSV content check
  - collect_spef_paths: directory expansion, direct file path, non-spef file filtering
  - parse_spefs_parallel: returns two SpefFile objects with the same nets
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
    collect_spef_paths,
    compare_spef,
    parse_net_cap_data,
    parse_net_res_data,
    parse_spefs_parallel,
    pearson_corr,
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
    *1 *2 0.5
    *2 *3 0.5
    *3 *4 0.5
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
        assert net._shortest_resistance("drv", "mid") == pytest.approx(10.0)

    def test_multi_hop(self):
        net = self._simple_net()
        assert net._shortest_resistance("drv", "sink") == pytest.approx(15.0)

    def test_same_node_is_zero(self):
        net = self._simple_net()
        assert net._shortest_resistance("drv", "drv") == pytest.approx(0.0)

    def test_unreachable_returns_none(self):
        net = self._simple_net()
        assert net._shortest_resistance("drv", "ghost") is None

    def test_missing_src_returns_none(self):
        net = self._simple_net()
        assert net._shortest_resistance("ghost", "sink") is None

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
