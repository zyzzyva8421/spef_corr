"""
test_perf_regression.py – lightweight performance regression guard.

Runs the SPEF parser on a 20 000-net synthetic file and asserts that:
  • throughput is above a conservative floor (25 000 nets/s)
  • the number of parsed nets is exactly correct

The threshold is intentionally conservative so it passes on CI runners (often
slow or single-CPU) while still catching catastrophic regressions (e.g. an
accidental O(n²) change).

To update the baseline, adjust THROUGHPUT_FLOOR_NETS_PER_S below.
"""
import os
import random
import sys
import tempfile
import time

import pytest

# Ensure the package root is on sys.path regardless of where pytest is invoked.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import spef_core

# Conservative CI floor — chosen to be well below the measured baseline (~230 K
# nets/s) so the test passes on slow CI runners, cold-cache scenarios, and VMs
# while still catching catastrophic regressions (e.g. accidental O(n²) changes).
# Update this value upward only when hardware is known to sustain higher rates.
THROUGHPUT_FLOOR_NETS_PER_S = 25_000
N_NETS = 20_000


def _generate_spef(n_nets: int, seed: int = 42) -> str:
    rng = random.Random(seed)
    lines = [
        '*SPEF "IEEE 1481-1999"',
        '*DESIGN "perf_test"',
        '*T_UNIT 1.00 NS',
        '*C_UNIT 1.00 PF',
        '*R_UNIT 1.00 OHM',
        '*L_UNIT 1.00 HENRY',
        '*DIVIDER /',
        '*DELIMITER :',
        '*BUS_DELIMITER [ ]',
        '*NAME_MAP',
    ]
    for i in range(1, n_nets + 1):
        lines.append(f'*{i} net_{i}')
    lines.append('')

    for i in range(1, n_nets + 1):
        total_cap = round(rng.uniform(0.01, 1.0), 6)
        n_sinks = rng.randint(1, 3)
        lines.append(f'*D_NET *{i} {total_cap}')
        lines.append('*CONN')
        lines.append(f'*P drv_{i} I')
        for s in range(n_sinks):
            lines.append(f'*I *{i}:s{s} I')
        lines.append('*CAP')
        lines.append(f'1 *{i}:s0 {round(total_cap * 0.4, 8)}')
        if i < n_nets:
            j = (i % n_nets) + 1
            lines.append(f'2 *{i}:s0 *{j}:s0 {round(rng.uniform(0.001, 0.01), 8)}')
        lines.append('*RES')
        for s in range(n_sinks):
            lines.append(f'{s+1} *{i}:s{s} nd_{i}_{s} {round(rng.uniform(0.1, 5.0), 6)}')
        lines.append('*END')

    return '\n'.join(lines) + '\n'


@pytest.fixture(scope='module')
def spef_file():
    content = _generate_spef(N_NETS)
    with tempfile.NamedTemporaryFile(
            mode='w', suffix='.spef', delete=False, prefix='perf_') as f:
        f.write(content)
        path = f.name
    yield path
    os.unlink(path)


def test_parse_net_count(spef_file):
    """Parser must return exactly the expected number of nets."""
    parsed = spef_core.parse_spef(spef_file)
    assert len(parsed.nets) == N_NETS, (
        f"Expected {N_NETS} nets, got {len(parsed.nets)}")


def test_parse_throughput(spef_file):
    """Single-parse throughput must exceed the regression floor."""
    # Warm up (fills OS page cache)
    spef_core.parse_spef(spef_file)
    # Timed run
    t0 = time.perf_counter()
    parsed = spef_core.parse_spef(spef_file)
    elapsed = time.perf_counter() - t0

    throughput = len(parsed.nets) / elapsed
    print(f"\n  Throughput: {throughput:,.0f} nets/s  (floor: {THROUGHPUT_FLOOR_NETS_PER_S:,})")
    assert throughput >= THROUGHPUT_FLOOR_NETS_PER_S, (
        f"Throughput {throughput:.0f} nets/s below floor {THROUGHPUT_FLOOR_NETS_PER_S}")


def test_parse_units(spef_file):
    """Units should be parsed correctly from the header."""
    parsed = spef_core.parse_spef(spef_file)
    assert parsed.c_unit == 'PF'
    assert parsed.r_unit == 'OHM'
    assert abs(parsed.c_scale - 1.0) < 1e-9
    assert abs(parsed.r_scale - 1.0) < 1e-9


def test_parse_name_map(spef_file):
    """NAME_MAP should be populated and resolvable."""
    parsed = spef_core.parse_spef(spef_file)
    assert len(parsed.name_map) == N_NETS
    assert parsed.name_map['*1'] == 'net_1'
    assert parsed.name_map[f'*{N_NETS}'] == f'net_{N_NETS}'


def test_net_fields(spef_file):
    """Spot-check fields on a few nets after parsing."""
    parsed = spef_core.parse_spef(spef_file)
    for idx in [1, 100, N_NETS // 2, N_NETS]:
        net = parsed.nets.get(f'net_{idx}')
        assert net is not None, f'net_{idx} missing'
        assert net.total_cap > 0, f'net_{idx} total_cap={net.total_cap}'
        assert net.driver == f'drv_{idx}', f'net_{idx} driver={net.driver}'
        assert len(net.sinks) >= 1, f'net_{idx} has no sinks'
        assert len(net.res_graph) >= 1, f'net_{idx} has empty res_graph'


def test_coupling_caps(spef_file):
    """Coupling caps should be present and positive."""
    parsed = spef_core.parse_spef(spef_file)
    # Each net i < N_NETS contributes one coupling pair with net i+1
    assert len(parsed.coupling_caps) > 0
    for cc in parsed.coupling_caps:
        assert cc.cap_value > 0, f'Non-positive ccap: {cc}'
        assert cc.net1 != cc.net2
