#!/usr/bin/env python3
"""
bench_spef.py – SPEF parser performance benchmark.

Usage:
    python bench/bench_spef.py [--nets N] [--runs R] [--threads T]

Generates a synthetic SPEF with N nets, runs the parser R times, and prints
p50/p95 latency, throughput (nets/s), and peak RSS memory.

Output is also written to bench/results.txt for CI baseline tracking.
"""
import argparse
import os
import random
import resource
import statistics
import sys
import tempfile
import time

# Ensure the package root is on sys.path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import spef_core


# ──────────────────────────────────────────────────────────────────────────────
# SPEF generator
# ──────────────────────────────────────────────────────────────────────────────

def generate_spef(n_nets: int, seed: int = 42) -> str:
    """Return a synthetic SPEF string with *n_nets* D_NET blocks."""
    rng = random.Random(seed)
    lines = [
        '*SPEF "IEEE 1481-1999"',
        '*DESIGN "bench"',
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
        n_sinks = rng.randint(1, 4)
        lines.append(f'*D_NET *{i} {total_cap}')
        lines.append('*CONN')
        lines.append(f'*P driver_{i} I')
        for s in range(n_sinks):
            lines.append(f'*I *{i}:sink{s} I')
        lines.append('*CAP')
        lines.append(f'1 *{i}:sink0 {round(total_cap * 0.3, 8)}')
        if i < n_nets:
            j = (i % n_nets) + 1
            cc = round(rng.uniform(0.001, 0.01), 8)
            lines.append(f'2 *{i}:sink0 *{j}:sink0 {cc}')
        lines.append('*RES')
        for s in range(n_sinks):
            r = round(rng.uniform(0.1, 5.0), 6)
            lines.append(f'{s + 1} *{i}:sink{s} node_{i}_{s} {r}')
        lines.append('*END')

    return '\n'.join(lines) + '\n'


# ──────────────────────────────────────────────────────────────────────────────
# Benchmark runner
# ──────────────────────────────────────────────────────────────────────────────

def run_benchmark(n_nets: int, n_runs: int) -> dict:
    print(f"\n{'='*60}")
    print(f"Generating SPEF with {n_nets:,} nets …", flush=True)
    spef_str = generate_spef(n_nets)

    with tempfile.NamedTemporaryFile(
            mode='w', suffix='.spef', delete=False, prefix='bench_') as f:
        f.write(spef_str)
        spef_path = f.name

    file_kb = os.path.getsize(spef_path) // 1024
    print(f"File size: {file_kb:,} KB  |  Runs: {n_runs}")

    latencies = []
    try:
        for run in range(n_runs):
            t0 = time.perf_counter()
            parsed = spef_core.parse_spef(spef_path)
            t1 = time.perf_counter()
            latencies.append(t1 - t0)
            assert len(parsed.nets) == n_nets, \
                f"Run {run}: expected {n_nets} nets, got {len(parsed.nets)}"
    finally:
        os.unlink(spef_path)

    rss_kb = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    # Linux reports in KB, macOS in bytes
    if sys.platform == 'darwin':
        rss_kb //= 1024

    latencies.sort()
    p50 = statistics.median(latencies)
    p95 = latencies[int(len(latencies) * 0.95)]
    throughput = n_nets / p50

    print(f"\nResults ({n_runs} runs, {n_nets:,} nets):")
    print(f"  p50 latency : {p50*1000:.1f} ms")
    print(f"  p95 latency : {p95*1000:.1f} ms")
    print(f"  min latency : {min(latencies)*1000:.1f} ms")
    print(f"  max latency : {max(latencies)*1000:.1f} ms")
    print(f"  throughput  : {throughput:,.0f} nets/s")
    print(f"  peak RSS    : {rss_kb:,} KB")

    return {
        "n_nets": n_nets,
        "n_runs": n_runs,
        "file_kb": file_kb,
        "p50_ms": round(p50 * 1000, 2),
        "p95_ms": round(p95 * 1000, 2),
        "min_ms": round(min(latencies) * 1000, 2),
        "max_ms": round(max(latencies) * 1000, 2),
        "throughput_nets_per_s": round(throughput),
        "peak_rss_kb": rss_kb,
    }


def run_dual_parse_benchmark(n_nets: int, n_runs: int) -> dict:
    """Benchmark two-file parallel parse (export_plot_data scenario)."""
    print(f"\n{'='*60}")
    print(f"Dual-file benchmark: {n_nets:,} nets × 2  …", flush=True)
    s1 = generate_spef(n_nets, seed=1)
    s2 = generate_spef(n_nets, seed=2)

    with (tempfile.NamedTemporaryFile(mode='w', suffix='.spef', delete=False, prefix='bench_a_') as fa,
          tempfile.NamedTemporaryFile(mode='w', suffix='.spef', delete=False, prefix='bench_b_') as fb):
        fa.write(s1); fa_path = fa.name
        fb.write(s2); fb_path = fb.name

    latencies = []
    try:
        for _ in range(n_runs):
            t0 = time.perf_counter()
            p1 = spef_core.parse_spef(fa_path)
            p2 = spef_core.parse_spef(fb_path)
            t1 = time.perf_counter()
            latencies.append(t1 - t0)
            assert len(p1.nets) == n_nets
            assert len(p2.nets) == n_nets
    finally:
        os.unlink(fa_path)
        os.unlink(fb_path)

    p50 = statistics.median(latencies)
    throughput = (2 * n_nets) / p50
    print(f"  p50 latency (dual) : {p50*1000:.1f} ms  |  {throughput:,.0f} nets/s (combined)")
    return {"dual_p50_ms": round(p50 * 1000, 2), "dual_throughput": round(throughput)}


# ──────────────────────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="SPEF parser benchmark")
    parser.add_argument('--nets', type=int, default=100_000,
                        help='Number of nets to generate (default: 100_000)')
    parser.add_argument('--runs', type=int, default=5,
                        help='Number of timed runs per scenario (default: 5)')
    args = parser.parse_args()

    results = run_benchmark(args.nets, args.runs)
    dual = run_dual_parse_benchmark(min(args.nets, 50_000), max(args.runs, 3))
    results.update(dual)

    # Write results file for CI baseline tracking
    out_dir = os.path.dirname(os.path.abspath(__file__))
    out_path = os.path.join(out_dir, 'results.txt')
    with open(out_path, 'w') as fp:
        for k, v in results.items():
            fp.write(f'{k}={v}\n')
    print(f"\nResults written to {out_path}")


if __name__ == '__main__':
    main()
