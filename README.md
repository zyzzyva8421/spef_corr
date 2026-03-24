# SPEF RC Correlation Tool

Compare RC parasitic data between two SPEF files from different extraction tools.

## Features

- **Fast SPEF Parsing**: Optimized single-pass parser (~900ms for 62K nets)
- **Parallel Loading**: Extract two SPEF files concurrently
- **GUI Analysis**: Interactive multi-SPEF correlation visualization using Tkinter + Matplotlib
- **CLI Mode**: Batch analysis with CSV export
- **Correlation Computing**: Pearson correlation for capacitance and resistance

## Installation

```bash
python -m venv .venv
source .venv/bin/activate
pip install matplotlib
```

## Usage

### GUI Mode (Interactive)

```bash
python spef_rc_correlation.py [spef_file_or_dir] [spef_file_or_dir] --gui [--gui-auto-run]
```

Examples:
```bash
# Launch empty GUI
python spef_rc_correlation.py --gui

# Preload two SPEF files and auto-run analysis
python spef_rc_correlation.py file1.spef file2.spef --gui --gui-auto-run

# Load all .spef files from a directory and auto-run
python spef_rc_correlation.py ./netlists --gui --gui-auto-run
```

### CLI Mode (Batch)

```bash
python spef_rc_correlation.py spef1.spef spef2.spef [--csv-prefix PREFIX] [--r-agg {max,avg,total}]
```

Examples:
```bash
# Compare two files, export CSVs
python spef_rc_correlation.py toolA.spef toolB.spef --csv-prefix results/compare

# Change resistance aggregation mode to average
python spef_rc_correlation.py toolA.spef toolB.spef --csv-prefix results/compare --r-agg avg
```

## Testing

```bash
pytest test_spef_rc_correlation.py -v
```

Coverage: 47 regression tests covering parse, compare, GUI wiring, CSV export, etc.

## Performance

- **Optimized Parse**: 2.4s for two 62K-net SPEF files (parse + compare)
- **Parallel Loading**: ~13% speedup via ProcessPoolExecutor
- **Single-Pass Parser**: Regex-free hot path, local variable caching, early termination

## Key Improvements

1. **Parse Optimization** (47% faster):
   - Removed per-line regex in favor of `startswith()` branches
   - Inline `_resolve_name` to eliminate method dispatch
   - Local variable binding to reduce attribute lookup
   - Bounded `split(None, N)` to avoid tail parsing

2. **Parallel SPEF Loading**:
   - Two files parse concurrently via `ProcessPoolExecutor`
   - Automatic fallback to sequential on subprocess errors
   - Works in both CLI and GUI modes

3. **Comprehensive Testing**:
   - 47 regression tests (pearson_corr, parse, compare, CSV, GUI wiring)
   - No external SPEF file dependencies
   - Lightweight mocking for GUI layer

## Architecture

- `SpefFile`: Parses a single SPEF, builds net library with resistance graphs
- `NetRC`: Per-net data (driver, sinks, capacitance, resistance graph) with Dijkstra shortest-path
- `compare_spef()`: Two-file correlation analysis (capacitance & resistance)
- GUI (`launch_gui`): Tkinter + Matplotlib for interactive scatter plots
- CLI (`main`): Entry point for batch mode

## Limitations

- Assumes single main driver per net (direction O or B in *CONN)
- Pin-to-node mapping relies on prefix matching (e.g., `pin` ↔ `pin:1`)
- GUI requires `tkinter` and `matplotlib`
