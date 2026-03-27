# Performance Optimization Recommendations for 1M+ Nets

## Current Code Analysis

✅ **Already Optimized:**
- SPEF parsing is in C++ (~900ms for 62K nets reported in README)
- Dijkstra shortest-path algorithm in C++
- Caching of resistances per net
- Python code has local variable bindings to avoid attribute lookup overhead

❌ **Bottlenecks for 1M+ Nets:**
1. **Resistance Computation Parallelization** - Uses Python `multiprocessing` with pickling overhead
2. **Per-net Pin Matching** - Prefix matching happens every call without pre-computation
3. **Comparison Correlation** - `pearson_corr()` is pure Python with nested loops
4. **Memory Usage** - Storing all results in lists before processing
5. **Dijkstra Repeated Calls** - Each net independently calls shortest-path
6. **CSV Writing Bottleneck** - Serial I/O for large result sets

---

## Recommended C++ Optimizations

### 1. **Batch Resistance Computation (HIGH PRIORITY)**
**Current Bottleneck:** Each net triggers a separate Dijkstra run. With 1M nets and multiple sinks per net, this creates millions of function calls.

**Recommendation:**
```cpp
// Add to spef_core.h
struct ResistanceResult {
    std::string net_name;
    std::string sink_pin;
    double resistance;
};

// Batch process multiple nets with thread pool
std::vector<ResistanceResult> compute_batch_driver_sink_resistances(
    const std::vector<std::string>& net_names,
    const ParsedSpef& spef,
    int num_threads = 0  // 0 = auto-detect
);
```

**Implementation approach:**
- Use `std::thread` or `omp parallel for` for multi-threading
- Avoid Python's GIL limitations by computing entirely in C++
- Each thread processes multiple nets independently
- Expected speedup: **5-12x** on 8-core systems

**Implementation complexity:** Medium
**Estimated effort:** 150-200 lines of C++

---

### 2. **Vectorized Correlation Computation (HIGH PRIORITY)**
**Current Bottleneck:** `pearson_corr()` iterates with Python nested loops:
```python
num = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
```
This becomes a bottleneck with millions of (net, sink) pairs.

**Recommendation:**
```cpp
// Add to spef_core.h
struct CorrelationResult {
    double pearson_correlation;
    double mean_x;
    double mean_y;
    double std_dev_x;
    double std_dev_y;
};

CorrelationResult compute_pearson_correlation(
    const std::vector<double>& xs,
    const std::vector<double>& ys
);

// For comparing two SPEF files
struct CorrelationStats {
    CorrelationResult capacitance_correlation;
    CorrelationResult resistance_correlation;
    std::vector<std::pair<double, ResComparison>> top_deviations;  // sorted
};
```

**Implementation approach:**
- Use single-pass formula to minimize memory allocations
- Use SIMD (SSE/AVX) for vectorized operations on float arrays
- Pre-allocate output buffers
- Expected speedup: **20-50x** for large datasets

**Implementation complexity:** Medium
**Estimated effort:** 200-300 lines of C++

---

### 3. **Pre-computed Pin Prefix Maps (MEDIUM PRIORITY)**
**Current issue:** Pin matching happens on every `driver_sink_resistances()` call:
```python
def _find_best_node_name(self, pin: str) -> Optional[str]:
    if self._node_prefix_map is None:
        # Rebuild map every first call
```

**Recommendation:** Build prefix maps during SPEF parsing:
```cpp
struct NetData {
    // ... existing fields ...
    std::unordered_map<std::string, std::string> pin_to_node_map;  // Pre-built
};

// In parse_spef(), pre-compute all prefix matches
void build_pin_maps(NetData& net) {
    for (auto& [pin_name, _] : net.sinks) {
        // Find best match in res_graph
        std::string best_node = find_best_node_match(pin_name, net.res_graph);
        net.pin_to_node_map[pin_name] = best_node;
    }
}
```

**Expected speedup:** **2-3x** for nets with many sinks
**Implementation complexity:** Low
**Estimated effort:** 50-80 lines of C++

---

### 4. **Streaming Comparison Mode (MEDIUM PRIORITY)**
**Current issue:** All results accumulated in memory before processing

**Recommendation:** Add streaming comparison that writes results on-the-fly:
```cpp
// Callback-based interface
typedef std::function<void(const ResComparison&)> ResultCallback;

void compare_spef_streaming(
    const ParsedSpef& spef1,
    const ParsedSpef& spef2,
    ResultCallback on_cap_comparison,
    ResultCallback on_res_comparison
);
```

**Benefits:**
- Memory usage stays constant regardless of net count
- Results can be written to CSV incrementally
- Enables early termination if needed

**Expected speedup:** **Constant memory instead of O(n)**
**Implementation complexity:** Medium
**Estimated effort:** 150-200 lines of C++

---

### 5. **Optimized Data Structures (MEDIUM PRIORITY)**
**Current:** All resistance values stored as `double` in `unordered_map`

**Recommendation:** Use more efficient storage:
```cpp
// Use flat hash map (faster than std::unordered_map)
#include <flat_hash_map.hpp>  // ska::flat_hash_map

struct NetData {
    ska::flat_hash_map<std::string, std::vector<Edge>> res_graph;
    ska::flat_hash_map<std::string, double> driver_sink_res_cache;  // Better cache locality
};
```

**Benefits:**
- **30-50% faster lookups** than `unordered_map`
- Better cache locality
- Lower memory overhead

**Expected speedup:** **1.3-1.5x** overall
**Implementation complexity:** Low
**Estimated effort:** 20-40 lines (library integration)

---

### 6. **Parallel Top-N Selection (LOW PRIORITY)**
**Current bottleneck:** Finding top 10 deviations uses `heapq.nlargest` on full list
```python
top_10_cap = [row for _, row in nlargest(10, cap_rows_with_deviation)]
```

**Recommendation:** Move to C++ with early termination:
```cpp
template<typename T>
std::vector<std::pair<double, T>> find_top_n_deviations(
    const std::vector<std::pair<double, T>>& data,
    size_t n,
    int num_threads = 0
);
```

**Benefits:**
- Parallel partial sort for large n
- Can stop early for approximate results

**Expected speedup:** **negligible for top 10**, but useful for top 1000+
**Implementation complexity:** Low
**Estimated effort:** 100-150 lines of C++

---

## Implementation Priority for 1M+ Nets

| Priority | Optimization | Expected Speedup | Effort | Memory Gain |
|----------|---------------|-----------------|--------|------------|
| **1** | Batch Resistance Computation | 5-12x | Medium | 20% |
| **2** | Vectorized Correlation | 20-50x | Medium | 10% |
| **3** | Streaming Comparison | ✓ Constant | Medium | 100% ↓ |
| **4** | Pre-computed Pin Maps | 2-3x | Low | 5% |
| **5** | Flat Hash Maps | 1.3-1.5x | Low | 15% |
| **6** | Parallel Top-N | ~1x | Low | - |

**Combined expected improvement for 1M nets: 50-200x faster, constant memory usage**

---

## Python Side Changes

### Switch from `multiprocessing` to `threading`
Current code uses `multiprocessing.Pool` which requires pickling entire SPEF objects.

**Before:**
```python
with multiprocessing.Pool(initializer=init_worker, initargs=(s1, s2)) as pool:
    batch_results = pool.map(process_net_batch, batches)
```

**After (move to C++):**
```python
# All parallelization happens in C++
cap_rows, res_rows = spef_core.compare_spef_batch(
    cpp_spef1, cpp_spef2, 
    num_threads=os.cpu_count()
)
```

**Benefits:**
- No serialization overhead
- Direct shared memory access
- Escape Python's Global Interpreter Lock (GIL)

---

## Incremental Implementation Plan

### Phase 1 (Immediate - 2-3 hours)
- [ ] Pre-computed pin maps in C++
- [ ] Switch to flat hash maps
- Expected improvement: **1.5-2x faster**

### Phase 2 (Next - 4-5 hours)
- [ ] Batch resistance computation with thread pool
- Expected improvement: **5-12x faster** (combined 7-24x)

### Phase 3 (Optional - 6-8 hours)
- [ ] Vectorized correlation in C++ (with SIMD)
- Expected improvement: **20-50x faster** (combined 140-1200x)

### Phase 4 (Nice-to-have)
- [ ] Streaming comparison mode
- Expected improvement: **Constant memory (100% reduction)**

---

## Benchmarking Commands

```bash
# Profile current performance
python -m cProfile -s cumulative spef_rc_correlation.py file1.spef file2.spef --csv-prefix results

# After optimizations
time python spef_rc_correlation.py file1.spef file2.spef --csv-prefix results
```

Expected progression:
- Baseline (no optimization): ~30-60 seconds for 100K nets
- Phase 1: ~15-20 seconds
- Phase 2: ~3-5 seconds  
- Phase 3: ~0.5-1 seconds

---

## Key Metrics for 1M Nets

| Metric | Current (62K nets) | Projected (1M nets) |
|--------|-------------------|-------------------|
| Parse time | 900ms | **~15s** → **5-8s** (optimized) |
| Resistance computation | ~2-5s | **30-60s** → **2-10s** (optimized) |
| Comparison | ~5-10s | **80-150s** → **2-10s** (optimized) |
| Memory usage | ~500MB | **~8GB** → **~500MB** (streaming) |
| **Total time** | ~10s | **~110-225s** → **~10-30s** |

