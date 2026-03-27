# Quick Reference: 1M+ Nets Optimization Checklist

## Status: ✅ Code pulled and analyzed

**Current Performance:** ~10-15 seconds for 62K nets (assuming 2 SPEF files)  
**Projected without optimization:** 160-240 seconds for 1M nets  
**Target with optimization:** 10-30 seconds for 1M nets  

---

## 🚀 Immediate Actions (Next 2-3 hours)

### Phase 1: Quick Wins (2-3x speedup)

- [ ] **Replace `std::unordered_map` with `ska::flat_hash_map`** in `spef_core.h`
  - File: `src/spef_core.h`
  - Lines: ~Line 20
  - Effort: 5 mins
  - Gain: 30-50% faster lookups
  
- [ ] **Pre-compute pin-to-node maps** during parsing
  - File: `src/spef_core.cpp`  
  - Add function: `build_pin_to_node_map(NetData& net)`
  - Update: `compute_driver_sink_resistances()` to use cache
  - Effort: 20 mins
  - Gain: 2-3x faster for multi-sink nets

### Phase 1 Subtotal: **1.5-2x faster, ~5 mins implementation**

---

### Phase 2: Batch Processing (5-12x speedup) ⭐ MOST IMPACTFUL

- [ ] **Add multi-threaded batch resistance computation**
  - File: `src/spef_core.h`
  - Add struct: `ResistanceResult`
  - Add function: `compute_batch_driver_sink_resistances()`
  - Effort: 1.5-2 hours
  - Gain: 5-12x parallel speedup

- [ ] **Update Python wrapper**
  - File: `src/wrapper.cpp`
  - Add pybind11 binding for new function
  - Effort: 15 mins

- [ ] **Update Python caller**
  - File: `spef_rc_correlation.py`
  - Replace multiprocessing with C++ threading
  - Eliminate serialization overhead
  - Effort: 30 mins

### Phase 2 Subtotal: **7-24x faster, ~2 hours implementation**

---

## 📊 Memory Usage Optimization (Constant Memory)

### Phase 3: Streaming Mode (Unlimited scalability)

- [ ] **Implement callback-based streaming**
  - File: `src/spef_core.cpp/h`
  - Add function: `compare_spef_streaming()`
  - Benefit: Memory stays constant regardless of net count
  - Effort: 2-3 hours

### Phase 3 Subtotal: **Constant memory vs O(n) growth**

---

## 📈 Vector Correlation (20-50x speedup)

### Phase 4: CPU-Intensive Operations

- [ ] **Move correlation computation to C++**
  - File: `src/spef_core.cpp/h`
  - Add function: `compute_pearson_correlation()`
  - Add SIMD optimizations (optional but recommended)
  - Effort: 2-3 hours
  - Gain: 20-50x for large datasets

---

## 🎯 Testing After Each Phase

```bash
cd /home/aliu/Downloads/spef

# Build after changes
python setup.py build_ext --inplace

# Test on sample file
time python spef_rc_correlation.py 20-blabla.spef 20-blabla_shuffled.spef

# Test with larger file
time python spef_rc_correlation.py 20-blabla_new_shuffled_backmarked.spef \
                                   20-blabla_random.spef --csv-prefix test
```

---

## 📋 Files to Modify

| File | Changes | Priority |
|------|---------|----------|
| `src/spef_core.h` | Add new structs, function signatures | P1 |
| `src/spef_core.cpp` | Implement new functions | P1 |
| `src/wrapper.cpp` | Add pybind11 bindings | P2 |
| `spef_rc_correlation.py` | Use C++ functions, remove multiprocessing | P2 |
| `setup.py` | Enable compiler optimizations (O3, march=native) | P1 |

---

## 🔧 Key Code Patterns

### Pattern 1: Thread Pool Work Distribution
```cpp
for (size_t i = thread_id; i < net_count; i += num_threads) {
    // Process net i in this thread
}
```

### Pattern 2: Thread-Safe Result Collection
```cpp
std::mutex results_mutex;
{
    std::lock_guard<std::mutex> lock(results_mutex);
    results.insert(results.end(), local_results.begin(), local_results.end());
}
```

### Pattern 3: Memory Layout Optimization
```cpp
// Use pointer to vector data for faster access
const double* x_ptr = xs.data();
for (size_t i = 0; i < n; ++i) {
    double x = x_ptr[i];  // Faster than xs[i]
}
```

---

## 📊 Benchmarking Strategy

1. **Baseline:** Time current code on test files
2. **Phase 1:** Should see 1.5-2x improvement
3. **Phase 2:** Should see 7-24x improvement (most dramatic)
4. **Phase 3:** Memory should be constant instead of linear
5. **Phase 4:** Additional 2-3x for correlation operations

---

## 🆘 Common Issues & Solutions

### Issue: Import Error on `spef_core`
```
ImportError: cannot import name 'spef_core'
```
**Solution:** Rebuild: `python setup.py build_ext --inplace`

### Issue: Segmentation Fault in Threading
```
Segmentation fault (core dumped)
```
**Solution:** Ensure all data structures are thread-safe. Use `std::mutex` for shared data.

### Issue: Threads Not Using All Cores
**Solution:** Check `num_threads = os.cpu_count()` is being passed correctly

---

## 💡 Pro Tips

1. **Use `flat_hash_map`**: Easy to integrate, significant speedup
2. **Profile before optimizing**: Use `cProfile` to identify real bottlenecks
3. **Compile with `-ffast-math -O3 -march=native`** for maximum performance
4. **Test with progressively larger files** (10K → 100K → 1M nets)
5. **Use `std::move`** to avoid unnecessary copies in C++
6. **Pre-allocate vectors** with `.reserve()` when size is known

---

## 📞 Expected Results Timeline

| Time | Action | Expected Speedup |
|------|--------|-----------------|
| Now | Code analyzed | Baseline |
| +30 mins | Phase 1 changes | 1.5-2x |
| +2 hrs | Phase 2 changes | 7-24x ⭐ |
| +5 hrs | Phase 3 changes | Constant memory |
| +8 hrs | Phase 4 changes | 200-1000x total |

---

## Next Steps

1. **Backup current code:**
   ```bash
   git commit -m "checkpoint: before performance optimization"
   ```

2. **Start with Phase 1 (quick wins):**
   - Switch to flat_hash_map
   - Pre-compute pin maps

3. **Test and benchmark:**
   - Run on test files
   - Measure memory usage

4. **If Phase 1 successful, proceed to Phase 2:**
   - Batch multi-threaded computation
   - This is the biggest opportunity

---

## References

- **Previous Documents:**
  - `PERFORMANCE_OPTIMIZATION_RECOMMENDATIONS.md` - Detailed analysis
  - `CPP_IMPLEMENTATION_EXAMPLES.md` - Full code examples

- **External Resources:**
  - [ska flat_hash_map](https://github.com/skarupke/flat_hash_map)
  - [OpenMP for parallelization](https://www.openmp.org/)
  - [C++17 threading documentation](https://en.cppreference.com/w/cpp/thread)

