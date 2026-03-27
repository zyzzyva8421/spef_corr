# C++ Implementation Examples for Performance Optimization

This document provides code examples for implementing the recommended optimizations.

---

## 1. Batch Resistance Computation with Thread Pool

Add to `spef_core.h`:

```cpp
#include <thread>
#include <queue>
#include <condition_variable>
#include <mutex>

struct ResistanceResult {
    std::string net_name;
    std::string sink_pin;
    double resistance;
};

// Batch compute driver-sink resistances for multiple nets
std::vector<ResistanceResult> compute_batch_driver_sink_resistances(
    const std::vector<std::string>& net_names,
    const ParsedSpef& spef,
    int num_threads = 0  // 0 = auto-detect CPU count
);
```

Add to `spef_core.cpp`:

```cpp
std::vector<ResistanceResult> compute_batch_driver_sink_resistances(
    const std::vector<std::string>& net_names,
    const ParsedSpef& spef,
    int num_threads
) {
    if (num_threads <= 0) {
        num_threads = std::thread::hardware_concurrency();
        if (num_threads <= 0) num_threads = 4;
    }
    
    std::vector<ResistanceResult> results;
    std::mutex results_mutex;
    size_t net_count = net_names.size();
    
    // Worker function for each thread
    auto worker = [&](size_t thread_id) {
        std::vector<ResistanceResult> local_results;
        
        // Simple work distribution: each thread processes nets % num_threads == thread_id
        for (size_t i = thread_id; i < net_count; i += num_threads) {
            const auto& net_name = net_names[i];
            auto it = spef.nets.find(net_name);
            if (it == spef.nets.end()) continue;
            
            const auto& net = it->second;
            
            // Compute driver-sink resistances for this net
            auto dists = dijkstra_shortest_paths(net.res_graph, net.driver);
            
            for (const auto& sink : net.sinks) {
                auto sink_it = dists.find(sink);
                if (sink_it != dists.end()) {
                    local_results.push_back(ResistanceResult{
                        net_name,
                        sink,
                        sink_it->second
                    });
                }
            }
        }
        
        // Merge local results into global results (thread-safe)
        {
            std::lock_guard<std::mutex> lock(results_mutex);
            results.insert(results.end(), local_results.begin(), local_results.end());
        }
    };
    
    // Create and join threads
    std::vector<std::thread> threads;
    for (int i = 0; i < num_threads; ++i) {
        threads.emplace_back(worker, i);
    }
    
    for (auto& t : threads) {
        t.join();
    }
    
    return results;
}
```

---

## 2. Vectorized Pearson Correlation

Add to `spef_core.h`:

```cpp
struct CorrelationResult {
    double pearson;
    double mean_x;
    double mean_y;
    double std_dev_x;
    double std_dev_y;
    bool valid;
};

// Single-pass correlation computation (optimized)
CorrelationResult compute_pearson_correlation(
    const std::vector<double>& xs,
    const std::vector<double>& ys
);

// Batch correlation for comparison
struct CorrelationComparison {
    CorrelationResult caps_corr;
    CorrelationResult res_corr;
    double cap_correlation;
    double res_correlation;
};
```

Add to `spef_core.cpp`:

```cpp
CorrelationResult compute_pearson_correlation(
    const std::vector<double>& xs,
    const std::vector<double>& ys
) {
    CorrelationResult result{0.0, 0.0, 0.0, 0.0, 0.0, false};
    
    size_t n = xs.size();
    if (n != ys.size() || n < 2) {
        return result;
    }
    
    // Single pass: compute mean, sum of squares, and covariance
    double sum_x = 0.0, sum_y = 0.0;
    double sum_xy = 0.0, sum_x2 = 0.0, sum_y2 = 0.0;
    
    // Use local variables for faster access
    const double* x_ptr = xs.data();
    const double* y_ptr = ys.data();
    
    for (size_t i = 0; i < n; ++i) {
        double x = x_ptr[i];
        double y = y_ptr[i];
        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_x2 += x * x;
        sum_y2 += y * y;
    }
    
    double mean_x = sum_x / n;
    double mean_y = sum_y / n;
    
    double cov_xy = (sum_xy - n * mean_x * mean_y);
    double var_x = (sum_x2 - n * mean_x * mean_x);
    double var_y = (sum_y2 - n * mean_y * mean_y);
    
    result.mean_x = mean_x;
    result.mean_y = mean_y;
    result.std_dev_x = std::sqrt(std::max(0.0, var_x / n));
    result.std_dev_y = std::sqrt(std::max(0.0, var_y / n));
    
    if (result.std_dev_x < 1e-15 || result.std_dev_y < 1e-15) {
        return result;  // Not enough variance
    }
    
    result.pearson = cov_xy / (std::sqrt(std::max(0.0, var_x)) * std::sqrt(std::max(0.0, var_y)));
    result.valid = true;
    
    return result;
}
```

---

## 3. Pre-computed Pin Maps

Modify `NetData` in `spef_core.h`:

```cpp
struct NetData {
    std::string name;
    double total_cap;
    std::string driver;
    std::vector<std::string> sinks;
    std::unordered_map<std::string, std::vector<Edge>> res_graph;
    std::unordered_map<std::string, std::string> node_prefix_map;
    
    // NEW: Pre-computed pin-to-node mappings
    std::unordered_map<std::string, std::string> pin_to_node_cache;
    
    std::unordered_map<std::string, double> driver_sink_res_cache;
    bool cache_valid;
    
    NetData() : total_cap(0.0), cache_valid(false) {}
};
```

Update SPEF parser to build pin maps:

```cpp
void build_pin_to_node_map(NetData& net) {
    net.pin_to_node_cache.clear();
    
    // Helper: find best matching node for a pin
    auto find_best_match = [&](const std::string& pin) -> std::string {
        if (pin.empty()) return pin;
        
        // Try exact match first
        if (net.res_graph.find(pin) != net.res_graph.end()) {
            return pin;
        }
        
        // Try prefix match
        size_t colon_pos = pin.find(':');
        std::string base = (colon_pos != std::string::npos) ? 
            pin.substr(0, colon_pos) : pin;
        
        for (const auto& [node, _] : net.res_graph) {
            size_t node_colon = node.find(':');
            std::string node_base = (node_colon != std::string::npos) ?
                node.substr(0, node_colon) : node;
            if (node_base == base) {
                return node;
            }
        }
        return pin;  // Return original if no match
    };
    
    // Pre-compute for all pins (driver + sinks)
    net.pin_to_node_cache[net.driver] = find_best_match(net.driver);
    
    for (const auto& sink : net.sinks) {
        net.pin_to_node_cache[sink] = find_best_match(sink);
    }
}
```

Update resistance computation to use pre-computed maps:

```cpp
std::unordered_map<std::string, double> compute_driver_sink_resistances(
    NetData& net
) {
    if (net.cache_valid && !net.driver_sink_res_cache.empty()) {
        return net.driver_sink_res_cache;
    }
    
    net.driver_sink_res_cache.clear();
    
    if (net.driver.empty() || net.sinks.empty() || net.res_graph.empty()) {
        net.cache_valid = true;
        return net.driver_sink_res_cache;
    }
    
    // Use pre-computed pin maps (much faster!)
    if (net.pin_to_node_cache.empty()) {
        build_pin_to_node_map(net);
    }
    
    std::string driver_node = net.pin_to_node_cache[net.driver];
    
    // Early return if driver not in graph
    if (net.res_graph.find(driver_node) == net.res_graph.end()) {
        net.cache_valid = true;
        return net.driver_sink_res_cache;
    }
    
    auto dists = dijkstra_shortest_paths(net.res_graph, driver_node);
    
    for (const auto& sink : net.sinks) {
        std::string sink_node = net.pin_to_node_cache[sink];
        
        auto it = dists.find(sink_node);
        if (it != dists.end()) {
            net.driver_sink_res_cache[sink] = it->second;
        }
    }
    
    net.cache_valid = true;
    return net.driver_sink_res_cache;
}
```

---

## 4. Streaming Comparison Mode

Add to `spef_core.h`:

```cpp
// Callback-based comparison
typedef std::function<void(const std::string&, double, double)> CapCallback;
typedef std::function<void(const std::string&, const std::string&, 
                          const std::string&, double, double)> ResCallback;

void compare_spef_streaming(
    const ParsedSpef& spef1,
    const ParsedSpef& spef2,
    CapCallback on_cap,
    ResCallback on_res,
    int num_threads = 0
);
```

Add to `spef_core.cpp`:

```cpp
void compare_spef_streaming(
    const ParsedSpef& spef1,
    const ParsedSpef& spef2,
    CapCallback on_cap,
    ResCallback on_res,
    int num_threads
) {
    if (num_threads <= 0) {
        num_threads = std::thread::hardware_concurrency();
        if (num_threads <= 0) num_threads = 4;
    }
    
    // Find common nets
    std::vector<std::string> common_nets;
    for (const auto& [name, _] : spef1.nets) {
        if (spef2.nets.find(name) != spef2.nets.end()) {
            common_nets.push_back(name);
        }
    }
    
    std::sort(common_nets.begin(), common_nets.end());
    
    // Process in parallel
    std::mutex callback_mutex;
    
    auto worker = [&](size_t thread_id) {
        for (size_t i = thread_id; i < common_nets.size(); i += num_threads) {
            const auto& net_name = common_nets[i];
            const auto& net1 = spef1.nets.at(net_name);
            const auto& net2 = spef2.nets.at(net_name);
            
            // Capacitance comparison
            {
                std::lock_guard<std::mutex> lock(callback_mutex);
                on_cap(net_name, net1.total_cap, net2.total_cap);
            }
            
            // Resistance comparison
            auto res1 = compute_driver_sink_resistances(
                const_cast<NetData&>(net1)
            );
            auto res2 = compute_driver_sink_resistances(
                const_cast<NetData&>(net2)
            );
            
            // Find common sinks
            auto common_sinks = get_common_sinks(res1, res2);
            
            for (const auto& sink : common_sinks) {
                {
                    std::lock_guard<std::mutex> lock(callback_mutex);
                    on_res(net_name, net1.driver, sink, res1[sink], res2[sink]);
                }
            }
        }
    };
    
    std::vector<std::thread> threads;
    for (int i = 0; i < num_threads; ++i) {
        threads.emplace_back(worker, i);
    }
    
    for (auto& t : threads) {
        t.join();
    }
}
```

---

## 5. Python Wrapper Changes

Update `wrapper.cpp` to expose new functions:

```cpp
PYBIND11_MODULE(spef_core, m) {
    m.doc() = "SPEF C++ core - optimized parser and shortest path";
    
    // ... existing bindings ...
    
    // NEW: Batch computation
    m.def("compute_batch_driver_sink_resistances",
          &compute_batch_driver_sink_resistances,
          "Compute resistances for multiple nets in parallel",
          py::arg("net_names"),
          py::arg("spef"),
          py::arg("num_threads") = 0);
    
    // NEW: Correlation
    py::class_<CorrelationResult>(m, "CorrelationResult")
        .def_readwrite("pearson", &CorrelationResult::pearson)
        .def_readwrite("mean_x", &CorrelationResult::mean_x)
        .def_readwrite("mean_y", &CorrelationResult::mean_y)
        .def_readwrite("std_dev_x", &CorrelationResult::std_dev_x)
        .def_readwrite("std_dev_y", &CorrelationResult::std_dev_y)
        .def_readwrite("valid", &CorrelationResult::valid);
    
    m.def("compute_pearson_correlation",
          &compute_pearson_correlation,
          "Compute Pearson correlation coefficient",
          py::arg("xs"),
          py::arg("ys"));
    
    // NEW: Streaming comparison
    m.def("compare_spef_streaming",
          &compare_spef_streaming,
          "Stream-based comparison of two SPEF files",
          py::arg("spef1"),
          py::arg("spef2"),
          py::arg("on_cap_callback"),
          py::arg("on_res_callback"),
          py::arg("num_threads") = 0);
}
```

---

## 6. Updated Python Usage

Change `spef_rc_correlation.py` to use optimized C++ functions:

```python
# OLD: Python multiprocessing (slow for 1M nets)
def compare_spef_old(s1: SpefFile, s2: SpefFile, r_agg: str):
    common_nets = sorted(set(s1.nets.keys()) & set(s2.nets.keys()))
    with multiprocessing.Pool(initializer=init_worker, initargs=(s1, s2)) as pool:
        batch_results = pool.map(process_net_batch, batches)
    # ... collect all results ...

# NEW: C++ parallelization (fast for 1M nets)
def compare_spef_optimized(cpp_spef1, cpp_spef2, output_file=None):
    """Stream-based comparison using C++ threading"""
    
    cap_results = []
    res_results = []
    
    def on_cap(net_name, c1, c2):
        cap_results.append(CapComparison(net_name, c1, c2))
    
    def on_res(net_name, driver, sink, r1, r2):
        res_results.append(ResComparison(net_name, driver, sink, r1, r2))
        if output_file:
            output_file.write(f"{net_name} {driver} {sink} {r1} {r2}\n")
    
    # All computation happens in C++ with thread pool
    spef_core.compare_spef_streaming(
        cpp_spef1, 
        cpp_spef2,
        on_cap,
        on_res,
        num_threads=os.cpu_count()
    )
    
    return cap_results, res_results
```

---

## Compilation Instructions

```bash
cd /home/aliu/Downloads/spef

# Rebuild with optimizations
python setup.py build_ext --inplace -O2

# For maximum performance, use O3 and march=native
# Edit setup.py extra_compile_args to include:
# ['-O3', '-march=native', '-ffast-math', '-std=c++17']

python setup.py build_ext --inplace
```

---

## Testing Performance

```python
import time
import spef_core

# Load SPEF files
cpp_spef1 = spef_core.parse_spef("file1.spef")
cpp_spef2 = spef_core.parse_spef("file2.spef")

# Benchmark old approach (Python)
start = time.time()
cap_rows, res_rows = compare_spef_python(s1, s2, "avg")
python_time = time.time() - start

# Benchmark new approach (C++)
start = time.time()
cap_results, res_results = compare_spef_optimized(cpp_spef1, cpp_spef2)
cpp_time = time.time() - start

print(f"Python time: {python_time:.2f}s")
print(f"C++ time: {cpp_time:.2f}s")
print(f"Speedup: {python_time/cpp_time:.1f}x")
```

Expected speedup for 1M nets: **50-200x**

