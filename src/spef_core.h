#ifndef SPEF_CORE_H
#define SPEF_CORE_H

#include <iostream>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <tuple>
#include <limits>
#include <cmath>
#include <regex>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <thread>
#include <mutex>
#include <functional>

namespace py = pybind11;

struct Edge {
    std::string to;
    double weight;
};

struct NetData {
    std::string name;
    double total_cap;
    std::string driver;
    std::vector<std::string> sinks;
    std::unordered_map<std::string, std::vector<Edge>> res_graph;
    std::unordered_map<std::string, std::string> node_prefix_map;
    std::unordered_map<std::string, std::string> pin_to_node_cache;  // Recommendation 3: Pre-computed maps
    std::unordered_map<std::string, double> driver_sink_res_cache;
    bool cache_valid;
    bool pin_map_built;
    
    NetData() : total_cap(0.0), cache_valid(false), pin_map_built(false) {}
};

struct ParsedSpef {
    std::unordered_map<std::string, std::string> name_map;
    std::unordered_map<std::string, NetData> nets;
    std::string t_unit;
    std::string c_unit;
    std::string r_unit;
    std::string l_unit;
    
    ParsedSpef() : t_unit("NS"), c_unit("PF"), r_unit("OHM"), l_unit("HENRY") {}
};

// ============== NEW STRUCTURES FOR RECOMMENDATIONS ==============

// Recommendation 1: Batch Resistance Computation
struct ResistanceResult {
    std::string net_name;
    std::string sink_pin;
    double resistance;
};

// Recommendation 2: Vectorized Correlation Computation
struct CorrelationResult {
    double pearson;
    double mean_x;
    double mean_y;
    double std_dev_x;
    double std_dev_y;
    bool valid;
};

// Recommendation 4: Streaming Comparison Mode
struct CapComparisonData {
    std::string net_name;
    double c1;
    double c2;
};

struct ResComparisonData {
    std::string net_name;
    std::string driver;
    std::string sink;
    double r1;
    double r2;
};

// Dijkstra shortest path - C++ optimized
std::unordered_map<std::string, double> dijkstra_shortest_paths(
    const std::unordered_map<std::string, std::vector<Edge>>& graph,
    const std::string& source
);

// Recommendation 3: Pre-compute pin-to-node mapping for a net
void build_pin_to_node_map(NetData& net);

// Compute all driver->sink resistances for a net
std::unordered_map<std::string, double> compute_driver_sink_resistances(
    NetData& net
);

// Recommendation 1: Batch compute driver-sink resistances for multiple nets
std::vector<ResistanceResult> compute_batch_driver_sink_resistances(
    const std::vector<std::string>& net_names,
    ParsedSpef& spef,
    int num_threads = 0
);

// Recommendation 2: Vectorized Pearson correlation computation
CorrelationResult compute_pearson_correlation(
    const std::vector<double>& xs,
    const std::vector<double>& ys
);

// Recommendation 4: Streaming comparison with callbacks
typedef std::function<void(const CapComparisonData&)> CapCallback;
typedef std::function<void(const ResComparisonData&)> ResCallback;

void compare_spef_streaming(
    ParsedSpef& spef1,
    ParsedSpef& spef2,
    CapCallback on_cap,
    ResCallback on_res,
    int num_threads = 0
);

// Parse SPEF file - C++ optimized
ParsedSpef parse_spef(const std::string& filepath);

// Shuffle net ID mapping
void shuffle_spef(
    const std::string& input_path,
    const std::string& output_path,
    int seed
);

#endif // SPEF_CORE_H
