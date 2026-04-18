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
#include <set>

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
    // Temporary storage for coupling capacitances (raw format: node1|node2|cap_value)
    std::vector<std::string> raw_coupling_caps;
    bool cache_valid;
    bool pin_map_built;
    
    NetData() : total_cap(0.0), cache_valid(false), pin_map_built(false) {}
};

// Structure to represent coupling capacitance between two nets
struct CouplingCap {
    std::string net1;  // First net name (resolved)
    std::string net2;  // Second net name (resolved)
    double cap_value;   // Coupling capacitance value
    
    // For sorting and comparison
    bool operator<(const CouplingCap& other) const {
        if (net1 != other.net1) return net1 < other.net1;
        return net2 < other.net2;
    }
};

struct ParsedSpef {
    std::unordered_map<std::string, std::string> name_map;
    std::unordered_map<std::string, NetData> nets;
    std::vector<CouplingCap> coupling_caps;  // All coupling capacitors
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

struct CouplingCapComparison {
    std::string net1;  // First net name
    std::string net2;  // Second net name
    double c1;         // Coupling cap in spef1
    double c2;         // Coupling cap in spef2
};

// Dijkstra shortest path - C++ optimized
std::unordered_map<std::string, double> dijkstra_shortest_paths(
    const std::unordered_map<std::string, std::vector<Edge>>& graph,
    const std::string& source
);

// Recommendation 3: Pre-compute pin-to-node mapping for a net
void build_pin_to_node_map(NetData& net);

// Post-process coupling capacitances to map nodes to net names
void resolve_coupling_caps_to_nets(ParsedSpef& spef);

// Compare coupling capacitances between two SPEF files
std::vector<CouplingCapComparison> compare_coupling_caps(
    const ParsedSpef& spef1,
    const ParsedSpef& spef2
);

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

// ============== BACKMARK FUNCTIONS ==============

// Backmark: update cap/res values from data files
void backmark_spef(
    const std::string& spef_path,
    const std::string& cap_data_path,
    const std::string& res_data_path,
    const std::string& ccap_data_path,
    const std::string& output_path
);

// Parse backmark cap data
std::unordered_map<std::string, double> parse_backmark_cap_data(
    const std::string& path
);

// Parse backmark res data
std::unordered_map<std::string, std::unordered_map<std::string, double>> parse_backmark_res_data(
    const std::string& path
);

// Parse CSV cap data file (net,c1,c2 format)
std::vector<std::tuple<std::string, double, double>> parse_cap_data(
    const std::string& path
);

// Parse CSV res data file (net,r1,r2 format)
std::vector<std::tuple<std::string, std::string, std::string, double, double>> parse_res_data(
    const std::string& path
);

// Parse coupling cap data file (net1 net2 c1 c2)
std::vector<std::tuple<std::string, std::string, double, double>> parse_ccap_data(
    const std::string& path
);

// Forward declaration
struct PlotData;

// Create PlotData from cap/res/ccap files for unified plotting
PlotData create_plot_data_from_files(
    const std::string& cap_path,
    const std::string& res_path,
    const std::string& ccap_path = ""
);

// Compute segment scales for backmarking
std::unordered_map<std::string, std::unordered_map<std::string, double>> compute_res_segment_scales(
    NetData& net,
    const std::unordered_map<std::string, double>& sink_ratios,
    double avg_ratio
);

// ============== CORRELATION FUNCTIONS ==============

// Compare two SPEFs and return all comparisons
struct ComparisonResult {
    std::vector<CapComparisonData> cap_rows;
    std::vector<ResComparisonData> res_rows;
    std::vector<CapComparisonData> top_10_cap;
    std::vector<ResComparisonData> top_10_res;
    std::vector<std::string> common_nets;
    double cap_correlation;
    double res_correlation;
    size_t cap_count;
    size_t res_count;
};

ComparisonResult compare_spef_full(
    ParsedSpef& spef1,
    ParsedSpef& spef2,
    int num_threads = 0
);

// Summarize comparison results
std::string summarize_comparison(const ComparisonResult& result);

// Parse multiple SPEF files in parallel using C++ threads
std::vector<ParsedSpef> parse_spef_parallel(
    const std::vector<std::string>& filepaths,
    int num_threads = 0
);

// ============== UNIT CONVERSION ==============
// Convert value to standard unit (OHM for R, PF for C)
double convert_capacitance(double value, const std::string& from_unit);
double convert_resistance(double value, const std::string& from_unit);

// ============== NUMPY ARRAY EXPORT FOR FAST PLOTTING ==============
// Export comparison results as numpy arrays - avoids Python loop overhead

struct PlotData {
    // Capacitance data (num_nets x 1)
    py::array_t<double> cap_c1;
    py::array_t<double> cap_c2;
    std::vector<std::string> cap_net_names;
    
    // Resistance data (num_driver_sink_pairs x 1)
    py::array_t<double> res_r1;
    py::array_t<double> res_r2;
    std::vector<std::string> res_net_names;
    std::vector<std::string> res_sink_names;
    std::vector<std::string> res_driver_names;

    // Coupling capacitance data (num_pairs x 1)
    py::array_t<double> ccap_c1;
    py::array_t<double> ccap_c2;
    std::vector<std::string> ccap_net1_names;
    std::vector<std::string> ccap_net2_names;
    
    // Correlation values
    double cap_correlation;
    double res_correlation;
    double ccap_correlation;
    size_t cap_count;
    size_t res_count;
    size_t ccap_count;
};

PlotData export_plot_data(
    ParsedSpef& spef1,
    ParsedSpef& spef2,
    int num_threads = 0
);

// Chunked comparison for very large datasets (1M+ nets)
// Returns results in batches to avoid memory issues
struct ComparisonChunk {
    py::array_t<double> cap_c1;
    py::array_t<double> cap_c2;
    py::array_t<double> res_r1;
    py::array_t<double> res_r2;
    std::vector<std::string> cap_net_names;
    std::vector<std::string> res_net_names;
    std::vector<std::string> res_sink_names;
    bool is_last;
};

ComparisonChunk compare_spef_chunk(
    ParsedSpef& spef1,
    ParsedSpef& spef2,
    size_t start_idx,
    size_t chunk_size,
    int num_threads = 0
);

#endif // SPEF_CORE_H
