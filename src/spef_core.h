#ifndef SPEF_CORE_H
#define SPEF_CORE_H

#include <iostream>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <string>
#include <string_view>
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
#include <cstdlib>
#include <cstring>
#include <atomic>
#include <chrono>
#if defined(__unix__) || defined(__APPLE__)
#  include <sys/mman.h>
#  include <sys/stat.h>
#  include <fcntl.h>
#  include <unistd.h>
#endif

namespace py = pybind11;

struct Edge {
    std::string to;
    double weight;
};

// Structured intermediate coupling-cap entry stored during parse.
// Replaces the former "node1|node2|cap_value" string encoding, eliminating
// the serialize-then-parse round-trip in resolve_coupling_caps_to_nets.
struct RawCouplingEntry {
    std::string node1;
    std::string node2;
    double cap_val;
};

struct NetData {
    std::string name;
    double total_cap;
    std::string driver;
    std::vector<std::string> sinks;
    std::unordered_map<std::string, std::vector<Edge>> res_graph;
    std::unordered_map<std::string, double> driver_sink_res_cache;
    std::unordered_map<std::string, double> driver_sink_equiv_res_cache;
    // Structured coupling-cap entries accumulated during CAP section parsing.
    std::vector<RawCouplingEntry> raw_coupling_caps;
    bool cache_valid;
    bool equiv_res_cache_valid;

    NetData() : total_cap(0.0), cache_valid(false), equiv_res_cache_valid(false) {}
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
    double c_scale;  // *C_UNIT coefficient (e.g. 0.001 for "*C_UNIT 0.001 PF")
    double r_scale;  // *R_UNIT coefficient
    
    ParsedSpef() : t_unit("NS"), c_unit("PF"), r_unit("OHM"), l_unit("HENRY"),
                   c_scale(1.0), r_scale(1.0) {}
};

// Dijkstra shortest path - C++ optimized
std::unordered_map<std::string, double> dijkstra_shortest_paths(
    const std::unordered_map<std::string, std::vector<Edge>>& graph,
    const std::string& source
);

// Compute equivalent (Thevenin) resistance between two nodes using nodal analysis
double compute_equivalent_resistance(
    const std::unordered_map<std::string, std::vector<Edge>>& graph,
    const std::string& source,
    const std::string& sink
);

// Post-process coupling capacitances to map nodes to net names
void resolve_coupling_caps_to_nets(ParsedSpef& spef, int max_threads = 0);

// Compute all driver->sink resistances for a net (Dijkstra shortest path)
std::unordered_map<std::string, double> compute_driver_sink_resistances(
    NetData& net
);

// Compute all driver->sink equivalent (Thevenin) resistances for a net
std::unordered_map<std::string, double> compute_driver_sink_equivalent_resistances(
    NetData& net
);

// Dispatch: compute driver-sink resistances by method (0=dijkstra, 1=equivalent)
std::unordered_map<std::string, double> compute_driver_sink_res_by_method(
    NetData& net,
    int res_method
);

// Parse SPEF file - C++ optimized
ParsedSpef parse_spef(const std::string& filepath, int num_threads = 0);

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
    const std::string& output_path,
    int res_method = 0
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
    int num_threads = 0,
    int res_method = 0
);

#endif // SPEF_CORE_H
