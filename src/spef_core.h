#ifndef SPEF_CORE_H
#define SPEF_CORE_H

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
    std::unordered_map<std::string, double> driver_sink_res_cache;
    bool cache_valid;
    
    NetData() : total_cap(0.0), cache_valid(false) {}
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

// Dijkstra shortest path - C++ optimized
std::unordered_map<std::string, double> dijkstra_shortest_paths(
    const std::unordered_map<std::string, std::vector<Edge>>& graph,
    const std::string& source
);

// Compute all driver->sink resistances for a net
std::unordered_map<std::string, double> compute_driver_sink_resistances(
    NetData& net
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
