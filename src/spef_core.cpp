#include "spef_core.h"
#include <iomanip>
#include <sstream>

// ============== Dijkstra Implementation ==============
std::unordered_map<std::string, double> dijkstra_shortest_paths(
    const std::unordered_map<std::string, std::vector<Edge>>& graph,
    const std::string& source
) {
    std::unordered_map<std::string, double> dist;
    std::priority_queue<
        std::pair<double, std::string>,
        std::vector<std::pair<double, std::string>>,
        std::greater<std::pair<double, std::string>>
    > pq;
    
    dist[source] = 0.0;
    pq.push({0.0, source});
    
    while (!pq.empty()) {
        auto [d, node] = pq.top();
        pq.pop();
        
        // Skip if we've already found a shorter path
        if (d > dist[node]) continue;
        
        auto it = graph.find(node);
        if (it == graph.end()) continue;
        
        for (const auto& edge : it->second) {
            double new_dist = d + edge.weight;
            auto existing = dist.find(edge.to);
            if (existing == dist.end() || new_dist < existing->second) {
                dist[edge.to] = new_dist;
                pq.push({new_dist, edge.to});
            }
        }
    }
    
    return dist;
}

// ============== Driver-Sink Resistances ==============
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
    
    // Find best matching driver node
    std::string driver_node = net.driver;
    if (net.res_graph.find(net.driver) == net.res_graph.end()) {
        // Try prefix matching
        size_t colon_pos = net.driver.find(':');
        std::string base = (colon_pos != std::string::npos) ? 
            net.driver.substr(0, colon_pos) : net.driver;
        
        for (const auto& node : net.res_graph) {
            size_t node_colon = node.first.find(':');
            std::string node_base = (node_colon != std::string::npos) ?
                node.first.substr(0, node_colon) : node.first;
            if (node_base == base) {
                driver_node = node.first;
                break;
            }
        }
    }
    
    // Run Dijkstra once from driver
    auto dists = dijkstra_shortest_paths(net.res_graph, driver_node);
    
    // Compute resistances to all sinks
    for (const auto& sink : net.sinks) {
        std::string sink_node = sink;
        
        if (dists.find(sink) == dists.end()) {
            // Try prefix matching
            size_t colon_pos = sink.find(':');
            std::string base = (colon_pos != std::string::npos) ?
                sink.substr(0, colon_pos) : sink;
            
            for (const auto& d : dists) {
                size_t d_colon = d.first.find(':');
                std::string d_base = (d_colon != std::string::npos) ?
                    d.first.substr(0, d_colon) : d.first;
                if (d_base == base) {
                    sink_node = d.first;
                    break;
                }
            }
        }
        
        auto it = dists.find(sink_node);
        if (it != dists.end()) {
            net.driver_sink_res_cache[sink] = it->second;
        }
    }
    
    net.cache_valid = true;
    return net.driver_sink_res_cache;
}

// ============== RECOMMENDATION 3: Pre-computed Pin Prefix Maps ==============
void build_pin_to_node_map(NetData& net) {
    if (net.pin_map_built) return;
    
    net.pin_to_node_cache.clear();
    
    // Helper: find best matching node for a pin
    auto find_best_match = [&](const std::string& pin) -> std::string {
        if (pin.empty()) return pin;
        
        // Try exact match first
        if (net.res_graph.find(pin) != net.res_graph.end()) {
            return pin;
        }
        
        // Try prefix match: split by ':' and match the base
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
    
    // Pre-compute for driver
    net.pin_to_node_cache[net.driver] = find_best_match(net.driver);
    
    // Pre-compute for all sinks
    for (const auto& sink : net.sinks) {
        net.pin_to_node_cache[sink] = find_best_match(sink);
    }
    
    net.pin_map_built = true;
}

// ============== RECOMMENDATION 1: Batch Resistance Computation ==============
std::vector<ResistanceResult> compute_batch_driver_sink_resistances(
    const std::vector<std::string>& net_names,
    ParsedSpef& spef,
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
    auto worker = [&](int thread_id) {
        std::vector<ResistanceResult> local_results;
        // Reserve a proportional share to avoid repeated reallocations.
        // 4 sinks per net is a heuristic; actual fanout may vary.
        size_t est = (net_count / num_threads + 1) * 4;
        local_results.reserve(est);
        
        // Simple work distribution: each thread processes nets % num_threads == thread_id
        for (size_t i = thread_id; i < net_count; i += num_threads) {
            const auto& net_name = net_names[i];
            auto it = spef.nets.find(net_name);
            if (it == spef.nets.end()) continue;
            
            auto& net = it->second;
            
            // Build pin map if not already done
            if (!net.pin_map_built) {
                build_pin_to_node_map(net);
            }
            
            // Compute driver-sink resistances for this net
            auto dists = dijkstra_shortest_paths(net.res_graph, 
                net.pin_to_node_cache[net.driver]);
            
            for (const auto& sink : net.sinks) {
                std::string sink_node = net.pin_to_node_cache[sink];
                auto sink_it = dists.find(sink_node);
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
    threads.reserve(num_threads);
    for (int i = 0; i < num_threads; ++i) {
        threads.emplace_back(worker, i);
    }
    
    for (auto& t : threads) {
        t.join();
    }
    
    return results;
}

// ============== RECOMMENDATION 2: Vectorized Correlation Computation ==============
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
    
    // Use pointers for faster access
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
    
    // Check for sufficient variance
    if (result.std_dev_x < 1e-15 || result.std_dev_y < 1e-15) {
        return result;  // Not enough variance
    }
    
    result.pearson = cov_xy / (std::sqrt(std::max(0.0, var_x)) * std::sqrt(std::max(0.0, var_y)));
    result.valid = true;
    
    return result;
}

// ============== RECOMMENDATION 4: Streaming Comparison Mode ==============
void compare_spef_streaming(
    ParsedSpef& spef1,
    ParsedSpef& spef2,
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
    
    // Process in parallel: each thread accumulates results locally, then
    // holds the callback mutex once to flush them.  This reduces lock
    // contention from O(nets * sinks) acquires down to O(num_threads).
    std::mutex callback_mutex;

    auto worker = [&](int thread_id) {
        std::vector<CapComparisonData> local_caps;
        std::vector<ResComparisonData> local_res;

        for (size_t i = thread_id; i < common_nets.size(); i += num_threads) {
            const auto& net_name = common_nets[i];
            // Use .at() instead of [] to avoid UB on concurrent non-const
            // access to the shared unordered_map ([] can insert, which is
            // not safe across threads even for different keys).
            auto& net1 = spef1.nets.at(net_name);
            auto& net2 = spef2.nets.at(net_name);

            local_caps.push_back({net_name, net1.total_cap, net2.total_cap});

            // Resistance comparison
            auto res1 = compute_driver_sink_resistances(net1);
            auto res2 = compute_driver_sink_resistances(net2);

            // Find common sinks by extracting keys
            std::vector<std::string> sinks1_vec, sinks2_vec;
            sinks1_vec.reserve(res1.size());
            sinks2_vec.reserve(res2.size());
            for (const auto& [sink, _] : res1) sinks1_vec.push_back(sink);
            for (const auto& [sink, _] : res2) sinks2_vec.push_back(sink);

            std::sort(sinks1_vec.begin(), sinks1_vec.end());
            std::sort(sinks2_vec.begin(), sinks2_vec.end());

            std::vector<std::string> common_sinks;
            std::set_intersection(
                sinks1_vec.begin(), sinks1_vec.end(),
                sinks2_vec.begin(), sinks2_vec.end(),
                std::back_inserter(common_sinks)
            );

            for (const auto& sink : common_sinks) {
                local_res.push_back({net_name, net1.driver, sink,
                                     res1[sink], res2[sink]});
            }
        }

        // Flush local results through the callbacks once per thread
        {
            std::lock_guard<std::mutex> lock(callback_mutex);
            for (const auto& cap : local_caps) on_cap(cap);
            for (const auto& res : local_res) on_res(res);
        }
    };
    
    // Create and join threads
    std::vector<std::thread> threads;
    threads.reserve(num_threads);
    for (int i = 0; i < num_threads; ++i) {
        threads.emplace_back(worker, i);
    }
    
    for (auto& t : threads) {
        t.join();
    }
}

// ============== SPEF Parser ==============
static inline std::string trim(const std::string& s) {
    size_t start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return "";
    size_t end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
}

static inline std::string strip_quotes(const std::string& s) {
    if (s.size() >= 2 && s.front() == '"' && s.back() == '"') {
        return s.substr(1, s.size() - 2);
    }
    return s;
}

static inline double parse_float(const std::string& s) {
    try {
        return std::stod(s);
    } catch (...) {
        // Remove non-numeric characters
        std::string cleaned;
        for (char c : s) {
            if (c == '-' || c == '.' || c == 'e' || c == 'E' || std::isdigit(c)) {
                cleaned += c;
            }
        }
        if (cleaned.empty()) return 0.0;
        try {
            return std::stod(cleaned);
        } catch (...) {
            return 0.0;
        }
    }
}

ParsedSpef parse_spef(const std::string& filepath) {
    ParsedSpef spef;
    
    std::ifstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file: " + filepath);
    }
    
    enum Section { SEC_NONE, SEC_CONN, SEC_CAP, SEC_RES };
    Section section = SEC_NONE;
    
    bool in_name_map = false;
    std::string current_net_name;
    NetData* current_net = nullptr;
    bool r_is_kohm = false;
    size_t net_count = 0;
    auto t_start = std::chrono::steady_clock::now();
    
    std::string line;
    size_t line_num = 0;

    // Resolve tokens like *123 or *123:A using NAME_MAP.
    auto resolve_name_token = [&](const std::string& token) -> std::string {
        if (token.empty()) return token;

        std::string out = token;
        if (out[0] == '*') {
            size_t colon = out.find(':');
            if (colon != std::string::npos) {
                std::string base = out.substr(0, colon);
                std::string suffix = out.substr(colon);  // keep ':' + suffix
                auto it = spef.name_map.find(base);
                if (it != spef.name_map.end()) {
                    out = it->second + suffix;
                }
            } else {
                auto it = spef.name_map.find(out);
                if (it != spef.name_map.end()) {
                    out = it->second;
                }
            }
        }

        size_t pos;
        while ((pos = out.find("\\[")) != std::string::npos) out.replace(pos, 2, "[");
        while ((pos = out.find("\\]")) != std::string::npos) out.replace(pos, 2, "]");
        return out;
    };
    
    while (std::getline(file, line)) {
        line_num++;
        // Skip empty lines
        if (line.empty()) continue;
        
        // Find comment
        size_t comment_pos = line.find("//");
        std::string code_part = (comment_pos != std::string::npos) ? 
            line.substr(0, comment_pos) : line;
        std::string comment_part = (comment_pos != std::string::npos) ?
            trim(line.substr(comment_pos + 2)) : "";
        
        code_part = trim(code_part);
        if (code_part.empty()) continue;
        
        // Quick check for section headers
        if (code_part[0] == '*') {
            if (code_part.find("*NAME_MAP") == 0) {
                in_name_map = true;
                section = SEC_NONE;
                continue;
            }
            if (code_part.find("*PORTS") == 0) {
                in_name_map = false;
                continue;
            }
            if (code_part.find("*D_NET") == 0) {
                // New net
                std::istringstream iss(code_part);
                std::string token, net_id, net_name, total_cap_str;
                iss >> token >> net_id >> total_cap_str;
                
                // Reserve map capacity on the first net, using name_map size
                // as a proxy for total net count to avoid 1M+ rehashes.
                if (net_count == 0 && !spef.name_map.empty()) {
                    spef.nets.reserve(spef.name_map.size() * 5 / 4);
                }

                // *D_NET always ends the *NAME_MAP section.  Without this
                // reset, *I/*O/*P CONN entries would be silently swallowed
                // by the name_map handler for any net that follows the map.
                in_name_map = false;

                // Resolve net name
                std::string resolved_name = net_id;
                if (net_id[0] == '*') {
                    auto it = spef.name_map.find(net_id);
                    if (it != spef.name_map.end()) {
                        resolved_name = it->second;
                    }
                }
                
                current_net_name = resolved_name;
                current_net = &spef.nets[current_net_name];
                current_net->name = resolved_name;
                current_net->total_cap = parse_float(total_cap_str);
                section = SEC_NONE;
                net_count++;
                if (net_count % 5000 == 0) {
                    auto t_now = std::chrono::steady_clock::now();
                    double elapsed = std::chrono::duration<double>(t_now - t_start).count();
                    std::cout << "[" << filepath << "] parsed " << net_count << " nets... (C++/single, " << elapsed << "s)" << std::endl;
                }
                continue;
            }
            if (code_part.find("*END") == 0) {
                current_net = nullptr;
                section = SEC_NONE;
                continue;
            }
            if (code_part.find("*CONN") == 0) {
                section = SEC_CONN;
                continue;
            }
            if (code_part.find("*CAP") == 0) {
                section = SEC_CAP;
                continue;
            }
            if (code_part.find("*RES") == 0) {
                section = SEC_RES;
                continue;
            }
            if (code_part.find("*R_UNIT") == 0) {
                std::istringstream iss(code_part);
                std::string token, num, unit;
                iss >> token >> num >> unit;
                spef.r_unit = unit;
                r_is_kohm = (unit == "KOHM");
                continue;
            }
            if (code_part.find("*C_UNIT") == 0) {
                std::istringstream iss(code_part);
                std::string token, num, unit;
                iss >> token >> num >> unit;
                spef.c_unit = unit;
                continue;
            }
            if (code_part.find("*T_UNIT") == 0) {
                std::istringstream iss(code_part);
                std::string token, num, unit;
                iss >> token >> num >> unit;
                spef.t_unit = unit;
                continue;
            }
            if (code_part.find("*L_UNIT") == 0) {
                std::istringstream iss(code_part);
                std::string token, num, unit;
                iss >> token >> num >> unit;
                spef.l_unit = unit;
                continue;
            }
        }
        
        // Handle NAME_MAP entries
        if (in_name_map) {
            if (!code_part.empty() && code_part[0] == '*') {
                std::istringstream iss(code_part);
                std::string key, value;
                iss >> key >> value;
                // Only process keys that look like *NUMBER (e.g., *1, *2)
                if (!key.empty() && key.size() >= 2 && key[1] >= '0' && key[1] <= '9' && !value.empty()) {
                    value = strip_quotes(value);
                    // Unescape \[ and \]
                    size_t pos;
                    while ((pos = value.find("\\[")) != std::string::npos) {
                        value.replace(pos, 2, "[");
                    }
                    while ((pos = value.find("\\]")) != std::string::npos) {
                        value.replace(pos, 2, "]");
                    }
                    spef.name_map[key] = value;
                }
            } else {
                in_name_map = false;
            }
            continue;
        }
        
        // Process based on current section
        if (current_net == nullptr) continue;
        
        std::istringstream iss(code_part);
        std::vector<std::string> tokens;
        std::string tok;
        while (iss >> tok) {
            tokens.push_back(tok);
        }
        
        if (tokens.empty()) continue;
        
        if (section == SEC_RES && tokens.size() >= 4) {
            // RES entry: *idx node1 node2 R_value
            try {
                int idx = std::stoi(tokens[0]);
                std::string node1 = resolve_name_token(tokens[1]);
                std::string node2 = resolve_name_token(tokens[2]);
                double rval = parse_float(tokens[3]);
                if (r_is_kohm) rval *= 1000.0;
                
                current_net->res_graph[node1].push_back({node2, rval});
                current_net->res_graph[node2].push_back({node1, rval});
            } catch (...) {
                // Not a RES entry, might be section header
                if (tokens[0] == "*CAP") section = SEC_CAP;
                else if (tokens[0] == "*CONN") section = SEC_CONN;
                else if (tokens[0] == "*END") { current_net = nullptr; section = SEC_NONE; }
            }
        }
        else if (section == SEC_CAP && tokens.size() >= 3) {
            try {
                int idx = std::stoi(tokens[0]);
                // It's a CAP entry, we only need total_cap from D_NET
                // Skip detailed parsing for speed
            } catch (...) {
                if (tokens[0] == "*CONN") section = SEC_CONN;
                else if (tokens[0] == "*RES") section = SEC_RES;
                else if (tokens[0] == "*END") { current_net = nullptr; section = SEC_NONE; }
            }
        }
        else if (section == SEC_CONN && tokens.size() >= 3) {
            std::string pin = resolve_name_token(tokens[1]);
            // Direction is the first token that starts with O, B, or I
            std::string dir;
            for (size_t i = 2; i < tokens.size(); i++) {
                if (!tokens[i].empty() && (tokens[i][0] == 'O' || tokens[i][0] == 'B' || tokens[i][0] == 'I')) {
                    dir = tokens[i];
                    break;
                }
            }
            
            if (!dir.empty() && (dir[0] == 'O' || dir[0] == 'B')) {
                if (current_net->driver.empty()) {
                    current_net->driver = pin;
                }
            } else if (!dir.empty() && (dir[0] == 'I')) {
                current_net->sinks.push_back(pin);
            }
        }
        else if (section == SEC_NONE) {
            // Look for section header
            if (!tokens.empty() && tokens[0][0] == '*') {
                std::string header = tokens[0];
                if (header == "*CONN") {
                    section = SEC_CONN;
                    continue;
                }
                else if (header == "*CAP") {
                    section = SEC_CAP;
                    continue;
                }
                else if (header == "*RES") {
                    section = SEC_RES;
                    continue;
                }
                else if (header == "*END") {
                    current_net = nullptr;
                    continue;
                }
                // Also handle *I (input) and *P (port) as CONN entries when no explicit *CONN header
                else if ((header == "*I" || header == "*P") && tokens.size() >= 3) {
                    section = SEC_CONN;
                    // Process this line as CONN
                    std::string pin = resolve_name_token(tokens[1]);
                    std::string dir;
                    for (size_t i = 2; i < tokens.size(); i++) {
                        if (!tokens[i].empty() && (tokens[i][0] == 'O' || tokens[i][0] == 'B' || tokens[i][0] == 'I')) {
                            dir = tokens[i];
                            break;
                        }
                    }
                    if (!dir.empty() && (dir[0] == 'O' || dir[0] == 'B')) {
                        if (current_net && current_net->driver.empty()) {
                            current_net->driver = pin;
                        }
                    } else if (!dir.empty() && dir[0] == 'I') {
                        if (current_net) current_net->sinks.push_back(pin);
                    }
                    continue;
                }
            }
        }
    }
    
    file.close();
    auto t_end = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(t_end - t_start).count();
    std::cout << "[" << filepath << "] finished parsing " << net_count << " nets in " << elapsed << "s (C++/single)" << std::endl;
    return spef;
}

// ============== Parallel Single-File SPEF Parser ==============
//
// Two-phase strategy for 1M+ nets:
//   Phase 1 (single-threaded, I/O-bound):
//     - Read entire file into a vector<string> of lines.
//     - Parse header directives (*_UNIT) and the *NAME_MAP section.
//     - Record the [start, end) line-index range for every *D_NET..*END block.
//   Phase 2 (multi-threaded, CPU-bound):
//     - Divide the net-block list into contiguous chunks, one per thread.
//     - Each thread independently parses its net blocks; name_map is
//       accessed read-only so no synchronisation is needed.
//     - Results are stored in a pre-sized vector (one slot per net block),
//       so threads write to disjoint memory without any mutex.
//   Phase 3 (single-threaded):
//     - Move per-block NetData into the final ParsedSpef::nets map.
//
// Expected speedup over parse_spef(): roughly linear in num_threads for
// files whose per-net parsing is the bottleneck (typically >~100 K nets).
ParsedSpef parse_spef_mt(const std::string& filepath, int num_threads) {
    if (num_threads <= 0) {
        num_threads = static_cast<int>(std::thread::hardware_concurrency());
        if (num_threads <= 0) num_threads = 4;
    }

    auto t_start = std::chrono::steady_clock::now();

    // ---- Phase 1: load file, parse header/name_map, find net boundaries ----
    std::vector<std::string> all_lines;
    all_lines.reserve(1 << 22);  // ~4.2M lines – enough for most large files
    {
        std::ifstream file(filepath);
        if (!file.is_open())
            throw std::runtime_error("Cannot open file: " + filepath);
        std::string line;
        while (std::getline(file, line))
            all_lines.push_back(std::move(line));
    }

    ParsedSpef spef;
    bool in_name_map = false;
    bool r_is_kohm   = false;

    struct Boundary { size_t start; size_t end; };
    std::vector<Boundary> boundaries;
    boundaries.reserve(1 << 20);  // ~1M nets (1,048,576 slots)

    size_t cur_net_start = SIZE_MAX;

    // Helper: trim a raw line and strip // comments; returns trimmed string_view
    auto trimmed_code = [](const std::string& raw) -> std::string_view {
        size_t s = 0;
        while (s < raw.size() && (raw[s] == ' ' || raw[s] == '\t' || raw[s] == '\r')) ++s;
        size_t e = raw.find("//", s);
        if (e == std::string::npos) e = raw.size();
        while (e > s && (raw[e-1] == ' ' || raw[e-1] == '\t' || raw[e-1] == '\r')) --e;
        return std::string_view(raw.data() + s, e - s);
    };

    for (size_t li = 0; li < all_lines.size(); ++li) {
        std::string_view code = trimmed_code(all_lines[li]);
        if (code.empty()) continue;

        if (code[0] == '*') {
            if (code.find("*NAME_MAP") == 0) { in_name_map = true;  continue; }
            if (code.find("*PORTS")    == 0) { in_name_map = false; continue; }
            if (code.find("*D_NET")    == 0) { in_name_map = false; cur_net_start = li; continue; }
            if (code.find("*END")      == 0) {
                if (cur_net_start != SIZE_MAX) {
                    boundaries.push_back({cur_net_start, li + 1});
                    cur_net_start = SIZE_MAX;
                }
                continue;
            }
            // Unit directives
            auto parse_unit_line = [&](std::string& dest) {
                std::string code_str(code);
                std::istringstream iss(code_str);
                std::string tok, num, unit;
                if (iss >> tok >> num >> unit) dest = unit;
                in_name_map = false;
            };
            if (code.find("*R_UNIT") == 0) {
                parse_unit_line(spef.r_unit);
                r_is_kohm = (spef.r_unit == "KOHM");
                continue;
            }
            if (code.find("*C_UNIT") == 0) { parse_unit_line(spef.c_unit); continue; }
            if (code.find("*T_UNIT") == 0) { parse_unit_line(spef.t_unit); continue; }
            if (code.find("*L_UNIT") == 0) { parse_unit_line(spef.l_unit); continue; }
        }

        if (in_name_map) {
            if (!code.empty() && code[0] == '*') {
                size_t sp = code.find(' ');
                if (sp != std::string_view::npos) {
                    std::string key(code.data(), sp);
                    size_t vs = sp + 1;
                    while (vs < code.size() && (code[vs] == ' ' || code[vs] == '\t')) ++vs;
                    std::string value(code.data() + vs, code.size() - vs);
                    if (key.size() >= 2 && key[1] >= '0' && key[1] <= '9' && !value.empty()) {
                        value = strip_quotes(value);
                        size_t pos;
                        while ((pos = value.find("\\[")) != std::string::npos) value.replace(pos, 2, "[");
                        while ((pos = value.find("\\]")) != std::string::npos) value.replace(pos, 2, "]");
                        spef.name_map[key] = std::move(value);
                    }
                }
            } else {
                in_name_map = false;
            }
        }
    }

    size_t net_count = boundaries.size();
    spef.nets.reserve(net_count * 5 / 4);

    {
        auto t1 = std::chrono::steady_clock::now();
        std::cout << "[" << filepath << "] parse_spef_mt Phase 1: found "
                  << net_count << " nets, name_map=" << spef.name_map.size()
                  << " in " << std::chrono::duration<double>(t1 - t_start).count()
                  << "s" << std::endl;
    }

    if (net_count == 0) return spef;

    // ---- Phase 2: parallel per-net-block parsing ----
    std::vector<NetData> net_results(net_count);

    // Token resolver – name_map is read-only here so safe to call from threads
    auto resolve_token = [&](const std::string& token) -> std::string {
        if (token.empty()) return token;
        std::string out = token;
        if (out[0] == '*') {
            size_t colon = out.find(':');
            if (colon != std::string::npos) {
                std::string base   = out.substr(0, colon);
                std::string suffix = out.substr(colon);
                auto it = spef.name_map.find(base);
                if (it != spef.name_map.end()) out = it->second + suffix;
            } else {
                auto it = spef.name_map.find(out);
                if (it != spef.name_map.end()) out = it->second;
            }
        }
        size_t pos;
        while ((pos = out.find("\\[")) != std::string::npos) out.replace(pos, 2, "[");
        while ((pos = out.find("\\]")) != std::string::npos) out.replace(pos, 2, "]");
        return out;
    };

    // Parse one net block (lines [start, end)) into net_results[block_idx]
    auto parse_net_block = [&](size_t block_idx) {
        const size_t start_li = boundaries[block_idx].start;
        const size_t end_li   = boundaries[block_idx].end;
        NetData& net = net_results[block_idx];

        enum Section { SEC_NONE, SEC_CONN, SEC_CAP, SEC_RES };
        Section section = SEC_NONE;

        for (size_t li = start_li; li < end_li; ++li) {
            const std::string& raw = all_lines[li];
            if (raw.empty()) continue;

            size_t s = raw.find_first_not_of(" \t\r\n");
            if (s == std::string::npos) continue;

            size_t cpos     = raw.find("//", s);
            size_t code_end = (cpos != std::string::npos) ? cpos : raw.size();
            while (code_end > s &&
                   (raw[code_end-1]==' '||raw[code_end-1]=='\t'||raw[code_end-1]=='\r'))
                --code_end;
            if (s >= code_end) continue;

            // Fast tokeniser (avoids istringstream overhead)
            std::vector<std::string> tokens;
            size_t p = s;
            while (p < code_end) {
                while (p < code_end && (raw[p]==' '||raw[p]=='\t')) ++p;
                if (p >= code_end) break;
                size_t q = p;
                while (q < code_end && raw[q]!=' ' && raw[q]!='\t') ++q;
                tokens.emplace_back(raw.data()+p, raw.data()+q);
                p = q;
            }
            if (tokens.empty()) continue;

            const std::string& t0 = tokens[0];

            if (t0[0] == '*') {
                if (t0 == "*D_NET" && tokens.size() >= 3) {
                    net.name      = resolve_token(tokens[1]);
                    net.total_cap = parse_float(tokens[2]);
                    continue;
                }
                if (t0 == "*CONN") { section = SEC_CONN; continue; }
                if (t0 == "*CAP")  { section = SEC_CAP;  continue; }
                if (t0 == "*RES")  { section = SEC_RES;  continue; }
                if (t0 == "*END")  { break; }
                // *I / *P may appear before an explicit *CONN header
                if ((t0 == "*I" || t0 == "*P") && section == SEC_NONE)
                    section = SEC_CONN;
                // fall through to CONN handler below
            }

            if (section == SEC_CONN && tokens.size() >= 3) {
                std::string pin = resolve_token(tokens[1]);
                std::string dir;
                for (size_t i = 2; i < tokens.size(); i++) {
                    if (!tokens[i].empty() &&
                        (tokens[i][0]=='O'||tokens[i][0]=='B'||tokens[i][0]=='I')) {
                        dir = tokens[i];
                        break;
                    }
                }
                if (!dir.empty() && (dir[0]=='O'||dir[0]=='B')) {
                    if (net.driver.empty()) net.driver = pin;
                } else if (!dir.empty() && dir[0]=='I') {
                    net.sinks.push_back(pin);
                }
            } else if (section == SEC_RES && tokens.size() >= 4) {
                try {
                    std::stoi(tokens[0]);  // verify index token
                    std::string node1 = resolve_token(tokens[1]);
                    std::string node2 = resolve_token(tokens[2]);
                    double rval = parse_float(tokens[3]);
                    if (r_is_kohm) rval *= 1000.0;
                    net.res_graph[node1].push_back({node2, rval});
                    net.res_graph[node2].push_back({node1, rval});
                } catch (...) {
                    if (t0 == "*CAP")  section = SEC_CAP;
                    else if (t0 == "*CONN") section = SEC_CONN;
                    else if (t0 == "*END")  break;
                }
            }
        }
    };

    if (num_threads > static_cast<int>(net_count))
        num_threads = static_cast<int>(net_count);

    std::vector<std::thread> threads;
    threads.reserve(num_threads);
    for (int t = 0; t < num_threads; ++t) {
        threads.emplace_back([&, t]() {
            // Contiguous chunk assignment gives better cache locality than
            // the interleaved (round-robin) distribution.
            size_t chunk      = (net_count + num_threads - 1) / num_threads;
            size_t begin_idx  = static_cast<size_t>(t) * chunk;
            size_t end_idx    = std::min(begin_idx + chunk, net_count);
            for (size_t i = begin_idx; i < end_idx; ++i)
                parse_net_block(i);
        });
    }
    for (auto& th : threads) th.join();

    {
        auto t2 = std::chrono::steady_clock::now();
        std::cout << "[" << filepath << "] parse_spef_mt Phase 2: parallel parse done in "
                  << std::chrono::duration<double>(t2 - t_start).count() << "s" << std::endl;
    }

    // ---- Phase 3: merge net_results into spef.nets ----
    for (size_t i = 0; i < net_count; ++i) {
        if (!net_results[i].name.empty()) {
            // Move the name out first so it can be used as the map key
            // without copying; the moved-from .name field is then discarded.
            std::string key = std::move(net_results[i].name);
            spef.nets.emplace(std::move(key), std::move(net_results[i]));
        }
    }

    auto t_end2 = std::chrono::steady_clock::now();
    std::cout << "[" << filepath << "] parse_spef_mt finished: "
              << spef.nets.size() << " nets in "
              << std::chrono::duration<double>(t_end2 - t_start).count()
              << "s (C++/mt, " << num_threads << " threads)" << std::endl;
    return spef;
}


void shuffle_spef(const std::string& input_path, const std::string& output_path, int seed) {
    std::srand(seed);
    
    std::ifstream input(input_path);
    std::ofstream output(output_path);
    
    if (!input.is_open()) {
        throw std::runtime_error("Cannot open input file: " + input_path);
    }
    if (!output.is_open()) {
        throw std::runtime_error("Cannot open output file: " + output_path);
    }
    
    // First pass: collect NAME_MAP and net IDs
    std::unordered_map<std::string, std::string> name_map;
    std::vector<std::string> net_ids;
    std::unordered_set<std::string> net_ids_set;
    
    std::regex re_net_id("^\\*\\d+$");
    std::regex re_nid("\\*(\\d+)");
    
    std::string line;
    bool in_name_map = false;
    
    // Reset and read again for net IDs
    input.clear();
    input.seekg(0);
    
    while (std::getline(input, line)) {
        std::string trimmed = trim(line);
        
        if (trimmed.find("*NAME_MAP") == 0) {
            in_name_map = true;
            continue;
        }
        
        if (in_name_map) {
            std::istringstream iss(trimmed);
            std::string key, value;
            if (iss >> key >> value) {
                if (std::regex_match(key, re_net_id)) {
                    name_map[key] = strip_quotes(value);
                } else {
                    in_name_map = false;
                }
            }
        }
        
        if (trimmed.find("*D_NET") == 0) {
            std::istringstream iss(trimmed);
            std::string token, net_id;
            iss >> token >> net_id;
            if (std::regex_match(net_id, re_net_id) && net_ids_set.find(net_id) == net_ids_set.end()) {
                net_ids.push_back(net_id);
                net_ids_set.insert(net_id);
            }
        }
    }
    
    if (net_ids.size() < 2) {
        // Just copy the file
        input.clear();
        input.seekg(0);
        while (std::getline(input, line)) {
            output << line << "\n";
        }
        input.close();
        output.close();
        return;
    }
    
    // Build shuffled assignment
    std::vector<std::string> net_names;
    for (const auto& nid : net_ids) {
        auto it = name_map.find(nid);
        net_names.push_back((it != name_map.end()) ? it->second : nid);
    }
    
    // Shuffle
    std::vector<std::string> shuffled_names = net_names;
    for (int i = 0; i < 20; i++) {
        std::random_shuffle(shuffled_names.begin(), shuffled_names.end());
        if (shuffled_names != net_names) break;
    }
    
    // Build substitution map
    std::unordered_map<std::string, std::string> subst;
    for (size_t i = 0; i < net_ids.size(); i++) {
        subst[net_ids[i]] = net_ids[i]; // placeholder
        
        // Find which old_nid has shuffled_names[i]
        for (size_t j = 0; j < net_names.size(); j++) {
            if (net_names[j] == shuffled_names[i]) {
                subst[net_ids[i]] = net_ids[j];
                break;
            }
        }
    }
    
    // Second pass: apply substitution
    input.clear();
    input.seekg(0);
    
    // Pre-build a simpler pattern for manual replacement
    while (std::getline(input, line)) {
        if (line.find('*') != std::string::npos) {
            std::string result;
            size_t pos = 0;
            size_t star_pos;
            
            while ((star_pos = line.find('*', pos)) != std::string::npos) {
                // Add text before the star
                result += line.substr(pos, star_pos - pos);
                
                // Check if it's followed by digits
                bool is_net_id = true;
                for (size_t i = star_pos + 1; i < line.size() && i < star_pos + 10; i++) {
                    if (!isdigit(line[i])) {
                        is_net_id = (line[i] == '*'); // Another star is okay
                        break;
                    }
                    if (i == line.size() - 1 || !isdigit(line[i + 1])) {
                        // End of digit sequence
                        std::string num = line.substr(star_pos + 1, i - star_pos);
                        std::string old_nid = "*" + num;
                        auto it = subst.find(old_nid);
                        if (it != subst.end()) {
                            result += it->second;
                        } else {
                            result += old_nid;
                        }
                        pos = i + 1;
                        break;
                    }
                }
                
                if (!is_net_id || star_pos + 1 >= line.size()) {
                    result += '*';
                    pos = star_pos + 1;
                }
            }
            
            // Add remaining
            result += line.substr(pos);
            line = result;
        }
        output << line << "\n";
    }
    
    input.close();
    output.close();
}

// ============== BACKMARK IMPLEMENTATIONS ==============

std::unordered_map<std::string, double> parse_backmark_cap_data(const std::string& path) {
    std::unordered_map<std::string, double> cap_map;
    
    std::ifstream file(path);
    if (!file.is_open()) {
        return cap_map;
    }
    
    std::string line;
    while (std::getline(file, line)) {
        // Skip empty lines and comments
        if (line.empty() || line[0] == '#') continue;
        
        std::istringstream iss(line);
        std::string net_key, dummy1, val_str;
        if (!(iss >> net_key >> dummy1 >> val_str)) continue;
        
        try {
            double new_cap = std::stod(val_str);
            cap_map[net_key] = new_cap;
        } catch (...) {
            continue;
        }
    }
    
    return cap_map;
}

std::unordered_map<std::string, std::unordered_map<std::string, double>> parse_backmark_res_data(const std::string& path) {
    std::unordered_map<std::string, std::unordered_map<std::string, double>> res_map;
    
    std::ifstream file(path);
    if (!file.is_open()) {
        return res_map;
    }
    
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        
        std::istringstream iss(line);
        std::string net_key, driver, sink, r_old, r_new;
        if (!(iss >> net_key >> driver >> sink >> r_old >> r_new)) continue;
        
        try {
            double new_r = std::stod(r_new);
            res_map[net_key][sink] = new_r;
        } catch (...) {
            continue;
        }
    }
    
    return res_map;
}

std::string resolve_spef_token(const std::string& tok, 
                               const std::unordered_map<std::string, std::string>& name_map) {
    if (tok.empty()) return tok;
    
    std::string out = tok;
    if (out[0] == '*') {
        size_t colon = out.find(':');
        std::string base, suffix;
        if (colon != std::string::npos) {
            base = out.substr(0, colon);
            suffix = out.substr(colon);
        } else {
            base = out;
            suffix = "";
        }
        
        auto it = name_map.find(base);
        if (it != name_map.end()) {
            out = it->second + suffix;
        }
    }
    
    // Unescape \[ and \]
    size_t pos;
    while ((pos = out.find("\\[")) != std::string::npos) out.replace(pos, 2, "[");
    while ((pos = out.find("\\]")) != std::string::npos) out.replace(pos, 2, "]");
    
    return out;
}

std::unordered_map<std::string, std::unordered_map<std::string, double>> compute_res_segment_scales(
    NetData& net,
    const std::unordered_map<std::string, double>& sink_ratios,
    double avg_ratio
) {
    std::unordered_map<std::string, std::unordered_map<std::string, double>> result;
    
    if (net.driver.empty() || sink_ratios.empty()) {
        return result;
    }
    
    // Find driver node
    std::string driver_node = net.driver;
    if (net.res_graph.find(net.driver) == net.res_graph.end()) {
        size_t colon_pos = net.driver.find(':');
        std::string base = (colon_pos != std::string::npos) ? 
            net.driver.substr(0, colon_pos) : net.driver;
        
        for (const auto& node : net.res_graph) {
            size_t node_colon = node.first.find(':');
            std::string node_base = (node_colon != std::string::npos) ?
                node.first.substr(0, node_colon) : node.first;
            if (node_base == base) {
                driver_node = node.first;
                break;
            }
        }
    }
    
    if (net.res_graph.find(driver_node) == net.res_graph.end()) {
        return result;
    }
    
    // Build prefix map for sink resolution
    auto find_best_sink_node = [&](const std::string& sink_pin) -> std::string {
        if (net.res_graph.find(sink_pin) != net.res_graph.end()) {
            return sink_pin;
        }
        size_t colon_pos = sink_pin.find(':');
        std::string base = (colon_pos != std::string::npos) ? 
            sink_pin.substr(0, colon_pos) : sink_pin;
        
        for (const auto& node : net.res_graph) {
            size_t node_colon = node.first.find(':');
            std::string node_base = (node_colon != std::string::npos) ?
                node.first.substr(0, node_colon) : node.first;
            if (node_base == base) {
                return node.first;
            }
        }
        return sink_pin;
    };
    
    // Map sink pins to graph nodes
    std::unordered_map<std::string, double> sink_node_to_ratio;
    for (const auto& [sink_pin, ratio] : sink_ratios) {
        std::string sink_node = find_best_sink_node(sink_pin);
        if (net.res_graph.find(sink_node) != net.res_graph.end()) {
            sink_node_to_ratio[sink_node] = ratio;
        }
    }
    
    if (sink_node_to_ratio.empty()) {
        return result;
    }
    
    // BFS from driver to build spanning tree
    std::unordered_map<std::string, std::string> parent;
    std::unordered_map<std::string, std::vector<std::string>> children;
    std::vector<std::string> order;
    std::queue<std::string> q;
    
    parent[driver_node] = "";
    q.push(driver_node);
    order.push_back(driver_node);
    
    while (!q.empty()) {
        std::string node = q.front();
        q.pop();
        children[node] = {};
        
        auto it = net.res_graph.find(node);
        if (it != net.res_graph.end()) {
            for (const auto& edge : it->second) {
                if (parent.find(edge.to) == parent.end()) {
                    parent[edge.to] = node;
                    children[node].push_back(edge.to);
                    q.push(edge.to);
                    order.push_back(edge.to);
                }
            }
        }
    }
    
    // Post-order: compute sinks in each subtree
    std::unordered_map<std::string, std::unordered_set<std::string>> sinks_below;
    for (auto it = order.rbegin(); it != order.rend(); ++it) {
        std::string node = *it;
        std::unordered_set<std::string> s;
        
        if (sink_node_to_ratio.find(node) != sink_node_to_ratio.end()) {
            s.insert(node);
        }
        
        for (const auto& child : children[node]) {
            auto child_sinks = sinks_below.find(child);
            if (child_sinks != sinks_below.end()) {
                for (const auto& sink : child_sinks->second) {
                    s.insert(sink);
                }
            }
        }
        
        sinks_below[node] = std::move(s);
    }
    
    // Assign scale factors to tree edges
    for (const auto& [child_node, par_node] : parent) {
        if (par_node.empty()) continue;
        
        double scale;
        auto sinks_it = sinks_below.find(child_node);
        if (sinks_it != sinks_below.end() && sinks_it->second.size() == 1) {
            scale = sink_node_to_ratio[*sinks_it->second.begin()];
        } else {
            scale = avg_ratio;
        }
        
        result[par_node][child_node] = scale;
        result[child_node][par_node] = scale;
    }
    
    return result;
}

std::string fmt_float(double val) {
    if (val == 0.0) return "0";
    if (std::abs(val) < 1e-4) {
        std::ostringstream oss;
        oss << std::scientific << std::setprecision(6) << val;
        return oss.str();
    }
    std::ostringstream oss;
    oss << std::setprecision(6) << std::fixed << val;
    std::string s = oss.str();
    // Remove trailing zeros after decimal point
    size_t dot = s.find('.');
    if (dot != std::string::npos) {
        size_t end = s.length() - 1;
        while (end > dot && s[end] == '0') end--;
        if (end == dot) s = s.substr(0, dot);
        else s = s.substr(0, end + 1);
    }
    return s;
}

void backmark_spef(
    const std::string& spef_path,
    const std::string& cap_data_path,
    const std::string& res_data_path,
    const std::string& output_path
) {
    std::cout << "[backmark] Parsing SPEF..." << std::endl;
    // Parse SPEF
    ParsedSpef spef = parse_spef(spef_path);
    
    // Build reverse name_map: net_name -> net_id
    std::unordered_map<std::string, std::string> reverse_name_map;
    for (const auto& [net_id, net_name] : spef.name_map) {
        reverse_name_map[net_name] = net_id;
    }
    
    // Load cap data
    std::unordered_map<std::string, double> cap_ratio;
    std::unordered_map<std::string, double> new_total_caps;
    
    if (!cap_data_path.empty()) {
        auto raw_cap = parse_backmark_cap_data(cap_data_path);
        for (const auto& [key, new_cap] : raw_cap) {
            std::string net_name = key;
            if (key[0] == '*') {
                auto it = spef.name_map.find(key);
                if (it != spef.name_map.end()) {
                    net_name = it->second;
                }
            }
            
            auto net_it = spef.nets.find(net_name);
            if (net_it == spef.nets.end()) continue;
            
            double old_cap = net_it->second.total_cap;
            new_total_caps[net_name] = new_cap;
            cap_ratio[net_name] = (old_cap != 0.0) ? (new_cap / old_cap) : 1.0;
        }
    }
    
    // Load res data
    std::unordered_map<std::string, std::unordered_map<std::string, std::unordered_map<std::string, double>>> res_segment_scales;
    std::unordered_map<std::string, double> res_avg_ratio;
    
    if (!res_data_path.empty()) {
        auto raw_res = parse_backmark_res_data(res_data_path);
        for (const auto& [key, sink_map] : raw_res) {
            std::string net_name = key;
            if (key[0] == '*') {
                auto it = spef.name_map.find(key);
                if (it != spef.name_map.end()) {
                    net_name = it->second;
                }
            }
            
            auto net_it = spef.nets.find(net_name);
            if (net_it == spef.nets.end()) continue;
            
            NetData& net = net_it->second;
            auto old_dr = compute_driver_sink_resistances(net);
            if (old_dr.empty()) continue;
            
            std::unordered_map<std::string, double> sink_ratios;
            for (const auto& [sink, new_r] : sink_map) {
                auto old_r_it = old_dr.find(sink);
                if (old_r_it != old_dr.end() && old_r_it->second > 0.0) {
                    sink_ratios[sink] = new_r / old_r_it->second;
                }
            }
            
            if (sink_ratios.empty()) continue;
            
            double sum = 0.0;
            for (const auto& [_, r] : sink_ratios) sum += r;
            double avg = sum / sink_ratios.size();
            res_avg_ratio[net_name] = avg;
            res_segment_scales[net_name] = compute_res_segment_scales(net, sink_ratios, avg);
        }
    }
    
    std::cout << "[backmark] Nets with cap update: " << cap_ratio.size() << std::endl;
    std::cout << "[backmark] Nets with res update: " << res_avg_ratio.size() << std::endl;
    
    // Rewrite SPEF line by line
    std::ifstream fin(spef_path);
    std::ofstream fout(output_path);
    
    if (!fin.is_open()) {
        throw std::runtime_error("Cannot open input file: " + spef_path);
    }
    if (!fout.is_open()) {
        throw std::runtime_error("Cannot open output file: " + output_path);
    }
    
    enum Section { SEC_NONE, SEC_CONN, SEC_CAP, SEC_RES };
    Section section = SEC_NONE;
    
    std::string current_net_name;
    std::string current_net_id;
    double c_scale = 1.0;
    double r_avg_scale = 1.0;
    std::unordered_map<std::string, std::unordered_map<std::string, double>> r_edge_scales;
    
    std::string line;
    size_t lines_written = 0;
    
    std::regex re_cap_idx("^\\s*\\d+\\s+");
    
    while (std::getline(fin, line)) {
        std::string raw = trim(line);
        
        // *D_NET header
        if (raw.compare(0, 7, "*D_NET ") == 0) {
            std::istringstream iss(raw);
            std::string token, net_id_tok, cap_str;
            iss >> token >> net_id_tok >> cap_str;
            
            std::string net_name_resolved = net_id_tok;
            auto it_nm = spef.name_map.find(net_id_tok);
            if (it_nm != spef.name_map.end()) {
                net_name_resolved = it_nm->second;
            }
            
            current_net_name = net_name_resolved;
            current_net_id = net_id_tok;
            section = SEC_NONE;
            
            auto it_cap = cap_ratio.find(net_name_resolved);
            if (it_cap != cap_ratio.end()) c_scale = it_cap->second;
            else c_scale = 1.0;
            
            auto it_avg = res_avg_ratio.find(net_name_resolved);
            if (it_avg != res_avg_ratio.end()) r_avg_scale = it_avg->second;
            else r_avg_scale = 1.0;
            
            auto it_edge = res_segment_scales.find(net_name_resolved);
            if (it_edge != res_segment_scales.end()) r_edge_scales = it_edge->second;
            else r_edge_scales.clear();
            
            // Write with updated total cap if needed
            if (new_total_caps.find(net_name_resolved) != new_total_caps.end()) {
                fout << "*D_NET " << net_id_tok << " " << fmt_float(new_total_caps[net_name_resolved]) << "\n";
                lines_written++;
                continue;
            }
            
            fout << line << "\n";
            lines_written++;
            continue;
        }
        
        // Inside a net
        if (!current_net_name.empty()) {
            if (raw == "*CONN") {
                section = SEC_CONN;
                fout << line << "\n";
                lines_written++;
                continue;
            }
            if (raw == "*CAP") {
                section = SEC_CAP;
                fout << line << "\n";
                lines_written++;
                continue;
            }
            if (raw == "*RES") {
                section = SEC_RES;
                fout << line << "\n";
                lines_written++;
                continue;
            }
            if (raw == "*END") {
                current_net_name.clear();
                current_net_id.clear();
                section = SEC_NONE;
                c_scale = 1.0;
                r_avg_scale = 1.0;
                r_edge_scales.clear();
                fout << line << "\n";
                lines_written++;
                continue;
            }
            
            // CAP section: scale cap values
            if (section == SEC_CAP && c_scale != 1.0) {
                // Check if this is a cap data line (starts with number)
                if (std::regex_search(raw, re_cap_idx)) {
                    std::istringstream iss(raw);
                    std::vector<std::string> tokens;
                    std::string tok;
                    while (iss >> tok) tokens.push_back(tok);
                    
                    if (tokens.size() >= 3) {
                        try {
                            // Try to parse last token as float (cap value)
                            std::string cap_tok = tokens.back();
                            double old_val = std::stod(cap_tok);
                            double new_val = old_val * c_scale;
                            tokens.back() = fmt_float(new_val);
                            
                            // Reconstruct line
                            std::string lead;
                            size_t non_space = line.find_first_not_of(" \t");
                            if (non_space != std::string::npos) {
                                lead = line.substr(0, non_space);
                            }
                            
                            for (size_t i = 0; i < tokens.size(); i++) {
                                fout << (i == 0 ? lead : " ") << tokens[i];
                            }
                            fout << "\n";
                            lines_written++;
                            continue;
                        } catch (...) {
                            // Not a cap line, fall through
                        }
                    }
                }
            }
            
            // RES section: scale res values
            if (section == SEC_RES && (!r_edge_scales.empty() || r_avg_scale != 1.0)) {
                std::istringstream iss(raw);
                std::vector<std::string> tokens;
                std::string tok;
                while (iss >> tok) tokens.push_back(tok);
                
                if (tokens.size() >= 4) {
                    try {
                        int idx = std::stoi(tokens[0]);  // index
                        
                        std::string n1 = resolve_spef_token(tokens[1], spef.name_map);
                        std::string n2 = resolve_spef_token(tokens[2], spef.name_map);
                        double old_val = std::stod(tokens[3]);
                        
                        double seg_scale = r_avg_scale;
                        
                        auto it_edge1 = r_edge_scales.find(n1);
                        if (it_edge1 != r_edge_scales.end()) {
                            auto it_edge2 = it_edge1->second.find(n2);
                            if (it_edge2 != it_edge1->second.end()) {
                                seg_scale = it_edge2->second;
                            }
                        }
                        
                        if (seg_scale != 1.0) {
                            double new_val = old_val * seg_scale;
                            
                            std::string lead;
                            size_t non_space = line.find_first_not_of(" \t");
                            if (non_space != std::string::npos) {
                                lead = line.substr(0, non_space);
                            }
                            
                            for (size_t i = 0; i < 3; i++) {
                                fout << (i == 0 ? lead : " ") << tokens[i];
                            }
                            fout << " " << fmt_float(new_val) << " \n";
                            lines_written++;
                            continue;
                        }
                    } catch (...) {
                        // Not a RES line, fall through
                    }
                }
            }
        }
        
        // Default: write line unchanged
        fout << line << "\n";
        lines_written++;
    }
    
    fin.close();
    fout.close();
    
    std::cout << "[backmark] Written " << lines_written << " lines to " << output_path << std::endl;
}

// ============== CORRELATION IMPLEMENTATIONS ==============

ComparisonResult compare_spef_full(
    ParsedSpef& spef1,
    ParsedSpef& spef2,
    int num_threads
) {
    std::cout << "[compare] Starting comparison with " << num_threads << " threads..." << std::endl;
    ComparisonResult result;
    
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
    result.common_nets = common_nets;
    
    // Parallel processing
    std::mutex result_mutex;
    std::vector<CapComparisonData> cap_rows_local;
    std::vector<ResComparisonData> res_rows_local;
    
    cap_rows_local.reserve(common_nets.size());
    res_rows_local.reserve(common_nets.size() * 4);  // Estimate
    
    auto worker = [&](int thread_id) {
        std::vector<CapComparisonData> local_caps;
        std::vector<ResComparisonData> local_ress;
        
        for (size_t i = thread_id; i < common_nets.size(); i += num_threads) {
            const std::string& net_name = common_nets[i];
            // .at() is safe for concurrent reads; operator[] is not (can insert)
            auto& net1 = spef1.nets.at(net_name);
            auto& net2 = spef2.nets.at(net_name);
            
            // Capacitance comparison
            local_caps.push_back({net_name, net1.total_cap, net2.total_cap});
            
            // Resistance comparison
            auto res1 = compute_driver_sink_resistances(net1);
            auto res2 = compute_driver_sink_resistances(net2);
            
            // Find common sinks
            std::vector<std::string> sinks1_vec, sinks2_vec;
            for (const auto& [sink, _] : res1) sinks1_vec.push_back(sink);
            for (const auto& [sink, _] : res2) sinks2_vec.push_back(sink);
            std::sort(sinks1_vec.begin(), sinks1_vec.end());
            std::sort(sinks2_vec.begin(), sinks2_vec.end());
            
            std::vector<std::string> common_sinks;
            std::set_intersection(
                sinks1_vec.begin(), sinks1_vec.end(),
                sinks2_vec.begin(), sinks2_vec.end(),
                std::back_inserter(common_sinks)
            );
            
            for (const auto& sink : common_sinks) {
                local_ress.push_back({
                    net_name,
                    net1.driver,
                    sink,
                    res1[sink],
                    res2[sink]
                });
            }
        }
        
        std::lock_guard<std::mutex> lock(result_mutex);
        cap_rows_local.insert(cap_rows_local.end(), local_caps.begin(), local_caps.end());
        res_rows_local.insert(res_rows_local.end(), local_ress.begin(), local_ress.end());
    };
    
    std::vector<std::thread> threads;
    threads.reserve(num_threads);
    for (int i = 0; i < num_threads; ++i) {
        threads.emplace_back(worker, i);
    }
    for (auto& t : threads) {
        t.join();
    }
    
    result.cap_rows = std::move(cap_rows_local);
    result.res_rows = std::move(res_rows_local);
    result.cap_count = result.cap_rows.size();
    result.res_count = result.res_rows.size();
    
    // Compute correlations
    if (result.cap_count > 0) {
        std::vector<double> xs, ys;
        xs.reserve(result.cap_count);
        ys.reserve(result.cap_count);
        for (const auto& row : result.cap_rows) {
            xs.push_back(row.c1);
            ys.push_back(row.c2);
        }
        auto corr = compute_pearson_correlation(xs, ys);
        result.cap_correlation = corr.valid ? corr.pearson : 0.0;
    }
    
    if (result.res_count > 0) {
        std::vector<double> xs, ys;
        xs.reserve(result.res_count);
        ys.reserve(result.res_count);
        for (const auto& row : result.res_rows) {
            xs.push_back(row.r1);
            ys.push_back(row.r2);
        }
        auto corr = compute_pearson_correlation(xs, ys);
        result.res_correlation = corr.valid ? corr.pearson : 0.0;
    }
    
    // Find top 10 cap deviations
    std::vector<std::pair<double, size_t>> cap_dev_idx;
    for (size_t i = 0; i < result.cap_rows.size(); ++i) {
        double dev = std::abs(result.cap_rows[i].c1 - result.cap_rows[i].c2);
        cap_dev_idx.push_back({dev, i});
    }
    std::nth_element(cap_dev_idx.begin(), cap_dev_idx.begin() + std::min((size_t)10, cap_dev_idx.size()),
                     cap_dev_idx.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
    
    for (size_t i = 0; i < std::min((size_t)10, cap_dev_idx.size()); ++i) {
        result.top_10_cap.push_back(result.cap_rows[cap_dev_idx[i].second]);
    }
    
    // Find top 10 res deviations
    std::vector<std::pair<double, size_t>> res_dev_idx;
    for (size_t i = 0; i < result.res_rows.size(); ++i) {
        double dev = std::abs(result.res_rows[i].r1 - result.res_rows[i].r2);
        res_dev_idx.push_back({dev, i});
    }
    std::nth_element(res_dev_idx.begin(), res_dev_idx.begin() + std::min((size_t)10, res_dev_idx.size()),
                     res_dev_idx.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
    
    for (size_t i = 0; i < std::min((size_t)10, res_dev_idx.size()); ++i) {
        result.top_10_res.push_back(result.res_rows[res_dev_idx[i].second]);
    }
    
    return result;
}

std::string summarize_comparison(const ComparisonResult& result) {
    std::ostringstream oss;
    oss << "=== SPEF RC Correlation Summary ===\n";
    oss << "Common nets: " << result.common_nets.size() << "\n";
    oss << "Cap rows: " << result.cap_count << ", Res rows: " << result.res_count << "\n";
    oss << "Cap correlation (Pearson): " << result.cap_correlation << "\n";
    oss << "Res correlation (Pearson): " << result.res_correlation << "\n";
    oss << "\nTop 10 Cap Deviations:\n";
    for (size_t i = 0; i < result.top_10_cap.size(); ++i) {
        const auto& row = result.top_10_cap[i];
        oss << "  " << row.net_name << ": " << row.c1 << " vs " << row.c2 
            << " (delta=" << (row.c2 - row.c1) << ")\n";
    }
    oss << "\nTop 10 Res Deviations:\n";
    for (size_t i = 0; i < result.top_10_res.size(); ++i) {
        const auto& row = result.top_10_res[i];
        oss << "  " << row.net_name << " / " << row.sink << ": " << row.r1 << " vs " << row.r2
            << " (delta=" << (row.r2 - row.r1) << ")\n";
    }
    return oss.str();
}

// Parse multiple SPEF files in parallel using C++ threads
std::vector<ParsedSpef> parse_spef_parallel(
    const std::vector<std::string>& filepaths,
    int num_threads
) {
    std::cout << "[parse_spef_parallel] Starting parallel parsing of " << filepaths.size() << " files with " << num_threads << " threads..." << std::endl;
    size_t n = filepaths.size();
    if (n == 0) return {};
    
    // Determine number of threads
    if (num_threads <= 0) {
        num_threads = std::thread::hardware_concurrency();
        if (num_threads == 0) num_threads = 2;
    }
    if (num_threads > static_cast<int>(n)) {
        num_threads = static_cast<int>(n);
    }
    
    std::vector<ParsedSpef> results(n);
    std::vector<std::thread> threads;
    std::mutex print_mutex;
    std::vector<double> elapsed_times(n, 0.0);

    for (int t = 0; t < num_threads; ++t) {
        threads.emplace_back([&, t]() {
            for (size_t i = t; i < n; i += num_threads) {
                auto t_start = std::chrono::steady_clock::now();
                results[i] = parse_spef(filepaths[i]);
                auto t_end = std::chrono::steady_clock::now();
                double elapsed = std::chrono::duration<double>(t_end - t_start).count();
                elapsed_times[i] = elapsed;
                {
                    std::lock_guard<std::mutex> lock(print_mutex);
                    std::cout << "[" << filepaths[i] << "] finished parsing "
                              << results[i].nets.size() << " nets in " << elapsed << "s (C++/parallel, thread " << t << ")" << std::endl;
                }
            }
        });
    }

    for (auto& th : threads) {
        th.join();
    }

    // Print summary
    double total = 0.0;
    for (size_t i = 0; i < n; ++i) total += elapsed_times[i];
    std::cout << "[parse_spef_parallel] Parsed " << n << " files in total " << total << "s (sum of wall times)" << std::endl;
    return results;
}

// ============== NUMPY ARRAY EXPORT FOR FAST PLOTTING ==============

PlotData export_plot_data(
    ParsedSpef& spef1,
    ParsedSpef& spef2,
    int num_threads
) {
    PlotData result;
    
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
    
    size_t n_cap = common_nets.size();
    result.cap_count = n_cap;
    
    // Use std::vector for temporary storage, then convert to numpy
    std::vector<double> cap_c1_vec;
    std::vector<double> cap_c2_vec;
    cap_c1_vec.reserve(n_cap);
    cap_c2_vec.reserve(n_cap);
    result.cap_net_names.reserve(n_cap);
    
    std::vector<double> res_r1_vec;
    std::vector<double> res_r2_vec;
    res_r1_vec.reserve(n_cap * 4);
    res_r2_vec.reserve(n_cap * 4);
    std::vector<std::string> res_net_names;
    std::vector<std::string> res_sink_names;
    res_net_names.reserve(n_cap * 4);
    res_sink_names.reserve(n_cap * 4);
    
    // Parallel processing
    std::mutex result_mutex;
    
    auto worker = [&](int thread_id) {
        std::vector<double> local_cap_c1;
        std::vector<double> local_cap_c2;
        std::vector<std::string> local_cap_names;
        
        std::vector<double> local_res_r1;
        std::vector<double> local_res_r2;
        std::vector<std::string> local_res_net_names;
        std::vector<std::string> local_res_sink_names;
        
        local_cap_c1.reserve(n_cap / num_threads + 1);
        local_cap_c2.reserve(n_cap / num_threads + 1);
        local_cap_names.reserve(n_cap / num_threads + 1);
        
        for (size_t i = thread_id; i < common_nets.size(); i += num_threads) {
            const std::string& net_name = common_nets[i];
            // .at() is safe for concurrent reads; operator[] is not (can insert)
            auto& net1 = spef1.nets.at(net_name);
            auto& net2 = spef2.nets.at(net_name);
            
            // Capacitance
            local_cap_c1.push_back(net1.total_cap);
            local_cap_c2.push_back(net2.total_cap);
            local_cap_names.push_back(net_name);
            
            // Resistance
            auto res1 = compute_driver_sink_resistances(net1);
            auto res2 = compute_driver_sink_resistances(net2);
            
            // Find common sinks
            std::vector<std::string> sinks1_vec, sinks2_vec;
            for (const auto& [sink, _] : res1) sinks1_vec.push_back(sink);
            for (const auto& [sink, _] : res2) sinks2_vec.push_back(sink);
            std::sort(sinks1_vec.begin(), sinks1_vec.end());
            std::sort(sinks2_vec.begin(), sinks2_vec.end());
            
            std::vector<std::string> common_sinks;
            std::set_intersection(
                sinks1_vec.begin(), sinks1_vec.end(),
                sinks2_vec.begin(), sinks2_vec.end(),
                std::back_inserter(common_sinks)
            );
            
            for (const auto& sink : common_sinks) {
                local_res_r1.push_back(res1[sink]);
                local_res_r2.push_back(res2[sink]);
                local_res_net_names.push_back(net_name);
                local_res_sink_names.push_back(sink);
            }
        }
        
        // Copy to shared vectors
        {
            std::lock_guard<std::mutex> lock(result_mutex);
            cap_c1_vec.insert(cap_c1_vec.end(), local_cap_c1.begin(), local_cap_c1.end());
            cap_c2_vec.insert(cap_c2_vec.end(), local_cap_c2.begin(), local_cap_c2.end());
            result.cap_net_names.insert(result.cap_net_names.end(), local_cap_names.begin(), local_cap_names.end());
            
            res_r1_vec.insert(res_r1_vec.end(), local_res_r1.begin(), local_res_r1.end());
            res_r2_vec.insert(res_r2_vec.end(), local_res_r2.begin(), local_res_r2.end());
            res_net_names.insert(res_net_names.end(), local_res_net_names.begin(), local_res_net_names.end());
            res_sink_names.insert(res_sink_names.end(), local_res_sink_names.begin(), local_res_sink_names.end());
        }
    };
    
    std::vector<std::thread> threads;
    threads.reserve(num_threads);
    for (int i = 0; i < num_threads; ++i) {
        threads.emplace_back(worker, i);
    }
    for (auto& t : threads) {
        t.join();
    }
    
    // Convert to numpy arrays with correct size
    result.cap_c1 = py::array_t<double>(cap_c1_vec.size());
    result.cap_c2 = py::array_t<double>(cap_c2_vec.size());
    auto cap_c1_buf = result.cap_c1.request();
    auto cap_c2_buf = result.cap_c2.request();
    double* cap_c1_ptr = static_cast<double*>(cap_c1_buf.ptr);
    double* cap_c2_ptr = static_cast<double*>(cap_c2_buf.ptr);
    std::memcpy(cap_c1_ptr, cap_c1_vec.data(), cap_c1_vec.size() * sizeof(double));
    std::memcpy(cap_c2_ptr, cap_c2_vec.data(), cap_c2_vec.size() * sizeof(double));
    
    result.res_count = res_r1_vec.size();
    result.res_r1 = py::array_t<double>(res_r1_vec.size());
    result.res_r2 = py::array_t<double>(res_r2_vec.size());
    auto res_r1_buf = result.res_r1.request();
    auto res_r2_buf = result.res_r2.request();
    double* res_r1_ptr = static_cast<double*>(res_r1_buf.ptr);
    double* res_r2_ptr = static_cast<double*>(res_r2_buf.ptr);
    std::memcpy(res_r1_ptr, res_r1_vec.data(), res_r1_vec.size() * sizeof(double));
    std::memcpy(res_r2_ptr, res_r2_vec.data(), res_r2_vec.size() * sizeof(double));
    
    result.res_net_names = std::move(res_net_names);
    result.res_sink_names = std::move(res_sink_names);
    
    // Compute correlations
    auto corr_c = compute_pearson_correlation(cap_c1_vec, cap_c2_vec);
    result.cap_correlation = corr_c.valid ? corr_c.pearson : 0.0;
    
    if (result.res_count > 0) {
        auto corr_r = compute_pearson_correlation(res_r1_vec, res_r2_vec);
        result.res_correlation = corr_r.valid ? corr_r.pearson : 0.0;
    }
    
    return result;
}

// Chunked comparison for large datasets
ComparisonChunk compare_spef_chunk(
    ParsedSpef& spef1,
    ParsedSpef& spef2,
    size_t start_idx,
    size_t chunk_size,
    int num_threads
) {
    ComparisonChunk chunk;
    
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
    
    size_t total_nets = common_nets.size();
    size_t end_idx = std::min(start_idx + chunk_size, total_nets);
    chunk.is_last = (end_idx >= total_nets);
    
    // Process only the chunk
    std::vector<CapComparisonData> local_caps;
    std::vector<ResComparisonData> local_ress;
    local_caps.reserve(chunk_size);
    local_ress.reserve(chunk_size * 4);
    
    for (size_t i = start_idx; i < end_idx; ++i) {
        const std::string& net_name = common_nets[i];
        auto& net1 = spef1.nets[net_name];
        auto& net2 = spef2.nets[net_name];
        
        local_caps.push_back({net_name, net1.total_cap, net2.total_cap});
        
        auto res1 = compute_driver_sink_resistances(net1);
        auto res2 = compute_driver_sink_resistances(net2);
        
        std::vector<std::string> sinks1_vec, sinks2_vec;
        for (const auto& [sink, _] : res1) sinks1_vec.push_back(sink);
        for (const auto& [sink, _] : res2) sinks2_vec.push_back(sink);
        std::sort(sinks1_vec.begin(), sinks1_vec.end());
        std::sort(sinks2_vec.begin(), sinks2_vec.end());
        
        std::vector<std::string> common_sinks;
        std::set_intersection(
            sinks1_vec.begin(), sinks1_vec.end(),
            sinks2_vec.begin(), sinks2_vec.end(),
            std::back_inserter(common_sinks)
        );
        
        for (const auto& sink : common_sinks) {
            local_ress.push_back({net_name, net1.driver, sink, res1[sink], res2[sink]});
        }
    }
    
    // Convert to numpy arrays
    size_t n_cap = local_caps.size();
    size_t n_res = local_ress.size();
    
    chunk.cap_c1 = py::array_t<double>(n_cap);
    chunk.cap_c2 = py::array_t<double>(n_cap);
    auto c1_buf = chunk.cap_c1.request();
    auto c2_buf = chunk.cap_c2.request();
    double* c1_ptr = static_cast<double*>(c1_buf.ptr);
    double* c2_ptr = static_cast<double*>(c2_buf.ptr);
    
    chunk.res_r1 = py::array_t<double>(n_res);
    chunk.res_r2 = py::array_t<double>(n_res);
    auto r1_buf = chunk.res_r1.request();
    auto r2_buf = chunk.res_r2.request();
    double* r1_ptr = static_cast<double*>(r1_buf.ptr);
    double* r2_ptr = static_cast<double*>(r2_buf.ptr);
    
    chunk.cap_net_names.reserve(n_cap);
    chunk.res_net_names.reserve(n_res);
    chunk.res_sink_names.reserve(n_res);
    
    for (size_t i = 0; i < n_cap; ++i) {
        c1_ptr[i] = local_caps[i].c1;
        c2_ptr[i] = local_caps[i].c2;
        chunk.cap_net_names.push_back(local_caps[i].net_name);
    }
    
    for (size_t i = 0; i < n_res; ++i) {
        r1_ptr[i] = local_ress[i].r1;
        r2_ptr[i] = local_ress[i].r2;
        chunk.res_net_names.push_back(local_ress[i].net_name);
        chunk.res_sink_names.push_back(local_ress[i].sink);
    }
    
    return chunk;
}
