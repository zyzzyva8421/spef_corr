#include "spef_core.h"
#include <iomanip>
#include <sstream>

// ============== Coupling Capacitance Resolution ==============
void resolve_coupling_caps_to_nets(ParsedSpef& spef) {
    auto resolve_node_to_net = [&](const std::string& node) -> std::string {
        if (node.empty()) return "";
        if (spef.nets.find(node) != spef.nets.end()) {
            return node;
        }
        size_t colon = node.find(':');
        std::string base = (colon == std::string::npos) ? node : node.substr(0, colon);
        if (spef.nets.find(base) != spef.nets.end()) {
            return base;
        }
        if (!base.empty() && base[0] == '*') {
            auto it = spef.name_map.find(base);
            if (it != spef.name_map.end()) {
                const std::string& mapped = it->second;
                if (spef.nets.find(mapped) != spef.nets.end()) {
                    return mapped;
                }
            }
        }
        return "";
    };

    // Create normalized pair key (sort so A|B and B|A map to the same key)
    auto make_pair_key = [](const std::string& a, const std::string& b) -> std::string {
        return (a < b) ? (a + "|" + b) : (b + "|" + a);
    };

    // Collect nets that actually contain coupling entries.
    std::vector<std::pair<const std::string*, const NetData*>> work_items;
    work_items.reserve(spef.nets.size());
    for (const auto& [net_name, net_data] : spef.nets) {
        if (!net_data.raw_coupling_caps.empty())
            work_items.push_back({&net_name, &net_data});
    }

    // First pass: accumulate capacitance for each pair across all nets.
    // Each net's section is summed into a local map first (handles multiple
    // entries for the same pair within one net's CAP section).  The local
    // result is then merged into an accumulator with try_emplace so that the
    // symmetric entry in the other net's CAP section is ignored.
    std::unordered_map<std::string, double> cap_accumulator;
    cap_accumulator.reserve(work_items.size() / 4 + 64);

    // Parallel overhead (thread startup + per-thread map merge) starts to pay off
    // only for larger workloads; keep small/medium parses on the sequential path.
    constexpr size_t kMinWorkItemsForParallel = 4096;
    // Align with parse_spef()'s existing cap to avoid oversubscription on large hosts.
    constexpr int kMaxResolveThreads = 8;
    int hw = (int)std::thread::hardware_concurrency();
    // Conservative fallback when hardware_concurrency() is unavailable.
    if (hw <= 0) hw = 2;
    int n_threads = (work_items.size() >= kMinWorkItemsForParallel) ? std::min(hw, kMaxResolveThreads) : 1;
    n_threads = std::min<int>(n_threads, (int)work_items.size());

    if (n_threads <= 1) {
        for (const auto& item : work_items) {
            const std::string& net_name = *item.first;
            const NetData& net_data = *item.second;
            std::unordered_map<std::string, double> local_caps;
            local_caps.reserve(net_data.raw_coupling_caps.size());
            for (const auto& entry : net_data.raw_coupling_caps) {
                // Use pre-parsed struct fields directly – no string splitting or stod call needed.
                std::string net1 = resolve_node_to_net(entry.node1);
                std::string net2 = resolve_node_to_net(entry.node2);
                if (net1.empty()) net1 = net_name;
                if (net1.empty() || net2.empty()) continue;

                if (net1 != net2) {
                    std::string pair_key = make_pair_key(net1, net2);
                    double cap_converted = spef.c_scale * convert_capacitance(entry.cap_val, spef.c_unit);
                    local_caps[pair_key] += cap_converted;
                }
            }
            for (const auto& [key, val] : local_caps)
                cap_accumulator.try_emplace(key, val);
        }
    } else {
        std::vector<std::unordered_map<std::string, double>> thread_accumulators((size_t)n_threads);
        size_t work_items_per_thread = (work_items.size() + (size_t)n_threads - 1) / (size_t)n_threads;
        for (auto& m : thread_accumulators) {
            m.reserve(work_items_per_thread / 4 + 64);
        }
        std::vector<std::thread> threads;
        threads.reserve((size_t)n_threads);

        for (int t = 0; t < n_threads; ++t) {
            threads.emplace_back([&thread_accumulators, &work_items, &spef, &resolve_node_to_net, &make_pair_key, t, n_threads]() {
                auto& thread_map = thread_accumulators[(size_t)t];
                for (size_t i = (size_t)t; i < work_items.size(); i += (size_t)n_threads) {
                    const std::string& net_name = *work_items[i].first;
                    const NetData& net_data = *work_items[i].second;
                    std::unordered_map<std::string, double> local_caps;
                    local_caps.reserve(net_data.raw_coupling_caps.size());

                    for (const auto& entry : net_data.raw_coupling_caps) {
                        std::string net1 = resolve_node_to_net(entry.node1);
                        std::string net2 = resolve_node_to_net(entry.node2);
                        if (net1.empty()) net1 = net_name;
                        if (net1.empty() || net2.empty()) continue;

                        if (net1 != net2) {
                            std::string pair_key = make_pair_key(net1, net2);
                            double cap_converted = spef.c_scale * convert_capacitance(entry.cap_val, spef.c_unit);
                            local_caps[pair_key] += cap_converted;
                        }
                    }

                    for (const auto& [key, val] : local_caps)
                        thread_map.try_emplace(key, val);
                }
            });
        }

        for (auto& th : threads) th.join();

        for (auto& thread_map : thread_accumulators) {
            for (auto& [key, val] : thread_map)
                cap_accumulator.try_emplace(std::move(key), val);
        }
    }

    // Second pass: store the accumulated totals
    spef.coupling_caps.reserve(cap_accumulator.size());
    for (const auto& [pair_key, cap_total] : cap_accumulator) {
        size_t sep = pair_key.find('|');
        std::string net1 = pair_key.substr(0, sep);
        std::string net2 = pair_key.substr(sep + 1);
        spef.coupling_caps.push_back({net1, net2, cap_total});
    }
    
    // Clear temporary storage
    for (auto& [net_name, net_data] : spef.nets) {
        net_data.raw_coupling_caps.clear();
        net_data.raw_coupling_caps.shrink_to_fit();
    }
}

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

// ============== Equivalent Resistance (Nodal Analysis) ==============

double compute_equivalent_resistance(
    const std::unordered_map<std::string, std::vector<Edge>>& graph,
    const std::string& source,
    const std::string& sink
) {
    if (source == sink || graph.empty()) return 0.0;

    // Build ordered node list
    std::vector<std::string> nodes;
    nodes.reserve(graph.size());
    std::unordered_map<std::string, int> node_idx;
    node_idx.reserve(graph.size());

    for (const auto& [node, _] : graph) {
        node_idx[node] = (int)nodes.size();
        nodes.push_back(node);
    }

    int n = (int)nodes.size();
    if (n < 2) return 0.0;

    auto src_it = node_idx.find(source);
    auto snk_it = node_idx.find(sink);
    if (src_it == node_idx.end() || snk_it == node_idx.end()) return 0.0;

    int src_idx = src_it->second;
    int snk_idx = snk_it->second;

    // Build conductance Laplacian (dense matrix)
    std::vector<std::vector<double>> G(n, std::vector<double>(n, 0.0));
    for (const auto& [node, edges] : graph) {
        int i = node_idx.at(node);
        for (const auto& edge : edges) {
            auto it = node_idx.find(edge.to);
            if (it == node_idx.end() || edge.weight <= 1e-15) continue;
            int j = it->second;
            double g = 1.0 / edge.weight;  // conductance = 1/resistance
            G[i][i] += g;
            G[i][j] -= g;
        }
    }

    // RHS: inject 1A at source, extract 1A at sink
    std::vector<double> b(n, 0.0);
    b[src_idx] = 1.0;
    b[snk_idx] = -1.0;

    // Ground the sink node (V[sink] = 0) to make the system non-singular
    for (int j = 0; j < n; j++) G[snk_idx][j] = 0.0;
    G[snk_idx][snk_idx] = 1.0;
    b[snk_idx] = 0.0;

    // Gaussian elimination with partial pivoting
    for (int col = 0; col < n; col++) {
        int pivot = -1;
        double max_val = 0.0;
        for (int row = col; row < n; row++) {
            if (std::abs(G[row][col]) > max_val) {
                max_val = std::abs(G[row][col]);
                pivot = row;
            }
        }
        if (pivot < 0 || max_val < 1e-15) return 0.0;  // Singular (disconnected net)

        if (pivot != col) {
            std::swap(G[col], G[pivot]);
            std::swap(b[col], b[pivot]);
        }

        double diag = G[col][col];
        for (int row = col + 1; row < n; row++) {
            double factor = G[row][col] / diag;
            if (std::abs(factor) < 1e-30) continue;
            G[row][col] = 0.0;
            for (int c = col + 1; c < n; c++) {
                G[row][c] -= factor * G[col][c];
            }
            b[row] -= factor * b[col];
        }
    }

    // Back substitution
    std::vector<double> v(n, 0.0);
    for (int row = n - 1; row >= 0; row--) {
        double sum = b[row];
        for (int c = row + 1; c < n; c++) {
            sum -= G[row][c] * v[c];
        }
        if (std::abs(G[row][row]) < 1e-15) return 0.0;
        v[row] = sum / G[row][row];
    }

    // R_equiv = V[source] - V[sink] = V[source]  (V[sink] = 0 by grounding)
    double r_equiv = v[src_idx];
    return (r_equiv > 0.0) ? r_equiv : 0.0;
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

// ============== Equivalent Driver-Sink Resistances ==============
std::unordered_map<std::string, double> compute_driver_sink_equivalent_resistances(
    NetData& net
) {
    if (net.equiv_res_cache_valid && !net.driver_sink_equiv_res_cache.empty()) {
        return net.driver_sink_equiv_res_cache;
    }

    net.driver_sink_equiv_res_cache.clear();

    if (net.driver.empty() || net.sinks.empty() || net.res_graph.empty()) {
        net.equiv_res_cache_valid = true;
        return net.driver_sink_equiv_res_cache;
    }

    // Find best matching driver node
    std::string driver_node = net.driver;
    if (net.res_graph.find(net.driver) == net.res_graph.end()) {
        size_t colon_pos = net.driver.find(':');
        std::string base = (colon_pos != std::string::npos) ?
            net.driver.substr(0, colon_pos) : net.driver;
        for (const auto& [node, _] : net.res_graph) {
            size_t nc = node.find(':');
            std::string nb = (nc != std::string::npos) ? node.substr(0, nc) : node;
            if (nb == base) { driver_node = node; break; }
        }
    }

    if (net.res_graph.find(driver_node) == net.res_graph.end()) {
        net.equiv_res_cache_valid = true;
        return net.driver_sink_equiv_res_cache;
    }

    for (const auto& sink : net.sinks) {
        // Resolve sink to graph node
        std::string sink_node = sink;
        if (net.res_graph.find(sink) == net.res_graph.end()) {
            size_t colon_pos = sink.find(':');
            std::string base = (colon_pos != std::string::npos) ?
                sink.substr(0, colon_pos) : sink;
            for (const auto& [node, _] : net.res_graph) {
                size_t nc = node.find(':');
                std::string nb = (nc != std::string::npos) ? node.substr(0, nc) : node;
                if (nb == base) { sink_node = node; break; }
            }
        }

        if (net.res_graph.find(sink_node) == net.res_graph.end()) continue;
        if (sink_node == driver_node) continue;

        double r_eq = compute_equivalent_resistance(net.res_graph, driver_node, sink_node);
        if (r_eq > 0.0) {
            net.driver_sink_equiv_res_cache[sink] = r_eq;
        }
    }

    net.equiv_res_cache_valid = true;
    return net.driver_sink_equiv_res_cache;
}

// Dispatch driver-sink resistance computation by method (0=dijkstra, 1=equivalent)
std::unordered_map<std::string, double> compute_driver_sink_res_by_method(
    NetData& net,
    int res_method
) {
    if (res_method == 1) {
        return compute_driver_sink_equivalent_resistances(net);
    }
    return compute_driver_sink_resistances(net);
}


// ============== RECOMMENDATION 2: Vectorized Correlation Computation ==============
static double compute_pearson_correlation(
    const std::vector<double>& xs,
    const std::vector<double>& ys
) {
    size_t n = xs.size();
    if (n != ys.size() || n < 2) return 0.0;

    double sum_x = 0.0, sum_y = 0.0;
    double sum_xy = 0.0, sum_x2 = 0.0, sum_y2 = 0.0;

    const double* x_ptr = xs.data();
    const double* y_ptr = ys.data();

    for (size_t i = 0; i < n; ++i) {
        double x = x_ptr[i];
        double y = y_ptr[i];
        sum_x  += x;
        sum_y  += y;
        sum_xy += x * y;
        sum_x2 += x * x;
        sum_y2 += y * y;
    }

    double mean_x = sum_x / n;
    double mean_y = sum_y / n;
    double cov_xy = sum_xy - n * mean_x * mean_y;
    double var_x  = sum_x2 - n * mean_x * mean_x;
    double var_y  = sum_y2 - n * mean_y * mean_y;

    double std_x = std::sqrt(std::max(0.0, var_x / n));
    double std_y = std::sqrt(std::max(0.0, var_y / n));
    if (std_x < 1e-15 || std_y < 1e-15) return 0.0;

    return cov_xy / (std::sqrt(std::max(0.0, var_x)) * std::sqrt(std::max(0.0, var_y)));
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

// ============== Fast low-level helpers ==============

// Read entire file into heap buffer with two null sentinel bytes.
// Single I/O call eliminates per-line syscall overhead.
static std::vector<char> read_file_buffer(const std::string& filepath) {
    FILE* f = fopen(filepath.c_str(), "rb");
    if (!f) throw std::runtime_error("Cannot open file: " + filepath);
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    std::vector<char> file_buf((size_t)std::max(0L, sz) + 2, '\0');
    if (sz > 0) {
        size_t n = fread(file_buf.data(), 1, (size_t)sz, f);
        (void)n;
    }
    fclose(f);
    return file_buf;
}

// Split up to max_n whitespace-delimited tokens from [begin, end).
// Writes string_views into out[]; returns actual token count.
// Zero-allocation — views point directly into the caller's buffer.
static inline int fast_split_sv(const char* begin, const char* end,
                                  std::string_view out[], int max_tokens) {
    int n = 0;
    const char* p = begin;
    while (p < end && n < max_tokens) {
        while (p < end && (*p == ' ' || *p == '\t')) ++p;
        if (p >= end) break;
        const char* s = p;
        while (p < end && *p != ' ' && *p != '\t') ++p;
        out[n++] = std::string_view(s, (size_t)(p - s));
    }
    return n;
}

// Fast float parse from string_view: uses strtod (no locale, no exceptions).
static inline double parse_float_sv(std::string_view sv) {
    if (sv.empty()) return 0.0;
    // Stack buffer covers almost all SPEF numeric tokens.
    char buf[64];
    const char* src = sv.data();
    size_t len = sv.size();
    if (len < sizeof(buf)) {
        memcpy(buf, src, len);
        buf[len] = '\0';
        char* endp;
        double val = strtod(buf, &endp);
        if (endp != buf) return val;
        // Fallback: strip non-numeric characters for malformed inputs (e.g., units
        // accidentally attached to a value).  This two-pass strategy keeps the fast
        // path allocation-free while still handling edge-cases gracefully.
        char clean[64];
        int clean_idx = 0;
        for (size_t i = 0; i < len && clean_idx < 62; ++i) {
            char c = src[i];
            if (c == '-' || c == '+' || c == '.' || c == 'e' || c == 'E' ||
                (c >= '0' && c <= '9'))
                clean[clean_idx++] = c;
        }
        if (clean_idx == 0) return 0.0;
        clean[clean_idx] = '\0';
        return strtod(clean, nullptr);
    }
    // Long token: heap-allocate (rare path).
    std::string tmp(src, len);
    char* endp;
    double val = strtod(tmp.c_str(), &endp);
    return (endp != tmp.c_str()) ? val : 0.0;
}

// Legacy std::string wrapper — keeps existing call-sites unchanged.
static inline double parse_float(const std::string& s) {
    return parse_float_sv(std::string_view(s.data(), s.size()));
}

// Resolve a name-map token to its actual net/node name.
// Fast path: tokens without '*' or '\\' are returned as-is (no extra allocation).
static std::string resolve_token_sv(std::string_view tok,
                                     const std::unordered_map<std::string, std::string>& name_map) {
    if (tok.empty()) return {};
    bool has_star      = (tok[0] == '*');
    bool has_backslash = (tok.find('\\') != std::string_view::npos);
    if (!has_star && !has_backslash) return std::string(tok);

    std::string out(tok);
    if (has_star) {
        size_t colon = out.find(':');
        if (colon != std::string::npos) {
            std::string base   = out.substr(0, colon);
            std::string suffix = out.substr(colon);
            auto it = name_map.find(base);
            if (it != name_map.end()) out = it->second + suffix;
        } else {
            auto it = name_map.find(out);
            if (it != name_map.end()) out = it->second;
        }
    }
    size_t pos;
    while ((pos = out.find("\\[")) != std::string::npos) out.replace(pos, 2, "[");
    while ((pos = out.find("\\]")) != std::string::npos) out.replace(pos, 2, "]");
    return out;
}

// ============== Header parser (NAME_MAP, units) ==============

// Parse the SPEF header from buffer [buf, buf+buf_size).
// Fills name_map, units, and scale factors into spef.
// Returns the buffer offset where the first *D_NET line begins
// (or buf_size if no *D_NET is found).
static size_t parse_spef_header_from_buf(const char* buf, size_t buf_size,
                                          ParsedSpef& spef) {
    bool in_name_map = false;
    const char* p   = buf;
    const char* end = buf + buf_size;

    while (p < end) {
        const char* nl           = (const char*)memchr(p, '\n', (size_t)(end - p));
        const char* line_end_raw = nl ? nl : end;
        const char* next_line    = nl ? nl + 1 : end;

        // Strip inline comment (//)
        const char* line_end = line_end_raw;
        for (const char* c = p; c + 1 < line_end_raw; ++c) {
            if (c[0] == '/' && c[1] == '/') { line_end = c; break; }
        }
        while (line_end > p &&
               (*(line_end - 1) == ' ' || *(line_end - 1) == '\t' ||
                *(line_end - 1) == '\r'))
            --line_end;
        const char* lstart = p;
        while (lstart < line_end && (*lstart == ' ' || *lstart == '\t')) ++lstart;

        p = next_line;

        size_t line_len = (size_t)(line_end - lstart);
        if (line_len == 0) continue;

        // Early exit: first *D_NET marks end of header
        if (line_len >= 6 &&
            lstart[0] == '*' && lstart[1] == 'D' && lstart[2] == '_' &&
            lstart[3] == 'N' && lstart[4] == 'E' && lstart[5] == 'T' &&
            (line_len == 6 || lstart[6] == ' ' || lstart[6] == '\t')) {
            return (size_t)(lstart - buf);
        }

        std::string_view toks[4];
        int ntok = fast_split_sv(lstart, line_end, toks, 4);
        if (ntok == 0) continue;

        const std::string_view& first = toks[0];

        // NAME_MAP entry: *<digits>  name
        if (in_name_map) {
            if (ntok >= 2 && !first.empty() && first[0] == '*' &&
                first.size() >= 2 && first[1] >= '0' && first[1] <= '9') {
                std::string key(first);
                std::string value(toks[1]);
                if (value.size() >= 2 && value.front() == '"' && value.back() == '"')
                    value = value.substr(1, value.size() - 2);
                size_t pos;
                while ((pos = value.find("\\[")) != std::string::npos) value.replace(pos, 2, "[");
                while ((pos = value.find("\\]")) != std::string::npos) value.replace(pos, 2, "]");
                spef.name_map.emplace(std::move(key), std::move(value));
                continue;
            } else {
                in_name_map = false;
            }
        }

        if (!first.empty() && first[0] == '*') {
            if (first == "*NAME_MAP") {
                in_name_map = true;
            } else if (first == "*PORTS") {
                in_name_map = false;
            } else if (first == "*R_UNIT" && ntok >= 3) {
                spef.r_unit  = std::string(toks[2]);
                spef.r_scale = parse_float_sv(toks[1]);
                if (spef.r_scale == 0.0) spef.r_scale = 1.0;
            } else if (first == "*C_UNIT" && ntok >= 3) {
                spef.c_unit  = std::string(toks[2]);
                spef.c_scale = parse_float_sv(toks[1]);
                if (spef.c_scale == 0.0) spef.c_scale = 1.0;
            } else if (first == "*T_UNIT" && ntok >= 3) {
                spef.t_unit = std::string(toks[2]);
            } else if (first == "*L_UNIT" && ntok >= 3) {
                spef.l_unit = std::string(toks[2]);
            }
        }
    }
    return buf_size;
}

// ============== Single net-block parser ==============

// Parse one *D_NET ... *END block from buffer slice [block_begin, block_end).
// Uses the shared (read-only) name_map for token resolution.
// Output net data is written into out_spef.nets.
static void parse_net_block(const char* block_begin, const char* block_end,
                             const std::unordered_map<std::string, std::string>& name_map,
                             ParsedSpef& out_spef) {
    enum Section { SEC_NONE, SEC_CONN, SEC_CAP, SEC_RES };
    Section section = SEC_NONE;
    NetData* current_net = nullptr;

    const char* p = block_begin;
    while (p < block_end) {
        const char* nl           = (const char*)memchr(p, '\n', (size_t)(block_end - p));
        const char* line_end_raw = nl ? nl : block_end;
        const char* next_line    = nl ? nl + 1 : block_end;

        // Strip inline comment (//)
        const char* line_end = line_end_raw;
        for (const char* c = p; c + 1 < line_end_raw; ++c) {
            if (c[0] == '/' && c[1] == '/') { line_end = c; break; }
        }
        while (line_end > p &&
               (*(line_end - 1) == ' ' || *(line_end - 1) == '\t' ||
                *(line_end - 1) == '\r'))
            --line_end;
        const char* lstart = p;
        while (lstart < line_end && (*lstart == ' ' || *lstart == '\t')) ++lstart;

        p = next_line;

        size_t line_len = (size_t)(line_end - lstart);
        if (line_len == 0) continue;

        // 6 slots: enough for any SPEF data line
        std::string_view toks[6];
        int ntok = fast_split_sv(lstart, line_end, toks, 6);
        if (ntok == 0) continue;

        const std::string_view& first = toks[0];

        if (first[0] == '*') {
            if (first == "*D_NET") {
                if (ntok >= 3) {
                    std::string net_id(toks[1]);
                    std::string resolved = net_id;
                    if (!net_id.empty() && net_id[0] == '*') {
                        auto it = name_map.find(net_id);
                        if (it != name_map.end()) resolved = it->second;
                    }
                    current_net = &out_spef.nets[resolved];
                    current_net->name       = resolved;
                    current_net->total_cap  = parse_float_sv(toks[2]);
                    section = SEC_NONE;
                }
            } else if (first == "*END") {
                current_net = nullptr;
                section     = SEC_NONE;
            } else if (first == "*CONN") {
                section = SEC_CONN;
            } else if (first == "*CAP") {
                section = SEC_CAP;
            } else if (first == "*RES") {
                section = SEC_RES;
            } else if (current_net && ntok >= 3 &&
                       (first == "*I" || first == "*P")) {
                // CONN pin entry — may appear in SEC_CONN or SEC_NONE
                if (section == SEC_NONE) section = SEC_CONN;
                std::string pin = resolve_token_sv(toks[1], name_map);
                char dir_char = '\0';
                for (int i = 2; i < ntok; ++i) {
                    if (!toks[i].empty() &&
                        (toks[i][0] == 'O' || toks[i][0] == 'B' || toks[i][0] == 'I')) {
                        dir_char = toks[i][0];
                        break;
                    }
                }
                if (dir_char) {
                    bool is_port   = (first[1] == 'P');
                    bool is_driver = (!is_port && dir_char == 'O') ||
                                      (is_port  && dir_char == 'I') ||
                                      dir_char == 'B';
                    bool is_sink   = (!is_port && dir_char == 'I') ||
                                      (is_port  && dir_char == 'O');
                    if (is_driver && current_net->driver.empty())
                        current_net->driver = pin;
                    else if (is_sink)
                        current_net->sinks.push_back(pin);
                }
            }
        } else {
            // Numeric-index data lines (CAP / RES entries)
            if (current_net == nullptr) continue;

            if (section == SEC_RES && ntok >= 4) {
                std::string node1 = resolve_token_sv(toks[1], name_map);
                std::string node2 = resolve_token_sv(toks[2], name_map);
                double rval = parse_float_sv(toks[3]);
                current_net->res_graph[node1].push_back({node2, rval});
                current_net->res_graph[node2].push_back({node1, rval});
            } else if (section == SEC_CAP && ntok >= 4) {
                // Coupling cap: idx node1 node2 cap_value
                // (3-token self-cap is skipped — total_cap already set from *D_NET header)
                std::string node1 = resolve_token_sv(toks[1], name_map);
                std::string node2 = resolve_token_sv(toks[2], name_map);
                double cap_val = parse_float_sv(toks[3]);
                current_net->raw_coupling_caps.push_back(
                    {std::move(node1), std::move(node2), cap_val});
            }
        }
    }
}

// ============== Intra-file parallel net parsing ==============

// Find byte offsets of all *D_NET lines in buf[start_offset .. buf_size).
// Uses memchr for fast newline scanning.
static std::vector<size_t> find_dnet_offsets(const char* buf, size_t buf_size,
                                               size_t start_offset) {
    std::vector<size_t> offsets;
    offsets.reserve(std::min<size_t>(1024 * 1024,
                                     // Heuristic: ~200 bytes per net + 64-entry buffer for small files.
                                     (buf_size > start_offset ?
                                      buf_size - start_offset : 0) / 200 + 64));

    const char* p   = buf + start_offset;
    const char* end = buf + buf_size;

    // Check if the content at start_offset is itself a *D_NET line
    {
        const char* ls = p;
        while (ls < end && (*ls == ' ' || *ls == '\t')) ++ls;
        if ((size_t)(end - ls) >= 6 &&
            ls[0] == '*' && ls[1] == 'D' && ls[2] == '_' &&
            ls[3] == 'N' && ls[4] == 'E' && ls[5] == 'T' &&
            ((size_t)(end - ls) == 6 || ls[6] == ' ' || ls[6] == '\t' ||
             ls[6] == '\r' || ls[6] == '\n'))
            offsets.push_back((size_t)(ls - buf));
    }

    while (p < end) {
        const char* nl = (const char*)memchr(p, '\n', (size_t)(end - p));
        if (!nl) break;
        const char* next = nl + 1;
        // Skip leading whitespace on next line
        while (next < end && (*next == ' ' || *next == '\t')) ++next;
        if ((size_t)(end - next) >= 6 &&
            next[0] == '*' && next[1] == 'D' && next[2] == '_' &&
            next[3] == 'N' && next[4] == 'E' && next[5] == 'T' &&
            ((size_t)(end - next) == 6 || next[6] == ' ' || next[6] == '\t' ||
             next[6] == '\r' || next[6] == '\n'))
            offsets.push_back((size_t)(next - buf));
        p = (next > nl + 1) ? next : nl + 1;
    }
    return offsets;
}

// Parse net blocks in parallel.
// Thread i processes dnet_offsets[i], [i+N], [i+2N], ...
// name_map is read-only; each thread writes into its own local ParsedSpef.
static void parse_nets_parallel(const char* buf, size_t buf_size,
                                  const std::vector<size_t>& dnet_offsets,
                                  const std::unordered_map<std::string, std::string>& name_map,
                                  ParsedSpef& out_spef, int n_threads) {
    size_t n_nets    = dnet_offsets.size();
    // Ceiling division: each thread gets at most ceil(n_nets / n_threads) blocks.
    size_t per_thread = (n_nets + (size_t)n_threads - 1) / (size_t)n_threads;

    std::vector<ParsedSpef> local_spefs((size_t)n_threads);
    for (auto& ls : local_spefs) {
        ls.c_scale = out_spef.c_scale;
        ls.r_scale = out_spef.r_scale;
        ls.c_unit  = out_spef.c_unit;
        ls.r_unit  = out_spef.r_unit;
        ls.nets.reserve(per_thread);
    }

    std::vector<std::thread> threads;
    threads.reserve((size_t)n_threads);
    for (int t = 0; t < n_threads; ++t) {
        threads.emplace_back([&, t]() {
            ParsedSpef& local = local_spefs[(size_t)t];
            for (size_t i = (size_t)t; i < n_nets; i += (size_t)n_threads) {
                size_t block_start = dnet_offsets[i];
                size_t block_end   = (i + 1 < n_nets) ? dnet_offsets[i + 1] : buf_size;
                parse_net_block(buf + block_start, buf + block_end, name_map, local);
            }
        });
    }
    for (auto& th : threads) th.join();

    // Merge local nets into out_spef (single-threaded after all threads join)
    out_spef.nets.reserve(n_nets + n_nets / 4);
    for (auto& ls : local_spefs) {
        for (auto& [name, net] : ls.nets)
            out_spef.nets.emplace(name, std::move(net));
    }
}

// ============== Public parse_spef entry point ==============

ParsedSpef parse_spef(const std::string& filepath) {
    auto t_start = std::chrono::steady_clock::now();

    // 1. Read entire file into memory — one syscall, eliminates per-line I/O overhead.
    std::vector<char> file_buf = read_file_buffer(filepath);
    // Two null sentinel bytes are appended; usable content is file_buf.size()-2 bytes.
    size_t buf_size = file_buf.size() >= 2 ? file_buf.size() - 2 : 0;
    const char* data = file_buf.data();
    auto t_io = std::chrono::steady_clock::now();

    ParsedSpef spef;

    // 2. Parse header section (NAME_MAP + units) — always single-threaded.
    spef.name_map.reserve(128 * 1024);
    size_t first_dnet_offset = parse_spef_header_from_buf(data, buf_size, spef);
    auto t_header = std::chrono::steady_clock::now();

    // 3. Locate all *D_NET block positions with fast memchr-based scan.
    std::vector<size_t> dnet_offsets = find_dnet_offsets(data, buf_size, first_dnet_offset);
    size_t net_count = dnet_offsets.size();
    auto t_scan = std::chrono::steady_clock::now();

    if (net_count == 0) {
        auto t_end = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(t_end - t_start).count();
        std::cout << "[" << filepath << "] finished parsing 0 nets in "
                  << elapsed << "s (C++/buffered)" << std::endl;
        return spef;
    }

    spef.nets.reserve(net_count + net_count / 4);

    // 4. Choose parallelism — thread spawn overhead pays off at ~10 K nets.
    int hw = (int)std::thread::hardware_concurrency();
    if (hw <= 0) hw = 2;
    int n_threads = (net_count >= 10000) ? std::min(hw, 8) : 1;

    if (n_threads > 1) {
        parse_nets_parallel(data, buf_size, dnet_offsets, spef.name_map, spef, n_threads);
    } else {
        for (size_t i = 0; i < net_count; ++i) {
            size_t block_start = dnet_offsets[i];
            size_t block_end   = (i + 1 < net_count) ? dnet_offsets[i + 1] : buf_size;
            parse_net_block(data + block_start, data + block_end, spef.name_map, spef);
        }
    }
    auto t_netparse = std::chrono::steady_clock::now();

    // 5. Post-process coupling caps (single-threaded; net ordering is irrelevant).
    resolve_coupling_caps_to_nets(spef);
    auto t_end = std::chrono::steady_clock::now();

    double elapsed      = std::chrono::duration<double>(t_end    - t_start  ).count();
    double t_io_s       = std::chrono::duration<double>(t_io     - t_start  ).count();
    double t_header_s   = std::chrono::duration<double>(t_header - t_io     ).count();
    double t_scan_s     = std::chrono::duration<double>(t_scan   - t_header ).count();
    double t_netparse_s = std::chrono::duration<double>(t_netparse - t_scan ).count();
    double t_ccap_s     = std::chrono::duration<double>(t_end    - t_netparse).count();

    std::cout << "[parse_spef:" << filepath << "] total=" << elapsed << "s"
              << "  file_io=" << t_io_s << "s"
              << "  header/namemap=" << t_header_s << "s"
              << "  dnet_scan=" << t_scan_s << "s"
              << "  net_blocks(" << n_threads << "t)=" << t_netparse_s << "s"
              << "  coupling_cap_resolve=" << t_ccap_s << "s"
              << "  nets=" << spef.nets.size()
              << "  ccaps=" << spef.coupling_caps.size()
              << std::endl;

    return spef;
}

// ============== Shuffle Implementation ==============
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

// ============== CSV Data File Parsing ==============

std::vector<std::tuple<std::string, double, double>> parse_cap_data(const std::string& path) {
    std::vector<std::tuple<std::string, double, double>> caps;
    
    std::ifstream file(path);
    if (!file.is_open()) {
        return caps;
    }
    
    std::string line;
    // Skip header line if CSV (check if first line has comma)
    if (std::getline(file, line)) {
        if (line.find(',') == std::string::npos) {
            // Not CSV - treat as backmark format (net c1 c2)
            std::istringstream iss(line);
            std::string net, c1_str, c2_str;
            if (iss >> net >> c1_str >> c2_str) {
                try {
                    caps.emplace_back(net, std::stod(c1_str), std::stod(c2_str));
                } catch (...) {}
            }
        }
    }
    
    // Continue reading
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        
        // Check format
        if (line.find(',') != std::string::npos) {
            // CSV format: net,c1,c2
            std::istringstream iss(line);
            std::string net, c1_str, c2_str;
            if (!std::getline(iss, net, ',') || 
                !std::getline(iss, c1_str, ',') || 
                !std::getline(iss, c2_str, ',')) {
                continue;
            }
            try {
                caps.emplace_back(net, std::stod(c1_str), std::stod(c2_str));
            } catch (...) {
                continue;
            }
        } else {
            // Backmark format: net c1 c2
            std::istringstream iss(line);
            std::string net, c1_str, c2_str;
            if (!(iss >> net >> c1_str >> c2_str)) continue;
            try {
                caps.emplace_back(net, std::stod(c1_str), std::stod(c2_str));
            } catch (...) {
                continue;
            }
        }
    }
    
    return caps;
}

std::vector<std::tuple<std::string, std::string, std::string, double, double>> parse_res_data(const std::string& path) {
    std::vector<std::tuple<std::string, std::string, std::string, double, double>> ress;
    
    std::ifstream file(path);
    if (!file.is_open()) {
        return ress;
    }
    
    std::string line;
    // Skip header line if CSV
    if (std::getline(file, line)) {
        if (line.find(',') == std::string::npos) {
            // Not CSV - treat as backmark format (net driver sink r1 r2)
            std::istringstream iss(line);
            std::string net, driver, sink, r1_str, r2_str;
            if (iss >> net >> driver >> sink >> r1_str >> r2_str) {
                try {
                    ress.emplace_back(net, driver, sink, std::stod(r1_str), std::stod(r2_str));
                } catch (...) {}
            }
        }
    }
    
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        
        // Check format
        if (line.find(',') != std::string::npos) {
            // CSV format: net,driver,sink,r1,r2
            std::istringstream iss(line);
            std::string net, driver, sink, r1_str, r2_str;
            if (!std::getline(iss, net, ',') || 
                !std::getline(iss, driver, ',') ||
                !std::getline(iss, sink, ',') || 
                !std::getline(iss, r1_str, ',') || 
                !std::getline(iss, r2_str, ',')) {
                continue;
            }
            try {
                ress.emplace_back(net, driver, sink, std::stod(r1_str), std::stod(r2_str));
            } catch (...) {
                continue;
            }
        } else {
            // Backmark format: net driver sink r1 r2
            std::istringstream iss(line);
            std::string net, driver, sink, r1_str, r2_str;
            if (!(iss >> net >> driver >> sink >> r1_str >> r2_str)) continue;
            try {
                ress.emplace_back(net, driver, sink, std::stod(r1_str), std::stod(r2_str));
            } catch (...) {
                continue;
            }
        }
    }
    
    return ress;
}

std::vector<std::tuple<std::string, std::string, double, double>> parse_ccap_data(const std::string& path) {
    std::vector<std::tuple<std::string, std::string, double, double>> ccaps;

    std::ifstream file(path);
    if (!file.is_open()) {
        return ccaps;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;

        // Support space format: net1 net2 c1 c2
        if (line.find(',') == std::string::npos) {
            std::istringstream iss(line);
            std::string net1, net2, c1_str, c2_str;
            if (!(iss >> net1 >> net2 >> c1_str >> c2_str)) continue;
            try {
                ccaps.emplace_back(net1, net2, std::stod(c1_str), std::stod(c2_str));
            } catch (...) {
                continue;
            }
            continue;
        }

        // Also accept CSV format: net1,net2,c1,c2
        std::istringstream iss(line);
        std::string net1, net2, c1_str, c2_str;
        if (!std::getline(iss, net1, ',') ||
            !std::getline(iss, net2, ',') ||
            !std::getline(iss, c1_str, ',') ||
            !std::getline(iss, c2_str, ',')) {
            continue;
        }
        try {
            ccaps.emplace_back(net1, net2, std::stod(c1_str), std::stod(c2_str));
        } catch (...) {
            continue;
        }
    }

    return ccaps;
}

PlotData create_plot_data_from_files(const std::string& cap_path, const std::string& res_path, const std::string& ccap_path) {
    PlotData result;
    
    // Initialize counts to 0
    result.cap_count = 0;
    result.res_count = 0;
    result.ccap_count = 0;
    result.cap_correlation = 0.0;
    result.res_correlation = 0.0;
    result.ccap_correlation = 0.0;
    
    // Parse cap data if provided
    if (!cap_path.empty()) {
        auto caps = parse_cap_data(cap_path);
        result.cap_count = caps.size();
        
        // Allocate arrays
        std::vector<double> c1_vec, c2_vec;
        c1_vec.reserve(caps.size());
        c2_vec.reserve(caps.size());
        
        for (const auto& cap : caps) {
            std::string net;
            double c1, c2;
            std::tie(net, c1, c2) = cap;
            result.cap_net_names.push_back(net);
            c1_vec.push_back(c1);
            c2_vec.push_back(c2);
        }
        
        // Create numpy arrays
        result.cap_c1 = py::array_t<double>(c1_vec.size(), c1_vec.data());
        result.cap_c2 = py::array_t<double>(c2_vec.size(), c2_vec.data());
        
        // Compute correlation
        if (!c1_vec.empty()) {
            result.cap_correlation = compute_pearson_correlation(c1_vec, c2_vec);
        }
    }
    
    // Parse res data if provided
    if (!res_path.empty()) {
        auto ress = parse_res_data(res_path);
        result.res_count = ress.size();
        
        // Allocate arrays
        std::vector<double> r1_vec, r2_vec;
        r1_vec.reserve(ress.size());
        r2_vec.reserve(ress.size());
        
        for (const auto& res : ress) {
            std::string net, driver, sink;
            double r1, r2;
            std::tie(net, driver, sink, r1, r2) = res;
            result.res_net_names.push_back(net);
            result.res_driver_names.push_back(driver);
            result.res_sink_names.push_back(sink);
            r1_vec.push_back(r1);
            r2_vec.push_back(r2);
        }
        
        // Create numpy arrays
        result.res_r1 = py::array_t<double>(r1_vec.size(), r1_vec.data());
        result.res_r2 = py::array_t<double>(r2_vec.size(), r2_vec.data());
        
        // Compute correlation
        if (!r1_vec.empty()) {
            result.res_correlation = compute_pearson_correlation(r1_vec, r2_vec);
        }
    }

    // Parse coupling cap data if provided
    if (!ccap_path.empty()) {
        auto ccaps = parse_ccap_data(ccap_path);
        result.ccap_count = ccaps.size();

        std::vector<double> c1_vec, c2_vec;
        c1_vec.reserve(ccaps.size());
        c2_vec.reserve(ccaps.size());

        for (const auto& ccap : ccaps) {
            std::string net1, net2;
            double c1, c2;
            std::tie(net1, net2, c1, c2) = ccap;
            result.ccap_net1_names.push_back(net1);
            result.ccap_net2_names.push_back(net2);
            c1_vec.push_back(c1);
            c2_vec.push_back(c2);
        }

        result.ccap_c1 = py::array_t<double>(c1_vec.size(), c1_vec.data());
        result.ccap_c2 = py::array_t<double>(c2_vec.size(), c2_vec.data());

        if (!c1_vec.empty()) {
            result.ccap_correlation = compute_pearson_correlation(c1_vec, c2_vec);
        }
    }
    
    return result;
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
    
    // Helper: look up the weight of an edge in res_graph (stored undirectionally).
    auto get_edge_weight = [&](const std::string& from_node, const std::string& to_node) -> double {
        auto it = net.res_graph.find(from_node);
        if (it != net.res_graph.end()) {
            for (const auto& edge : it->second) {
                if (edge.to == to_node) return edge.weight;
            }
        }
        // Try the reverse direction (SPEF resistance entries are bidirectional)
        auto it2 = net.res_graph.find(to_node);
        if (it2 != net.res_graph.end()) {
            for (const auto& edge : it2->second) {
                if (edge.to == from_node) return edge.weight;
            }
        }
        return 0.0;
    };

    // Assign scale factors to tree edges using the following strategy:
    //   - Shared edges (subtree covers more than one sink): use avg_ratio uniformly.
    //   - Exclusive edges (subtree covers exactly one sink): use a per-sink scale
    //     computed so that the rescaled driver-to-sink resistance strictly equals the
    //     target value.
    //
    // Math for sink S with ratio r = new_R / old_R:
    //   target_res        = path_total * r
    //   shared_scaled     = shared_old_sum * avg_ratio
    //   exclusive_scale   = (target_res - shared_scaled) / exclusive_old_sum
    //
    // This guarantees:
    //   shared_scaled + exclusive_old_sum * exclusive_scale == target_res  (exact)

    // Pass 1: assign avg_ratio to all shared edges.
    for (const auto& [child_node, par_node] : parent) {
        if (par_node.empty()) continue;
        auto sinks_it = sinks_below.find(child_node);
        bool is_shared = (sinks_it == sinks_below.end() || sinks_it->second.size() != 1);
        if (is_shared) {
            result[par_node][child_node] = avg_ratio;
            result[child_node][par_node] = avg_ratio;
        }
    }

    // Pass 2: for each sink, compute and assign a per-sink exclusive scale.
    for (const auto& [sink_node, ratio] : sink_node_to_ratio) {
        // Trace the path from sink back to driver using the BFS parent map.
        std::vector<std::pair<std::string, std::string>> path_edges; // (parent, child)
        std::string cur = sink_node;
        while (true) {
            auto pit = parent.find(cur);
            if (pit == parent.end() || pit->second.empty()) break;
            path_edges.push_back({pit->second, cur});
            cur = pit->second;
        }

        double shared_old_sum = 0.0;
        double exclusive_old_sum = 0.0;
        for (const auto& [par, child] : path_edges) {
            double w = get_edge_weight(par, child);
            auto sinks_it = sinks_below.find(child);
            bool is_shared = (sinks_it == sinks_below.end() || sinks_it->second.size() != 1);
            if (is_shared) {
                shared_old_sum += w;
            } else {
                exclusive_old_sum += w;
            }
        }

        double path_total    = shared_old_sum + exclusive_old_sum;
        double target_res    = path_total * ratio;
        double shared_scaled = shared_old_sum * avg_ratio;
        double exclusive_target = target_res - shared_scaled;

        double exclusive_scale;
        if (exclusive_old_sum > 1e-15 && exclusive_target > 0.0) {
            exclusive_scale = exclusive_target / exclusive_old_sum;
        } else {
            // Fallback: no exclusive segments, or degenerate target – keep avg_ratio.
            exclusive_scale = avg_ratio;
        }

        // Assign exclusive_scale to all exclusive edges on this sink's path.
        for (const auto& [par, child] : path_edges) {
            auto sinks_it = sinks_below.find(child);
            bool is_exclusive = (sinks_it != sinks_below.end() && sinks_it->second.size() == 1);
            if (is_exclusive) {
                result[par][child] = exclusive_scale;
                result[child][par] = exclusive_scale;
            }
        }
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

static double convert_capacitance_from_pf(double value_pf, const std::string& to_unit) {
    if (to_unit == "PF" || to_unit == "pf" || to_unit == "pF") {
        return value_pf;
    } else if (to_unit == "NF" || to_unit == "nf" || to_unit == "nF") {
        return value_pf / 1000.0;  // 1 PF = 0.001 NF
    } else if (to_unit == "UF" || to_unit == "uf" || to_unit == "uF" || to_unit == "µF") {
        return value_pf / 1000000.0;  // 1 PF = 1e-6 µF
    } else if (to_unit == "FF" || to_unit == "ff" || to_unit == "fF") {
        return value_pf * 1000.0;  // 1 PF = 1000 FF
    } else if (to_unit == "F" || to_unit == "f") {
        return value_pf * 1.0e-12;  // 1 PF = 1e-12 F
    }
    // Default: assume already in target unit
    return value_pf;
}

static double convert_resistance_from_ohm(double value_ohm, const std::string& to_unit) {
    if (to_unit == "OHM" || to_unit == "ohm") {
        return value_ohm;
    } else if (to_unit == "KOHM" || to_unit == "kohm") {
        return value_ohm / 1000.0;  // 1 OHM = 0.001 KOHM
    } else if (to_unit == "MOHM" || to_unit == "mohm") {
        return value_ohm / 1000000.0;  // 1 Ω = 1e-6 MΩ
    }
    // Default: assume already in target unit
    return value_ohm;
}

void backmark_spef(
    const std::string& spef_path,
    const std::string& cap_data_path,
    const std::string& res_data_path,
    const std::string& ccap_data_path,
    const std::string& output_path,
    int res_method
) {
    std::cout << "[backmark] Parsing SPEF..." << std::endl;
    // Parse SPEF
    ParsedSpef spef = parse_spef(spef_path);
    
    // Build reverse name_map: net_name -> net_id
    std::unordered_map<std::string, std::string> reverse_name_map;
    for (const auto& [net_id, net_name] : spef.name_map) {
        reverse_name_map[net_name] = net_id;
    }

    auto resolve_net_token = [&](const std::string& net_tok) -> std::string {
        if (net_tok.empty()) return "";
        if (spef.nets.find(net_tok) != spef.nets.end()) return net_tok;
        if (net_tok[0] == '*') {
            auto it = spef.name_map.find(net_tok);
            if (it != spef.name_map.end() && spef.nets.find(it->second) != spef.nets.end()) {
                return it->second;
            }
        }
        return net_tok;
    };

    auto resolve_node_to_net = [&](const std::string& node_tok) -> std::string {
        if (node_tok.empty()) return "";
        if (spef.nets.find(node_tok) != spef.nets.end()) return node_tok;

        size_t colon = node_tok.find(':');
        std::string base = (colon == std::string::npos) ? node_tok : node_tok.substr(0, colon);
        if (spef.nets.find(base) != spef.nets.end()) return base;

        if (!base.empty() && base[0] == '*') {
            auto it = spef.name_map.find(base);
            if (it != spef.name_map.end() && spef.nets.find(it->second) != spef.nets.end()) {
                return it->second;
            }
        }
        return "";
    };

    auto make_pair_key = [](const std::string& n1, const std::string& n2) -> std::string {
        if (n1 <= n2) return n1 + "\n" + n2;
        return n2 + "\n" + n1;
    };
    
    // Load cap data
    std::unordered_map<std::string, double> cap_ratio;
    std::unordered_map<std::string, double> new_total_caps;
    
    if (!cap_data_path.empty()) {
        auto raw_cap = parse_backmark_cap_data(cap_data_path);
        for (const auto& [key, new_cap_pf] : raw_cap) {
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
            double new_cap = convert_capacitance_from_pf(new_cap_pf, spef.c_unit) / spef.c_scale;
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
            auto old_dr = compute_driver_sink_res_by_method(net, res_method);
            if (old_dr.empty()) continue;
            
            std::unordered_map<std::string, double> sink_ratios;
            for (const auto& [sink, new_r_ohm] : sink_map) {
                auto old_r_it = old_dr.find(sink);
                if (old_r_it != old_dr.end() && old_r_it->second > 0.0) {
                    double new_r = convert_resistance_from_ohm(new_r_ohm, spef.r_unit);
                    sink_ratios[sink] = new_r / old_r_it->second;
                }
            }
            
            if (sink_ratios.empty()) continue;
            
            double sum = 0.0;
            for (const auto& [_, r] : sink_ratios) sum += r;
            double avg = sum / sink_ratios.size();
            res_avg_ratio[net_name] = avg;
            // For the equivalent (Thevenin) method every resistance segment is a
            // linear contributor to all driver-sink equivalent resistances.  Scaling
            // all segments by a single uniform factor k scales every equivalent
            // resistance by the same k, so using avg_ratio globally is both correct
            // and exact.  The shared/exclusive-segment decomposition performed by
            // compute_res_segment_scales is only meaningful for the Dijkstra (path
            // sum) method and must be skipped here to avoid the fallback error that
            // arises when exclusive_target turns negative.
            if (res_method != 1 /* not equivalent/Thevenin */) {
                res_segment_scales[net_name] = compute_res_segment_scales(net, sink_ratios, avg);
            }
        }
    }

    // Load coupling-cap targets from file: use LAST column as target value
    // Format example: net1 net2 ccap1 ccap2 -> use ccap2
    const bool use_ccap_backmark = !ccap_data_path.empty();
    std::unordered_map<std::string, double> ccap_target_by_pair;
    std::unordered_map<std::string, double> ccap_old_total_by_pair;
    std::unordered_map<std::string, size_t> ccap_old_count_by_pair;
    std::unordered_map<std::string, std::unordered_map<std::string, double>> ccap_old_by_net_pair;
    std::unordered_map<std::string, double> old_noncoupling_by_net;
    std::unordered_map<std::string, double> noncoupling_scale_by_net;

    if (use_ccap_backmark) {
        auto raw_ccap = parse_ccap_data(ccap_data_path);
        for (const auto& row : raw_ccap) {
            std::string n1, n2;
            double unused_old = 0.0;
            double target_ccap = 0.0;
            std::tie(n1, n2, unused_old, target_ccap) = row;

            std::string net1 = resolve_net_token(n1);
            std::string net2 = resolve_net_token(n2);
            if (net1.empty() || net2.empty() || net1 == net2) continue;

            ccap_target_by_pair[make_pair_key(net1, net2)] =
                convert_capacitance_from_pf(target_ccap, spef.c_unit) / spef.c_scale;
        }

        // First pass over original SPEF:
        // 1) collect original coupling totals per pair and per net section,
        // 2) collect original non-coupling CAP totals per net section.
        std::ifstream fscan(spef_path);
        if (!fscan.is_open()) {
            throw std::runtime_error("Cannot open input file for coupling scan: " + spef_path);
        }

        enum Section { SEC_NONE, SEC_CONN, SEC_CAP, SEC_RES };
        Section section_scan = SEC_NONE;
        std::string current_net_scan;
        std::string line_scan;

        while (std::getline(fscan, line_scan)) {
            std::string raw_scan = trim(line_scan);

            if (raw_scan.compare(0, 7, "*D_NET ") == 0) {
                std::istringstream iss(raw_scan);
                std::string token, net_id_tok, cap_str;
                iss >> token >> net_id_tok >> cap_str;
                current_net_scan = resolve_net_token(net_id_tok);
                section_scan = SEC_NONE;
                continue;
            }
            if (raw_scan == "*END") {
                current_net_scan.clear();
                section_scan = SEC_NONE;
                continue;
            }
            if (current_net_scan.empty()) continue;
            if (raw_scan == "*CONN") {
                section_scan = SEC_CONN;
                continue;
            }
            if (raw_scan == "*CAP") {
                section_scan = SEC_CAP;
                continue;
            }
            if (raw_scan == "*RES") {
                section_scan = SEC_RES;
                continue;
            }

            if (section_scan != SEC_CAP) continue;

            std::istringstream iss(raw_scan);
            std::vector<std::string> tokens;
            std::string tok;
            while (iss >> tok) tokens.push_back(tok);

            try {
                double old_v = std::stod(tokens.back());

                // Coupling cap line: idx node1 node2 value
                if (tokens.size() >= 4) {
                    std::string net1 = resolve_node_to_net(tokens[1]);
                    std::string net2 = resolve_node_to_net(tokens[2]);
                    if (!net1.empty() && !net2.empty() && net1 != net2) {
                        std::string pair_key = make_pair_key(net1, net2);
                        ccap_old_total_by_pair[pair_key] += old_v;
                        ccap_old_count_by_pair[pair_key] += 1;
                        ccap_old_by_net_pair[current_net_scan][pair_key] += old_v;
                    }
                } else {
                    // Non-coupling CAP (e.g. self-cap): idx node value
                    old_noncoupling_by_net[current_net_scan] += old_v;
                }
            } catch (...) {
                continue;
            }
        }

        // Compute per-net non-coupling scale when total cap update exists.
        // target_noncoupling = target_total_cap - target_coupling_sum
        for (const auto& [net_name, target_total_cap] : new_total_caps) {
            double target_coupling_sum = 0.0;
            auto it_net_pairs = ccap_old_by_net_pair.find(net_name);
            if (it_net_pairs != ccap_old_by_net_pair.end()) {
                for (const auto& [pair_key, old_net_pair_sum] : it_net_pairs->second) {
                    double old_pair_total = ccap_old_total_by_pair[pair_key];
                    if (old_pair_total <= 0.0) continue;

                    auto it_target = ccap_target_by_pair.find(pair_key);
                    double pair_target_total = (it_target != ccap_target_by_pair.end())
                        ? it_target->second
                        : old_pair_total;

                    target_coupling_sum += pair_target_total * (old_net_pair_sum / old_pair_total);
                }
            }

            double target_noncoupling = target_total_cap - target_coupling_sum;
            double old_noncoupling = old_noncoupling_by_net[net_name];
            if (old_noncoupling > 0.0) {
                noncoupling_scale_by_net[net_name] = target_noncoupling / old_noncoupling;
            }
        }
    }
    
    std::cout << "[backmark] Nets with cap update: " << cap_ratio.size() << std::endl;
    std::cout << "[backmark] Nets with res update: " << res_avg_ratio.size() << std::endl;
    std::cout << "[backmark] Coupling pairs with ccap update: " << ccap_target_by_pair.size() << std::endl;
    
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
    double noncoupling_scale = 1.0;
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

            auto it_nc = noncoupling_scale_by_net.find(net_name_resolved);
            if (it_nc != noncoupling_scale_by_net.end()) noncoupling_scale = it_nc->second;
            else noncoupling_scale = 1.0;
            
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
                noncoupling_scale = 1.0;
                r_avg_scale = 1.0;
                r_edge_scales.clear();
                fout << line << "\n";
                lines_written++;
                continue;
            }
            
            // CAP section: update coupling first (if ccap target provided), otherwise apply cap scale.
            if (section == SEC_CAP) {
                if (std::regex_search(raw, re_cap_idx)) {
                    std::istringstream iss(raw);
                    std::vector<std::string> tokens;
                    std::string tok;
                    while (iss >> tok) tokens.push_back(tok);

                    if (tokens.size() >= 3) {
                        try {
                            double old_val = std::stod(tokens.back());
                            double new_val = old_val;
                            bool updated = false;

                            // Coupling cap line: idx node1 node2 value
                            if (tokens.size() >= 4 && use_ccap_backmark) {
                                std::string net1 = resolve_node_to_net(tokens[1]);
                                std::string net2 = resolve_node_to_net(tokens[2]);
                                if (!net1.empty() && !net2.empty() && net1 != net2) {
                                    std::string pair_key = make_pair_key(net1, net2);
                                    auto it_target = ccap_target_by_pair.find(pair_key);
                                    if (it_target != ccap_target_by_pair.end()) {
                                        double target_total = it_target->second;
                                        double old_total = ccap_old_total_by_pair[pair_key];
                                        size_t old_count = ccap_old_count_by_pair[pair_key];

                                        if (old_total > 0.0) {
                                            // Preserve original per-part ratio for this counterpart pair.
                                            new_val = target_total * (old_val / old_total);
                                        } else if (old_count > 0) {
                                            // If original total is zero, split evenly as a fallback.
                                            new_val = target_total / static_cast<double>(old_count);
                                        } else {
                                            new_val = target_total;
                                        }
                                        updated = true;
                                    }
                                }
                            }

                            // Fallback:
                            // - without --net-ccap-data: apply total-cap scale to all CAP entries.
                            // - with --net-ccap-data: apply (total-cap - coupling-cap) scale to self-cap entries.
                            bool is_coupling_line = (tokens.size() >= 4);
                            if (!updated) {
                                if (!use_ccap_backmark && c_scale != 1.0) {
                                    new_val = old_val * c_scale;
                                    updated = true;
                                } else if (use_ccap_backmark && !is_coupling_line && noncoupling_scale != 1.0) {
                                    new_val = old_val * noncoupling_scale;
                                    updated = true;
                                }
                            }

                            if (updated) {
                                tokens.back() = fmt_float(new_val);

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
                            }
                        } catch (...) {
                            // Not a CAP data line, fall through
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
                        (void)std::stoi(tokens[0]);  // validate index token, not otherwise used
                        
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

// ============== UNIT CONVERSION ==============

double convert_capacitance(double value, const std::string& from_unit) {
    // Convert to PF (picofarad) as standard
    if (from_unit == "PF" || from_unit == "pf" || from_unit == "pF") {
        return value;
    } else if (from_unit == "NF" || from_unit == "nf" || from_unit == "nF") {
        return value * 1000.0;  // 1 NF = 1000 PF
    } else if (from_unit == "UF" || from_unit == "uf" || from_unit == "uF" || from_unit == "µF") {
        return value * 1000000.0;  // 1 µF = 1000000 PF
    } else if (from_unit == "FF" || from_unit == "ff" || from_unit == "fF") {
        return value * 0.001;  // 1 FF = 0.001 PF
    } else if (from_unit == "F" || from_unit == "f") {
        return value * 1.0e12;  // 1 F = 1e12 PF
    }
    // Default: assume already PF
    return value;
}

double convert_resistance(double value, const std::string& from_unit) {
    // Convert to OHM as standard
    if (from_unit == "OHM" || from_unit == "ohm") {
        return value;
    } else if (from_unit == "KOHM" || from_unit == "kohm" || from_unit == "KOHM") {
        return value * 1000.0;  // 1 KOHM = 1000 OHM
    } else if (from_unit == "MOHM" || from_unit == "mohm" || from_unit == "MOHM") {
        return value * 1000000.0;  // 1 MOHM = 1000000 OHM
    }
    // Default: assume already OHM
    return value;
}

// ============== CORRELATION IMPLEMENTATIONS ==============

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
    int num_threads,
    int res_method
) {
    auto t_epd_start = std::chrono::steady_clock::now();

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
    auto t_common = std::chrono::steady_clock::now();
    
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
    std::vector<std::string> res_driver_names;
    res_net_names.reserve(n_cap * 4);
    res_sink_names.reserve(n_cap * 4);
    res_driver_names.reserve(n_cap * 4);
    
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
        std::vector<std::string> local_res_driver_names;
        
        local_cap_c1.reserve(n_cap / num_threads + 1);
        local_cap_c2.reserve(n_cap / num_threads + 1);
        local_cap_names.reserve(n_cap / num_threads + 1);
        
        for (size_t i = thread_id; i < common_nets.size(); i += num_threads) {
            const std::string& net_name = common_nets[i];
            auto& net1 = spef1.nets[net_name];
            auto& net2 = spef2.nets[net_name];
            
            // Capacitance - convert to standard unit (PF), apply *C_UNIT coefficient
            local_cap_c1.push_back(spef1.c_scale * convert_capacitance(net1.total_cap, spef1.c_unit));
            local_cap_c2.push_back(spef2.c_scale * convert_capacitance(net2.total_cap, spef2.c_unit));
            local_cap_names.push_back(net_name);
            
            // Resistance - convert to standard unit (OHM)
            auto res1 = compute_driver_sink_res_by_method(net1, res_method);
            auto res2 = compute_driver_sink_res_by_method(net2, res_method);
            
            // Find common sinks: O(k) hash lookup instead of O(k log k) sort + set_intersection
            for (const auto& [sink, r1] : res1) {
                auto it = res2.find(sink);
                if (it == res2.end()) continue;
                // Convert resistance to standard unit (OHM), apply *R_UNIT coefficient
                local_res_r1.push_back(spef1.r_scale * convert_resistance(r1, spef1.r_unit));
                local_res_r2.push_back(spef2.r_scale * convert_resistance(it->second, spef2.r_unit));
                local_res_net_names.push_back(net_name);
                local_res_sink_names.push_back(sink);
                local_res_driver_names.push_back(net1.driver);
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
            res_driver_names.insert(res_driver_names.end(), local_res_driver_names.begin(), local_res_driver_names.end());
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
    auto t_capres = std::chrono::steady_clock::now();
    
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
    result.res_driver_names = std::move(res_driver_names);
    
    // Compute correlations
    result.cap_correlation = compute_pearson_correlation(cap_c1_vec, cap_c2_vec);
    if (result.res_count > 0) {
        result.res_correlation = compute_pearson_correlation(res_r1_vec, res_r2_vec);
    }

    // Coupling capacitance comparison (intersection of pairs present in both SPEFs)
    {
        auto make_pair_key = [](const std::string& a, const std::string& b) -> std::string {
            return (a < b) ? (a + "|" + b) : (b + "|" + a);
        };

        std::unordered_map<std::string, double> caps1, caps2;
        for (const auto& cc : spef1.coupling_caps) {
            caps1[make_pair_key(cc.net1, cc.net2)] += cc.cap_value;
        }
        for (const auto& cc : spef2.coupling_caps) {
            caps2[make_pair_key(cc.net1, cc.net2)] += cc.cap_value;
        }

        std::vector<double> cc1_vec, cc2_vec;
        for (const auto& [key, c1] : caps1) {
            auto it = caps2.find(key);
            if (it == caps2.end()) continue;
            size_t pos = key.find('|');
            if (pos == std::string::npos) continue;
            result.ccap_net1_names.push_back(key.substr(0, pos));
            result.ccap_net2_names.push_back(key.substr(pos + 1));
            cc1_vec.push_back(c1);
            cc2_vec.push_back(it->second);
        }

        result.ccap_count = cc1_vec.size();
        result.ccap_c1 = py::array_t<double>(cc1_vec.size(), cc1_vec.data());
        result.ccap_c2 = py::array_t<double>(cc2_vec.size(), cc2_vec.data());
        if (!cc1_vec.empty()) {
            result.ccap_correlation = compute_pearson_correlation(cc1_vec, cc2_vec);
        }
    }
    auto t_epd_end = std::chrono::steady_clock::now();

    double epd_total     = std::chrono::duration<double>(t_epd_end - t_epd_start).count();
    double epd_common_s  = std::chrono::duration<double>(t_common  - t_epd_start).count();
    double epd_capres_s  = std::chrono::duration<double>(t_capres  - t_common   ).count();
    double epd_ccap_s    = std::chrono::duration<double>(t_epd_end - t_capres   ).count();

    std::cout << "[export_plot_data] total=" << epd_total << "s"
              << "  common_nets=" << epd_common_s << "s"
              << "  cap+res(" << num_threads << "t)=" << epd_capres_s << "s"
              << "  ccap_compare+corr=" << epd_ccap_s << "s"
              << "  nets=" << n_cap
              << "  res_pairs=" << result.res_count
              << "  ccaps=" << result.ccap_count
              << std::endl;

    return result;
}
