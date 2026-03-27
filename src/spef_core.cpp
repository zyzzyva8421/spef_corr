#include "spef_core.h"

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
    
    std::string line;
    size_t line_num = 0;
    
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
                    std::cout << "[" << filepath << "] parsed " << net_count << " nets... (C++)" << std::endl;
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
                std::string node1 = tokens[1];
                std::string node2 = tokens[2];
                double rval = parse_float(tokens[3]);
                if (r_is_kohm) rval *= 1000.0;
                
                // Unescape node names
                size_t pos;
                while ((pos = node1.find("\\[")) != std::string::npos) node1.replace(pos, 2, "[");
                while ((pos = node1.find("\\]")) != std::string::npos) node1.replace(pos, 2, "]");
                while ((pos = node2.find("\\[")) != std::string::npos) node2.replace(pos, 2, "[");
                while ((pos = node2.find("\\]")) != std::string::npos) node2.replace(pos, 2, "]");
                
                // Resolve via name_map
                if (node1.size() > 1 && node1[0] == '*') {
                    auto it = spef.name_map.find(node1);
                    if (it != spef.name_map.end()) node1 = it->second;
                }
                if (node2.size() > 1 && node2[0] == '*') {
                    auto it = spef.name_map.find(node2);
                    if (it != spef.name_map.end()) node2 = it->second;
                }
                
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
            std::string pin = tokens[1];
            // Direction is the first token that starts with O, B, or I
            std::string dir;
            for (size_t i = 2; i < tokens.size(); i++) {
                if (!tokens[i].empty() && (tokens[i][0] == 'O' || tokens[i][0] == 'B' || tokens[i][0] == 'I')) {
                    dir = tokens[i];
                    break;
                }
            }
            
            // Handle pin:idx format
            std::string pin_base = pin;
            std::string pin_idx;
            size_t colon_pos = pin.find(':');
            if (colon_pos != std::string::npos) {
                pin_base = pin.substr(0, colon_pos);
                pin_idx = pin.substr(colon_pos);  // e.g., ":Q"
            }
            
            // Resolve pin_base via name_map
            if (pin_base.size() > 1 && pin_base[0] == '*') {
                auto it = spef.name_map.find(pin_base);
                if (it != spef.name_map.end()) {
                    pin = it->second + pin_idx;
                }
            }
            
            // Unescape
            size_t pos;
            while ((pos = pin.find("\\[")) != std::string::npos) pin.replace(pos, 2, "[");
            while ((pos = pin.find("\\]")) != std::string::npos) pin.replace(pos, 2, "]");
            
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
            std::cerr << "DEBUG SEC_NONE block, tokens[0]=" << (tokens.empty() ? "empty" : tokens[0]) << std::endl;
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
                    std::cerr << "DEBUG: Processing *I/*P line, pin=" << tokens[1] << " dir=" << tokens[2] << std::endl;
                    // Process this line as CONN
                    std::string pin = tokens[1];
                    std::string dir;
                    for (size_t i = 2; i < tokens.size(); i++) {
                        if (!tokens[i].empty() && (tokens[i][0] == 'O' || tokens[i][0] == 'B' || tokens[i][0] == 'I')) {
                            dir = tokens[i];
                            break;
                        }
                    }
                    // Handle pin:idx format
                    std::string pin_base = pin;
                    std::string pin_idx;
                    size_t colon_pos = pin.find(':');
                    if (colon_pos != std::string::npos) {
                        pin_base = pin.substr(0, colon_pos);
                        pin_idx = pin.substr(colon_pos);
                    }
                    if (pin_base.size() > 1 && pin_base[0] == '*') {
                        auto it = spef.name_map.find(pin_base);
                        if (it != spef.name_map.end()) {
                            pin = it->second + pin_idx;
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
