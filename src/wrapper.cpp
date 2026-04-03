#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include "spef_core.h"

namespace py = pybind11;

PYBIND11_MODULE(spef_core, m) {
    m.doc() = "SPEF C++ core - optimized parser and shortest path";
    
    // Edge struct
    py::class_<Edge>(m, "Edge")
        .def(py::init<>())
        .def_readwrite("to", &Edge::to)
        .def_readwrite("weight", &Edge::weight);
    
    // NetData class
    py::class_<NetData>(m, "NetData")
        .def(py::init<>())
        .def_readwrite("name", &NetData::name)
        .def_readwrite("total_cap", &NetData::total_cap)
        .def_readwrite("driver", &NetData::driver)
        .def_readwrite("sinks", &NetData::sinks)
        .def_readwrite("res_graph", &NetData::res_graph)
        .def("compute_driver_sink_resistances", &compute_driver_sink_resistances);
    
    // ParsedSpef class
    py::class_<ParsedSpef>(m, "ParsedSpef")
        .def(py::init<>())
        .def_readwrite("name_map", &ParsedSpef::name_map)
        .def_readwrite("nets", &ParsedSpef::nets)
        .def_readwrite("t_unit", &ParsedSpef::t_unit)
        .def_readwrite("c_unit", &ParsedSpef::c_unit)
        .def_readwrite("r_unit", &ParsedSpef::r_unit)
        .def_readwrite("l_unit", &ParsedSpef::l_unit);
    
    // ResistanceResult struct
    py::class_<ResistanceResult>(m, "ResistanceResult")
        .def(py::init<>())
        .def_readwrite("net_name", &ResistanceResult::net_name)
        .def_readwrite("sink_pin", &ResistanceResult::sink_pin)
        .def_readwrite("resistance", &ResistanceResult::resistance);
    
    // CorrelationResult struct
    py::class_<CorrelationResult>(m, "CorrelationResult")
        .def(py::init<>())
        .def_readwrite("pearson", &CorrelationResult::pearson)
        .def_readwrite("mean_x", &CorrelationResult::mean_x)
        .def_readwrite("mean_y", &CorrelationResult::mean_y)
        .def_readwrite("std_dev_x", &CorrelationResult::std_dev_x)
        .def_readwrite("std_dev_y", &CorrelationResult::std_dev_y)
        .def_readwrite("valid", &CorrelationResult::valid);
    
    // CapComparisonData struct
    py::class_<CapComparisonData>(m, "CapComparisonData")
        .def(py::init<>())
        .def_readwrite("net_name", &CapComparisonData::net_name)
        .def_readwrite("c1", &CapComparisonData::c1)
        .def_readwrite("c2", &CapComparisonData::c2);
    
    // ResComparisonData struct
    py::class_<ResComparisonData>(m, "ResComparisonData")
        .def(py::init<>())
        .def_readwrite("net_name", &ResComparisonData::net_name)
        .def_readwrite("driver", &ResComparisonData::driver)
        .def_readwrite("sink", &ResComparisonData::sink)
        .def_readwrite("r1", &ResComparisonData::r1)
        .def_readwrite("r2", &ResComparisonData::r2);
    
    // ComparisonResult struct
    py::class_<ComparisonResult>(m, "ComparisonResult")
        .def(py::init<>())
        .def_readwrite("cap_rows", &ComparisonResult::cap_rows)
        .def_readwrite("res_rows", &ComparisonResult::res_rows)
        .def_readwrite("top_10_cap", &ComparisonResult::top_10_cap)
        .def_readwrite("top_10_res", &ComparisonResult::top_10_res)
        .def_readwrite("common_nets", &ComparisonResult::common_nets)
        .def_readwrite("cap_correlation", &ComparisonResult::cap_correlation)
        .def_readwrite("res_correlation", &ComparisonResult::res_correlation)
        .def_readwrite("cap_count", &ComparisonResult::cap_count)
        .def_readwrite("res_count", &ComparisonResult::res_count);
    
    // Module functions
    m.def("parse_spef", &parse_spef, "Parse SPEF file", py::arg("filepath"));
    
    m.def("shuffle_spef", &shuffle_spef, "Shuffle SPEF net IDs",
          py::arg("input_path"),
          py::arg("output_path"),
          py::arg("seed") = -1);
    
    m.def("build_pin_to_node_map", &build_pin_to_node_map, 
          "Build pin-to-node mapping for a net",
          py::arg("net"));
    
    m.def("compute_batch_driver_sink_resistances",
          &compute_batch_driver_sink_resistances,
          "Compute resistances for multiple nets in parallel",
          py::arg("net_names"),
          py::arg("spef"),
          py::arg("num_threads") = 0);
    
    m.def("compute_pearson_correlation", &compute_pearson_correlation,
          "Compute Pearson correlation coefficient",
          py::arg("xs"),
          py::arg("ys"));
    
    m.def("compare_spef_streaming",
          [](ParsedSpef& spef1, ParsedSpef& spef2, 
             py::object py_on_cap, py::object py_on_res, int num_threads) {
              auto on_cap = [&](const CapComparisonData& data) {
                  py_on_cap(data);
              };
              auto on_res = [&](const ResComparisonData& data) {
                  py_on_res(data);
              };
              compare_spef_streaming(spef1, spef2, on_cap, on_res, num_threads);
          },
          "Stream-based comparison of two SPEF files",
          py::arg("spef1"),
          py::arg("spef2"),
          py::arg("on_cap_callback"),
          py::arg("on_res_callback"),
          py::arg("num_threads") = 0);
    
    m.def("backmark_spef", &backmark_spef,
          "Rewrite SPEF with updated cap/res values from data files",
          py::arg("spef_path"),
          py::arg("cap_data_path"),
          py::arg("res_data_path"),
          py::arg("output_path"));
    
    m.def("parse_backmark_cap_data", &parse_backmark_cap_data,
          "Parse backmark cap data file",
          py::arg("path"));
    
    m.def("parse_backmark_res_data", &parse_backmark_res_data,
          "Parse backmark res data file",
          py::arg("path"));
    
    m.def("parse_cap_data", &parse_cap_data,
          "Parse CSV cap data file (net,c1,c2 format)",
          py::arg("path"));
    
    m.def("parse_res_data", &parse_res_data,
          "Parse CSV res data file (net,r1,r2 format)",
          py::arg("path"));
    
    m.def("create_plot_data_from_files", &create_plot_data_from_files,
          "Create PlotData from CSV cap/res files",
          py::arg("cap_path"),
          py::arg("res_path"));
    
    m.def("compute_res_segment_scales", &compute_res_segment_scales,
          "Compute resistance segment scales for backmarking",
          py::arg("net"),
          py::arg("sink_ratios"),
          py::arg("avg_ratio"));
    
    m.def("compare_spef_full", &compare_spef_full,
          "Compare two SPEFs and return all results",
          py::arg("spef1"),
          py::arg("spef2"),
          py::arg("num_threads") = 0);
    
    m.def("summarize_comparison", &summarize_comparison,
          "Generate text summary of comparison result",
          py::arg("result"));
    
    m.def("parse_spef_parallel", &parse_spef_parallel,
          "Parse multiple SPEF files in parallel using C++ threads",
          py::arg("filepaths"),
          py::arg("num_threads") = 0);
    
    // ============== New functions for large dataset optimization ==============
    
    // PlotData struct bindings
    py::class_<PlotData>(m, "PlotData")
        .def(py::init<>())
        .def_readwrite("cap_c1", &PlotData::cap_c1)
        .def_readwrite("cap_c2", &PlotData::cap_c2)
        .def_readwrite("cap_net_names", &PlotData::cap_net_names)
        .def_readwrite("res_r1", &PlotData::res_r1)
        .def_readwrite("res_r2", &PlotData::res_r2)
        .def_readwrite("res_net_names", &PlotData::res_net_names)
        .def_readwrite("res_sink_names", &PlotData::res_sink_names)
        .def_readwrite("cap_correlation", &PlotData::cap_correlation)
        .def_readwrite("res_correlation", &PlotData::res_correlation)
        .def_readwrite("cap_count", &PlotData::cap_count)
        .def_readwrite("res_count", &PlotData::res_count);
    
    // ComparisonChunk struct bindings
    py::class_<ComparisonChunk>(m, "ComparisonChunk")
        .def(py::init<>())
        .def_readwrite("cap_c1", &ComparisonChunk::cap_c1)
        .def_readwrite("cap_c2", &ComparisonChunk::cap_c2)
        .def_readwrite("res_r1", &ComparisonChunk::res_r1)
        .def_readwrite("res_r2", &ComparisonChunk::res_r2)
        .def_readwrite("cap_net_names", &ComparisonChunk::cap_net_names)
        .def_readwrite("res_net_names", &ComparisonChunk::res_net_names)
        .def_readwrite("res_sink_names", &ComparisonChunk::res_sink_names)
        .def_readwrite("is_last", &ComparisonChunk::is_last);
    
    // New optimized functions
    m.def("export_plot_data", &export_plot_data,
          "Export comparison results as numpy arrays for fast plotting",
          py::arg("spef1"),
          py::arg("spef2"),
          py::arg("num_threads") = 0);
    
    m.def("compare_spef_chunk", &compare_spef_chunk,
          "Compare SPEF files in chunks for large datasets",
          py::arg("spef1"),
          py::arg("spef2"),
          py::arg("start_idx"),
          py::arg("chunk_size"),
          py::arg("num_threads") = 0);
    
    // Unit conversion functions - exposed for testing
    m.def("convert_capacitance", &convert_capacitance,
          "Convert capacitance to standard unit (PF)",
          py::arg("value"),
          py::arg("unit"));
    
    m.def("convert_resistance", &convert_resistance,
          "Convert resistance to standard unit (OHM)",
          py::arg("value"),
          py::arg("unit"));
}
