#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
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
    
    // ============== NEW BINDINGS FOR RECOMMENDATIONS ==============
    
    // Recommendation 1: ResistanceResult struct
    py::class_<ResistanceResult>(m, "ResistanceResult")
        .def(py::init<>())
        .def_readwrite("net_name", &ResistanceResult::net_name)
        .def_readwrite("sink_pin", &ResistanceResult::sink_pin)
        .def_readwrite("resistance", &ResistanceResult::resistance);
    
    // Recommendation 2: CorrelationResult struct
    py::class_<CorrelationResult>(m, "CorrelationResult")
        .def(py::init<>())
        .def_readwrite("pearson", &CorrelationResult::pearson)
        .def_readwrite("mean_x", &CorrelationResult::mean_x)
        .def_readwrite("mean_y", &CorrelationResult::mean_y)
        .def_readwrite("std_dev_x", &CorrelationResult::std_dev_x)
        .def_readwrite("std_dev_y", &CorrelationResult::std_dev_y)
        .def_readwrite("valid", &CorrelationResult::valid);
    
    // Recommendation 4: CapComparisonData struct
    py::class_<CapComparisonData>(m, "CapComparisonData")
        .def(py::init<>())
        .def_readwrite("net_name", &CapComparisonData::net_name)
        .def_readwrite("c1", &CapComparisonData::c1)
        .def_readwrite("c2", &CapComparisonData::c2);
    
    // Recommendation 4: ResComparisonData struct
    py::class_<ResComparisonData>(m, "ResComparisonData")
        .def(py::init<>())
        .def_readwrite("net_name", &ResComparisonData::net_name)
        .def_readwrite("driver", &ResComparisonData::driver)
        .def_readwrite("sink", &ResComparisonData::sink)
        .def_readwrite("r1", &ResComparisonData::r1)
        .def_readwrite("r2", &ResComparisonData::r2);
    
    // Module functions
    m.def("parse_spef", &parse_spef, "Parse SPEF file",
          py::arg("filepath"));
    
    m.def("shuffle_spef", &shuffle_spef, "Shuffle SPEF net IDs",
          py::arg("input_path"),
          py::arg("output_path"),
          py::arg("seed") = -1);
    
    // Recommendation 3: Pre-compute pin maps
    m.def("build_pin_to_node_map", &build_pin_to_node_map, 
          "Build pin-to-node mapping for a net",
          py::arg("net"));
    
    // Recommendation 1: Batch resistance computation
    m.def("compute_batch_driver_sink_resistances",
          &compute_batch_driver_sink_resistances,
          "Compute resistances for multiple nets in parallel",
          py::arg("net_names"),
          py::arg("spef"),
          py::arg("num_threads") = 0);
    
    // Recommendation 2: Correlation computation
    m.def("compute_pearson_correlation",
          &compute_pearson_correlation,
          "Compute Pearson correlation coefficient",
          py::arg("xs"),
          py::arg("ys"));
    
    // Recommendation 4: Streaming comparison
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
}
