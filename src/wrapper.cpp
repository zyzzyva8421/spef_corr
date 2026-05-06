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
    
    // CouplingCap struct
    py::class_<CouplingCap>(m, "CouplingCap")
        .def(py::init<>())
        .def_readwrite("net1", &CouplingCap::net1)
        .def_readwrite("net2", &CouplingCap::net2)
        .def_readwrite("cap_value", &CouplingCap::cap_value);
    
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
        .def_readwrite("coupling_caps", &ParsedSpef::coupling_caps)
        .def_readwrite("t_unit", &ParsedSpef::t_unit)
        .def_readwrite("c_unit", &ParsedSpef::c_unit)
        .def_readwrite("r_unit", &ParsedSpef::r_unit)
        .def_readwrite("l_unit", &ParsedSpef::l_unit);

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
        .def_readwrite("res_driver_names", &PlotData::res_driver_names)
        .def_readwrite("ccap_c1", &PlotData::ccap_c1)
        .def_readwrite("ccap_c2", &PlotData::ccap_c2)
        .def_readwrite("ccap_net1_names", &PlotData::ccap_net1_names)
        .def_readwrite("ccap_net2_names", &PlotData::ccap_net2_names)
        .def_readwrite("cap_correlation", &PlotData::cap_correlation)
        .def_readwrite("res_correlation", &PlotData::res_correlation)
        .def_readwrite("ccap_correlation", &PlotData::ccap_correlation)
        .def_readwrite("cap_count", &PlotData::cap_count)
        .def_readwrite("res_count", &PlotData::res_count)
        .def_readwrite("ccap_count", &PlotData::ccap_count);

    // Module functions
    m.def("parse_spef", &parse_spef, "Parse SPEF file", py::arg("filepath"));

    m.def("shuffle_spef", &shuffle_spef, "Shuffle SPEF net IDs",
          py::arg("input_path"),
          py::arg("output_path"),
          py::arg("seed") = -1);

    m.def("backmark_spef", &backmark_spef,
          "Rewrite SPEF with updated cap/res/coupling-cap values from data files",
          py::arg("spef_path"),
          py::arg("cap_data_path"),
          py::arg("res_data_path"),
          py::arg("ccap_data_path"),
          py::arg("output_path"),
          py::arg("res_method") = 0);

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
          "Create PlotData from cap/res/ccap files",
          py::arg("cap_path"),
          py::arg("res_path"),
          py::arg("ccap_path") = "");

    m.def("compute_res_segment_scales", &compute_res_segment_scales,
          "Compute resistance segment scales for backmarking",
          py::arg("net"),
          py::arg("sink_ratios"),
          py::arg("avg_ratio"));

    m.def("compute_equivalent_resistance", &compute_equivalent_resistance,
          "Compute Thevenin equivalent resistance between two nodes using nodal analysis",
          py::arg("graph"),
          py::arg("source"),
          py::arg("sink"));

    m.def("parse_spef_parallel", &parse_spef_parallel,
          "Parse multiple SPEF files in parallel using C++ threads",
          py::arg("filepaths"),
          py::arg("num_threads") = 0);

    m.def("export_plot_data", &export_plot_data,
          "Export comparison results as numpy arrays for fast plotting",
          py::arg("spef1"),
          py::arg("spef2"),
          py::arg("num_threads") = 0,
          py::arg("res_method") = 0);

    // Unit conversion functions
    m.def("convert_capacitance", &convert_capacitance,
          "Convert capacitance to standard unit (PF)",
          py::arg("value"),
          py::arg("unit"));

    m.def("convert_resistance", &convert_resistance,
          "Convert resistance to standard unit (OHM)",
          py::arg("value"),
          py::arg("unit"));
}

