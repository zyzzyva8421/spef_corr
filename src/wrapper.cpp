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
    
    // Module functions
    m.def("parse_spef", &parse_spef, "Parse SPEF file",
          py::arg("filepath"));
    
    m.def("shuffle_spef", &shuffle_spef, "Shuffle SPEF net IDs",
          py::arg("input_path"),
          py::arg("output_path"),
          py::arg("seed") = -1);
}
