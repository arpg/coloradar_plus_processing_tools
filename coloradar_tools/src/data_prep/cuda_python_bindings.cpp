#include <pybind11/pybind11.h>
#include "coloradar_cuda.h"


namespace py = pybind11;


PYBIND11_MODULE(coloradar_cuda_tools, m) {
    py::module dataset_tools = py::module::import("coloradar_dataset_tools");
    py::object radar_config_class = dataset_tools.attr("RadarConfig");

    // RadarProcessor
    py::class_<coloradar::RadarProcessor, std::shared_ptr<coloradar::RadarProcessor>>(m, "RadarProcessor")
        .def(py::init<const coloradar::RadarConfig*, const double&, const double&, const double&>(),
             py::arg("config"),
             py::arg("blackmanParamA0") = 0.42,
             py::arg("blackmanParamA1") = 0.5,
             py::arg("blackmanParamA2") = 0.08)
        .def("cubeToHeatmap", &coloradar::RadarProcessor::cubeToHeatmap,
             py::arg("datacube"),
             py::arg("applyCollapseDoppler") = false,
             py::arg("removeAntennaCoupling") = false,
             py::arg("applyPhaseFrequencyCalib") = false)
         .def_readonly("config", &coloradar::RadarProcessor::config);
}
