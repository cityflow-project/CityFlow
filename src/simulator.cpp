#include "engine/engine.h"

#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include <boost/preprocessor/stringize.hpp>

namespace py = pybind11;
using namespace py::literals;

PYBIND11_MODULE(cityflow, m) {
    py::class_<CityFlow::Engine>(m, "Engine")
        .def(py::init<const std::string&, int>(),
            "config_file"_a,
            "thread_num"_a=1
        )
        .def("next_step", &CityFlow::Engine::nextStep)
        .def("get_vehicle_count", &CityFlow::Engine::getVehicleCount)
        .def("get_lane_vehicle_count", &CityFlow::Engine::getLaneVehicleCount)
        .def("get_lane_waiting_vehicle_count", &CityFlow::Engine::getLaneWaitingVehicleCount)
		.def("get_lane_vehicles", &CityFlow::Engine::getLaneVehicles)
        .def("get_vehicle_speed", &CityFlow::Engine::getVehicleSpeed)
        .def("get_vehicle_distance", &CityFlow::Engine::getVehicleDistance)
        .def("get_current_time", &CityFlow::Engine::getCurrentTime)
        .def("set_tl_phase", &CityFlow::Engine::setTrafficLightPhase)
        .def("push_vehicle", (void (CityFlow::Engine::*)(const std::map<std::string, double>&, const std::vector<std::string>&)) &CityFlow::Engine::pushVehicle)
        .def("reset", &CityFlow::Engine::reset);
        
#ifdef VERSION
    m.attr("__version__") = BOOST_PP_STRINGIZE(VERSION);
#else
    m.attr("__version__") = "dev";
#endif
}