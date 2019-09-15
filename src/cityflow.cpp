#include "engine/engine.h"
#include "engine/archive.h"

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
        .def("get_vehicles", &CityFlow::Engine::getVehicles, "include_waiting"_a=false)
        .def("get_lane_vehicle_count", &CityFlow::Engine::getLaneVehicleCount)
        .def("get_lane_waiting_vehicle_count", &CityFlow::Engine::getLaneWaitingVehicleCount)
        .def("get_lane_vehicles", &CityFlow::Engine::getLaneVehicles)
        .def("get_vehicle_speed", &CityFlow::Engine::getVehicleSpeed)
        .def("get_vehicle_distance", &CityFlow::Engine::getVehicleDistance)
        .def("get_current_time", &CityFlow::Engine::getCurrentTime)
        .def("set_tl_phase", &CityFlow::Engine::setTrafficLightPhase, "intersection_id"_a, "phase_id"_a)
        .def("set_replay_file", &CityFlow::Engine::setReplayLogFile, "replay_file"_a)
        .def("set_random_seed", &CityFlow::Engine::setRandomSeed, "seed"_a)
        .def("set_save_replay", &CityFlow::Engine::setSaveReplay, "open"_a)
        .def("push_vehicle", (void (CityFlow::Engine::*)(const std::map<std::string, double>&, const std::vector<std::string>&)) &CityFlow::Engine::pushVehicle)
        .def("reset", &CityFlow::Engine::reset, "seed"_a=false)
        .def("load", &CityFlow::Engine::load, "archive"_a)
        .def("snapshot", &CityFlow::Engine::snapshot)
        .def("load_from_file", &CityFlow::Engine::loadFromFile, "path"_a);

    py::class_<CityFlow::Archive>(m, "Archive")
        .def(py::init<const CityFlow::Engine&>())
        .def("dump", &CityFlow::Archive::dump, "path"_a);
#ifdef VERSION
    m.attr("__version__") = BOOST_PP_STRINGIZE(VERSION);
#else
    m.attr("__version__") = "dev";
#endif
}