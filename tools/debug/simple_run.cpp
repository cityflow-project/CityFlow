#include "engine/engine.h"
#include <boost/program_options.hpp>

#include <string>
#include <iostream>
#include <cstdlib>
#include <ctime>

using namespace CityFlow;
namespace bpo = boost::program_options;

int main(int argc, char* argv[]) {

    std::string configFile;
    bool verbose;
    int totalStep, threadNum;

    bpo::options_description opts("all options");
    bpo::variables_map vm;

    opts.add_options()
        ("configFile", bpo::value<std::string>(&configFile)->required(), "config file")
        ("totalStep", bpo::value<int>(&totalStep)->default_value(1000), "simulation steps")
        ("threadNum", bpo::value<int>(&threadNum)->default_value(1), "number of threads")
        ("verbose", bpo::bool_switch(&verbose)->default_value(false), "be verbose")
        ("help", "Simulator Parameters");

    try {
        bpo::store(bpo::parse_command_line(argc, argv, opts), vm);
        bpo::notify(vm);
    }
    catch (const bpo::error &e) {
        std::cout << e.what() << std::endl;
        return 0;
    }

    if (vm.count("help")) {
        std::cout << opts << std::endl;
        return 0;
    }

    std::string dataDir(std::getenv("DATADIR"));

    Engine engine(dataDir + "/config/" + configFile, (size_t)threadNum);
    time_t startTime,endTime;
    time(&startTime);
    for (int i = 0; i < totalStep; i++) {
        if (verbose) {
            std::cout << i << " " << engine.getVehicleCount() << std::endl;
        }
        engine.nextStep();
        //engine.getVehicleSpeed();
        //engine.getLaneVehicles();
        //engine.getLaneWaitingVehicleCount();
        //engine.getVehicleDistance();
        //engine.getCurrentTime();
    }
    time(&endTime);
    std::cout << "Total Step: " << totalStep << std::endl;
    std::cout << "Total Time: " << (endTime - startTime) << "s" << std::endl;
    return 0;
} 