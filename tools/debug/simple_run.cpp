#include "engine/engine.h"
#include "utility/optionparser.h"

#include <string>
#include <iostream>
#include <cstdlib>
#include <ctime>

using namespace CityFlow;

int main(int argc, char const *argv[]) {
    optionparser::OptionParser parser;

    parser.add_option("--configFile", "-c")
            .help("config file")
            .mode(optionparser::StorageMode::STORE_VALUE)
            .required(true);

    parser.add_option("--totalStep", "-s")
            .help("simulation steps")
            .default_value(1000)
            .mode(optionparser::StorageMode::STORE_VALUE);

    parser.add_option("--threadNum", "-t")
            .help("number of threads")
            .default_value(1)
            .mode(optionparser::StorageMode::STORE_VALUE);

    parser.add_option("--verbose", "-v")
            .help("be verbose")
            .mode(optionparser::StorageMode::STORE_TRUE);

    parser.eat_arguments(argc, argv);
    std::string configFile = parser.get_value<std::string>("configFile");
    bool verbose = parser.get_value("verbose");
    size_t totalStep = parser.get_value<int>("totalStep");
    size_t threadNum = parser.get_value<int>("threadNum");

    std::string dataDir(std::getenv("DATADIR"));

    Engine engine(dataDir + configFile, (size_t) threadNum);
    time_t startTime, endTime;
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