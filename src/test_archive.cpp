#include "engine/engine.h"
#include "engine/archive.h"
#include <boost/program_options.hpp>

#include <string>
#include <iostream>
#include <cstdlib>
#include <functional>
#include <chrono>

using namespace CityFlow;
namespace bpo = boost::program_options;
constexpr size_t period = 100;

bool test0(Engine &engine);
bool test1(Engine &engine);
bool test2(Engine &engine);
bool test3(Engine &engine);

int main(int argc, char* argv[]) {

    std::string configFile;
    int threadNum;

    bpo::options_description opts("all options");
    bpo::variables_map vm;

    opts.add_options()
            ("configFile", bpo::value<std::string>(&configFile)->required(), "config file")
            ("threadNum", bpo::value<int>(&threadNum)->default_value(1), "number of threads")
            ("help", "Test Parameters");

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

    Engine engine(dataDir + "/config/" + configFile, (unsigned int)threadNum);

    std::vector<std::function<bool(Engine &engine)>> testFunctions{test0, test1, test2, test3};
    std::vector<std::string> captions{
        "Single save, single load",
        "Single save, multiple loads",
        "Multiple saves, multiple loads",
        "Performance test"
        };
    for (size_t i = 0 ; i < period ; ++i) engine.nextStep();
    for (size_t testIndex = 0 ; testIndex < testFunctions.size() ; ++testIndex) {
        std::cout <<"[test " << testIndex << "] " << captions[testIndex] << std::endl;
        if (testFunctions[testIndex](engine))
            std::cout << "\33[1;32m" << "PASS" << "\33[0m" << std::endl;
        else {
            std::cout << "\33[1;31m" << "FAIL" << "\33[0m" << std::endl;
            return -1;
        }
    }
    return 0;
}

bool test0(Engine &engine)
{
    auto before = engine.getLaneVehicleCount();
    Archive archive(engine);
    for (size_t i = 0 ; i < period ; ++i) engine.nextStep();
    auto after1 = engine.getLaneVehicleCount();
    engine.load(archive);
    for (size_t i = 0 ; i < period ; ++i) engine.nextStep();
    auto after2 = engine.getLaneVehicleCount();
    return after1 == after2;
}

bool test1(Engine &engine)
{
    auto before = engine.getLaneVehicleCount();
    Archive archive(engine);
    for (size_t i = 0 ; i < period ; ++i) engine.nextStep();
    auto after1 = engine.getLaneVehicleCount();
    engine.load(archive);
    for (size_t i = 0 ; i < period ; ++i) engine.nextStep();
    auto after2 = engine.getLaneVehicleCount();
    engine.load(archive);
    for (size_t i = 0 ; i < period ; ++i) engine.nextStep();
    auto after3 = engine.getLaneVehicleCount();
    return after1 == after2 && after2 == after3;
}

bool test2(Engine &engine)
{
    /*
     *  step     0   100 200 300 400
     *  archive  0   1   2   3   4
     */
    constexpr int periodNum = 5;
    std::vector<Archive> archives;
    std::vector<decltype(engine.getLaneWaitingVehicleCount())> snapshot;
    for (size_t i = 0 ; i < period * periodNum ; ++i) {
        if (i % period == 0) {
            archives.emplace_back(engine);
            snapshot.push_back(engine.getLaneVehicleCount());
        }
    }
    for (size_t i = 0 ; i < period ; i += period) {
        engine.load(archives[i / period]);
        for (size_t j = i ; j < period * periodNum ; ++j) {
            if (j % period == 0) {
                auto current = engine.getLaneVehicleCount();
                if (current != snapshot[j / period])
                    return false;
            }
        }
    }
    return true;
}

bool test3(Engine &engine)
{
    constexpr size_t repeats = 1;
    auto before = engine.getLaneVehicleCount();
    Archive archive;
    auto t0 = std::chrono::steady_clock::now();
    for (size_t i = 0 ; i < repeats ; ++i) {
        archive = Archive(engine);
    }
    auto t1 = std::chrono::steady_clock::now();
    std::chrono::duration<double> saveTime = t1 - t0;
    for (size_t i = 0 ; i < period ; ++i) engine.nextStep();
    auto after1 = engine.getLaneVehicleCount();

    auto t2 = std::chrono::steady_clock::now();
    for (size_t i = 0 ; i < repeats ; ++i) {
        engine.load(archive);
    }
    auto t3 = std::chrono::steady_clock::now();
    std::chrono::duration<double> loadTime = t3 - t2;
    for (size_t i = 0 ; i < period ; ++i) engine.nextStep();
    auto after2 = engine.getLaneVehicleCount();

    std::cout << "save: " << saveTime.count()
              << " load: " << loadTime.count() << std::endl;
    return after1 == after2;
}