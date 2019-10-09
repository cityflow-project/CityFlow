#include "engine/engine.h"
#include <string>
#include <cstdlib>
#include <gtest/gtest.h>

using namespace CityFlow;

size_t threads = std::min(std::thread::hardware_concurrency(), 4u);
std::string configFile = "examples/config.json";

TEST(Basic, Basic) {
    size_t totalStep = 2000;

    Engine engine(configFile, threads);
    for (size_t i = 0; i < totalStep; i++) {
        engine.nextStep();
    }
    SUCCEED();
}

TEST(Basic, API) {
    size_t totalStep = 200;

    Engine engine(configFile, threads);
    for (size_t i = 0; i < totalStep; i++) {
        engine.nextStep();
        engine.getVehicleSpeed();
        engine.getLaneVehicles();
        engine.getLaneWaitingVehicleCount();
        engine.getVehicleDistance();
        engine.getCurrentTime();
        engine.getVehicleCount();
    }
    SUCCEED();
}

TEST(Basic, reset) {
    size_t totalStep = 200;

    Engine engine(configFile, threads);
    for (size_t i = 0; i < totalStep; i++) {
        engine.nextStep();
    }
    double curTime = engine.getCurrentTime();
    size_t vehCnt = engine.getVehicleCount();
    engine.reset(true);
    for (size_t i = 0; i < totalStep; i++) {
        engine.nextStep();
    }
    EXPECT_EQ(engine.getCurrentTime(), curTime);
    EXPECT_EQ(engine.getVehicleCount(), vehCnt);
    SUCCEED();
}

int main(int argc, char* argv[]) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}