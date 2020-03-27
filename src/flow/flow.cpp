#include "flow/flow.h"
#include "engine/engine.h"


namespace CityFlow {
    void Flow::nextStep(double timeInterval) {
        if (!valid) return;
        if (endTime != -1 && currentTime > endTime) return;
        if (currentTime >= startTime) {
            while (nowTime >= interval) {
                Vehicle* vehicle = new Vehicle(vehicleTemplate, id + "_" + std::to_string(cnt++), engine, this);
                int priority = vehicle->getPriority();
                while (engine->checkPriority(priority)) priority = engine->rnd();
                vehicle->setPriority(priority);
                engine->pushVehicle(vehicle, false);
                vehicle->getFirstRoad()->addPlanRouteVehicle(vehicle);
                nowTime -= interval;
            }
            nowTime += timeInterval;
        }
        currentTime += timeInterval;
    }

    std::string Flow::getId() const {
        return id;
    }

    void Flow::reset() {
        nowTime = interval;
        currentTime = 0;
        cnt = 0;
    }
}