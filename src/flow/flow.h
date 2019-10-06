#ifndef CITYFLOW_FLOW_H
#define CITYFLOW_FLOW_H

#include "vehicle/vehicle.h"
#include "flow/route.h"


namespace CityFlow {
    class Engine;

    class Flow {
        friend class Archive;
    private:
        VehicleInfo vehicleTemplate;
        std::shared_ptr<const Route> route;
        double interval;
        double nowTime = 0;
        double currentTime = 0;
        int startTime = 0;
        int endTime = -1;
        int cnt = 0;
        Engine *engine;
        std::string id;

    public:
        Flow(const VehicleInfo &vehicleTemplate, double timeInterval,
            Engine *engine, int startTime, int endTime, const std::string &id) 
            : vehicleTemplate(vehicleTemplate), interval(timeInterval),
              startTime(startTime), endTime(endTime), engine(engine), id(id) {
            assert(timeInterval >= 1 || (startTime == endTime));
            nowTime = interval;
        }

        void nextStep(double timeInterval);

        std::string getId() const;

        void reset();

    };
}

#endif //CITYFLOW_FLOW_H
