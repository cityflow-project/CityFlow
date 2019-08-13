#ifndef CITYFLOW_ARCHIVE_H
#define CITYFLOW_ARCHIVE_H

#include <deque>
#include <vector>
#include <map>
#include <list>
#include <random>
#include "roadnet/roadnet.h"

namespace CityFlow {
    class Engine;
    class Flow;
    class Vehicle;
    class TrafficLight;

    class Archive {
    private:
        using VehiclePool = std::map<int, std::pair<Vehicle *, int>>;
        struct TrafficLightArchive {
            double remainDuration;
            int curPhaseIndex;
        };

        struct FlowArchive {
            double nowTime;
            double currentTime;
            int cnt;
        };

        struct DrivableArchive {
            std::list<Vehicle *> vehicles;
            std::deque<Vehicle *> waitingBuffer;

            std::list<Lane::HistoryRecord> history;
            int    historyVehicleNum = 0;
            double historyAverageSpeed = 0;
        };

        VehiclePool vehiclePool;
        std::map<const Drivable *, DrivableArchive> drivableArchive;
        std::map<const Flow *, FlowArchive> flowArchive;
        std::map<const TrafficLight *, TrafficLightArchive> trafficLightArchive;
        size_t step;
        size_t activeVehicleCount;
        std::mt19937 rnd;

        static VehiclePool copyVehiclePool(const VehiclePool& src);
        static Vehicle *getNewPointer(const VehiclePool &vehiclePool, const Vehicle *old);
        void archiveDrivable(const Drivable *drivable, DrivableArchive &drivableArchive);
        void archiveFlow(const Flow *flow, FlowArchive &flowArchive);
        void archiveTrafficLight(const TrafficLight *light, TrafficLightArchive &trafficLightArchive);

    public:
        Archive() = default;
        explicit Archive(const Engine &engine);
        void resume(Engine &engine) const;
    };

}


#endif //CITYFLOW_ARCHIVE_H
