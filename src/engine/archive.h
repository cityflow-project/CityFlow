#ifndef CITYFLOW_ARCHIVE_H
#define CITYFLOW_ARCHIVE_H

#include "rapidjson/document.h"
#include "rapidjson/allocators.h"
#include "roadnet/roadnet.h"

#include <deque>

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
        std::map<const Drivable *, DrivableArchive> drivablesArchive;
        std::map<const Flow *, FlowArchive> flowsArchive;
        std::map<const Intersection *, TrafficLightArchive> trafficLightsArchive;
        size_t step;
        size_t activeVehicleCount;
        std::mt19937 rnd;

        int finishedVehicleCnt;
        double cumulativeTravelTime;

        static VehiclePool copyVehiclePool(const VehiclePool& src);
        static Vehicle *getNewPointer(const VehiclePool &vehiclePool, const Vehicle *old);
        void archiveDrivable(const Drivable *drivable, DrivableArchive &drivableArchive);
        void archiveFlow(const Flow *flow, FlowArchive &flowArchive);
        void archiveTrafficLight(const TrafficLight *light, TrafficLightArchive &trafficLightArchive);

        rapidjson::Value dumpVehicle(const Vehicle &vehicle, rapidjson::Document &jsonRoot) const;
        void dumpVehicles(rapidjson::Document &jsonRoot) const;
        void dumpDrivables(rapidjson::Document &jsonRoot) const;
        void dumpFlows(rapidjson::Document &jsonRoot) const;
        void dumpTrafficLights(rapidjson::Document &jsonRoot) const;

        template <typename T>
        static void addObjectAsMember(rapidjson::Value &jsonObject, const std::string &name,
                                      const T &object, rapidjson::MemoryPoolAllocator<> &allocator) {
            if (object) {
                jsonObject.AddMember(
                        rapidjson::Value(name, allocator).Move(),
                        rapidjson::Value(object->getId(), allocator).Move(),
                        allocator
                        );
            }
        }

        template <typename T>
        static void pushBackObjectAsMember(rapidjson::Value &jsonObject,
                                      const T &object, rapidjson::MemoryPoolAllocator<> &allocator) {
            if (object) {
                jsonObject.PushBack(
                        rapidjson::Value(object->getId(), allocator).Move(),
                        allocator
                );
            }
        }

    public:
        Archive() = default;
        explicit Archive(const Engine &engine);
        Archive(Engine &engine, const std::string &filename);
        void resume(Engine &engine) const;
        void dump(const std::string &fileName) const;
    };

}


#endif //CITYFLOW_ARCHIVE_H
