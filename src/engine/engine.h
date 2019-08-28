#ifndef CITYFLOW_ENGINE_H
#define CITYFLOW_ENGINE_H

#include "flow/flow.h"
#include "vehicle/vehicle.h"
#include "roadnet/roadnet.h"
#include "engine/archive.h"

#include <boost/thread/mutex.hpp>
#include <boost/thread/barrier.hpp>
#include <boost/thread/thread.hpp>
#include <boost/ref.hpp>

#include <utility>
#include <vector>
#include <map>
#include <set>
#include <random>
#include <string>
#include <fstream>


namespace CityFlow {

    class Engine {
        friend class Archive;
    private:
        static bool vehicleCmp(const std::pair<Vehicle *, double> &a, const std::pair<Vehicle *, double> &b) {
            return a.second > b.second;
        }

        std::map<int, std::pair<Vehicle *, int>> vehiclePool;
        std::vector<std::set<Vehicle *>> threadVehiclePool;
        std::vector<std::vector<Road *>> threadRoadPool;
        std::vector<std::vector<Intersection *>> threadIntersectionPool;
        std::vector<std::vector<Drivable *>> threadDrivablePool;
        std::vector<Flow> flows;
        RoadNet roadnet;
        int threadNum;
        double interval;
        bool saveReplay;
        bool warnings;
        std::vector<std::pair<Vehicle *, double>> pushBuffer;
        std::vector<Vehicle *> laneChangeNotifyBuffer;
        std::set<Vehicle *> vehicleRemoveBuffer;
        rapidjson::Document jsonRoot;
        std::string stepLog;

        size_t step = 0;
        size_t activeVehicleCount = 0;
        boost::mutex lock;
        boost::barrier startBarrier, endBarrier;
        std::vector<boost::thread> threadPool;
        bool finished = false;
        std::ofstream logOut;

        bool rlTrafficLight;
        bool laneChange;
        int manuallyPushCnt = 0;

    private:
        void vehicleControl(Vehicle &vehicle, std::vector<std::pair<Vehicle *, double>> &buffer);

        void getAction();

        void updateAction();

        void updateLocation();

        void updateLeaderAndGap();

        void planLaneChange();


        void threadController(std::set<Vehicle *> &vehicles, 
                              std::vector<Road *> &roads,
                              std::vector<Intersection *> &intersections,
                              std::vector<Drivable *> &drivables);

        void threadGetAction(std::set<Vehicle *> &vehicles);

        void threadUpdateAction(std::set<Vehicle *> &vehicles);

        void threadUpdateLeaderAndGap(const std::vector<Drivable *> &drivables);

        void threadUpdateLocation(const std::vector<Drivable *> &drivables);

        void threadNotifyCross(const std::vector<Intersection *> &intersections);

        void threadInitSegments(const std::vector<Road *> &roads);

        void threadPlanLaneChange(const std::set<Vehicle *> &vehicles);

        void handleWaiting();

        void updateLog();

        bool checkWarning();

        bool loadRoadNet(const std::string &jsonFile);

        bool loadFlow(const std::string &jsonFilename);

        std::vector<const Vehicle *> getRunningVehicles(bool includeWaiting=false) const;

        void scheduleLaneChange();

        void insertShadow(Vehicle *vehicle);

    public:
        std::mt19937 rnd;

        Engine(const std::string &configFile, int threadNum);

        double getInterval() const { return interval; }

        bool hasLaneChange() const { return laneChange; }

        bool loadConfig(const std::string &configFile);

        void notifyCross();

        void nextStep();

        bool checkPriority(int priority);

        void pushVehicle(Vehicle *const vehicle);

        void setLogFile(const std::string &jsonFile, const std::string &logFile);

        void initSegments();

        ~Engine();

        // RL related api

        void pushVehicle(const std::map<std::string, double> &info, const std::vector<std::string> &roads);

        size_t getVehicleCount() const;

        std::vector<std::string> getVehicles(bool includeWaiting = false) const;

        std::map<std::string, int> getLaneVehicleCount() const;

        std::map<std::string, int> getLaneWaitingVehicleCount() const;

        std::map<std::string, std::vector<std::string>> getLaneVehicles();

        std::map<std::string, double> getVehicleSpeed() const;

        std::map<std::string, double> getVehicleDistance() const;

        double getCurrentTime() const;

        void setTrafficLightPhase(const std::string &id, int phaseIndex);

        void reset();

        // archive
        void load(const Archive &archive) { archive.resume(*this); }
        Archive snapshot() { return Archive(*this); }
    };

}

#endif //CITYFLOW_ENGINE_H
