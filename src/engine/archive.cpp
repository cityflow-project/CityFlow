#include "engine/archive.h"
#include "engine/engine.h"
#include "flow/flow.h"
#include "roadnet/roadnet.h"
#include "roadnet/trafficlight.h"
#include "vehicle/vehicle.h"

namespace CityFlow {

    Archive::Archive(const Engine &engine)
    : step(engine.step), activeVehicleCount(engine.activeVehicleCount), rnd(engine.rnd) {
        // copy the vehicle Pool
        vehiclePool = copyVehiclePool(engine.vehiclePool);

        // record the information of each drivable object
        for (const auto &drivable : engine.roadnet.getDrivables()) {
            auto result = drivableArchive.emplace(drivable, DrivableArchive());
            assert(result.second);
            archiveDrivable(drivable, result.first->second);
        }

        // record the information of each flow
        for (const auto &flow : engine.flows) {
            auto result = flowArchive.emplace(&flow, FlowArchive());
            assert(result.second);
            archiveFlow(&flow, result.first->second);
        }

        //record the information of each traffic light
        for (const auto &intersection : engine.roadnet.getIntersections()) {
            const auto &light = &intersection.getTrafficLight();
            auto result = trafficLightArchive.emplace(light, TrafficLightArchive());
            assert(result.second);
            archiveTrafficLight(light, result.first->second);
        }

    }

    Vehicle *Archive::getNewPointer(const VehiclePool &vehiclePool, const Vehicle *old) {
        if (!old) return nullptr;
        int priority = old->getPriority();
        auto result = vehiclePool.find(priority);
        assert(result != vehiclePool.end());
        return result->second.first;
    }

    void Archive::archiveDrivable(const Drivable *drivable, Archive::DrivableArchive &drivableArchive) {
        for (const auto &vehicle : drivable->getVehicles())
            drivableArchive.vehicles.emplace_back(getNewPointer(vehiclePool, vehicle));
        if (drivable->isLane()) {
            const Lane *lane = static_cast<const Lane *>(drivable);
            for (const auto &vehicle : lane->getWaitingBuffer()) {
                drivableArchive.waitingBuffer.emplace_back(getNewPointer(vehiclePool, vehicle));
            }
            drivableArchive.history = lane->history;
            drivableArchive.historyVehicleNum = lane->historyVehicleNum;
            drivableArchive.historyAverageSpeed = lane->historyAverageSpeed;
        }

    }

    void Archive::archiveFlow(const Flow *flow, Archive::FlowArchive &flowArchive) {
        flowArchive.currentTime = flow->currentTime;
        flowArchive.nowTime = flow->nowTime;
        flowArchive.cnt = flow->cnt;
    }

    void Archive::archiveTrafficLight(const TrafficLight *light, Archive::TrafficLightArchive &trafficLightArchive) {
        trafficLightArchive.curPhaseIndex = light->curPhaseIndex;
        trafficLightArchive.remainDuration = light->remainDuration;
    }


    void Archive::resume(Engine &engine) const{
        engine.step = step;
        engine.activeVehicleCount = activeVehicleCount;
        for (auto &veh : engine.vehiclePool) {
            auto vehicle = veh.second.first;
            delete vehicle;
        }
        engine.vehiclePool = copyVehiclePool(vehiclePool);
        // TODO: reallocate threads when threadNum changes (read from disk)
        engine.rnd = rnd;
        for (auto &threadVeh : engine.threadVehiclePool)
            threadVeh.clear();
        for (const auto &pair : engine.vehiclePool) {
            const auto &vehicle = pair.second.first;
            size_t threadIndex = pair.second.second;
            engine.threadVehiclePool[threadIndex].emplace(vehicle);
        }
        for (auto &drivable : engine.roadnet.getDrivables()) {
            const auto &archive = drivableArchive.find(drivable)->second;
            drivable->vehicles.clear();
            for (const auto &vehicle : archive.vehicles) {
                drivable->vehicles.emplace_back(getNewPointer(engine.vehiclePool, vehicle));
            }

            if (drivable->isLane()) {
                Lane *lane = static_cast<Lane *>(drivable);
                lane->waitingBuffer.clear();
                for (const auto &vehicle : archive.waitingBuffer) {
                    lane->waitingBuffer.emplace_back(getNewPointer(engine.vehiclePool, vehicle));
                }
                lane->history = archive.history;
                lane->historyVehicleNum = archive.historyVehicleNum;
                lane->historyAverageSpeed = archive.historyAverageSpeed;
            }
        }
        for (auto &flow : engine.flows) {
            const auto &archive = flowArchive.find(&flow)->second;
            flow.currentTime = archive.currentTime;
            flow.nowTime = archive.nowTime;
            flow.cnt = archive.cnt;
        }
        for (auto &intersection : engine.roadnet.getIntersections()) {
            auto &light = intersection.getTrafficLight();
            const auto &archive = trafficLightArchive.find(&light)->second;
            light.remainDuration = archive.remainDuration;
            light.curPhaseIndex = archive.curPhaseIndex;
        }

    }

    Archive::VehiclePool Archive::copyVehiclePool(const VehiclePool &src) {
        VehiclePool newPool;
        for (const auto &veh : src) {
            const Vehicle *oldVehicle = veh.second.first;
            Vehicle *newVehicle = new Vehicle(*oldVehicle);
            newPool.emplace(oldVehicle->getPriority(), std::make_pair(newVehicle, veh.second.second));
        }

        // update the vehicle pointers
        for (const auto &veh : newPool) {
            Vehicle *vehicle = veh.second.first;
            vehicle->laneChangeInfo.partner = getNewPointer(newPool, vehicle->laneChangeInfo.partner);
            vehicle->controllerInfo.leader  = getNewPointer(newPool, vehicle->controllerInfo.leader);
            vehicle->controllerInfo.blocker = getNewPointer(newPool, vehicle->controllerInfo.blocker);

            std::shared_ptr<LaneChange> laneChange = vehicle->laneChange;
            laneChange->targetLeader = getNewPointer(newPool, laneChange->targetLeader);
            laneChange->targetFollower = getNewPointer(newPool, laneChange->targetFollower);
            if (laneChange->signalRecv) {
                laneChange->signalRecv = getNewPointer(newPool, laneChange->signalRecv->source)->laneChange->signalSend;
            }
        }
        return newPool;
    }
}