#include "engine/archive.h"
#include "engine/engine.h"

#include <sstream>
#include <string>

namespace CityFlow {

    Archive::Archive(const Engine &engine)
    : step(engine.step), activeVehicleCount(engine.activeVehicleCount), rnd(engine.rnd),
      finishedVehicleCnt(engine.finishedVehicleCnt), cumulativeTravelTime(engine.cumulativeTravelTime) {
        // copy the vehicle Pool
        vehiclePool = copyVehiclePool(engine.vehiclePool);

        // record the information of each drivable object
        for (const auto &drivable : engine.roadnet.getDrivables()) {
            auto result = drivablesArchive.emplace(drivable, DrivableArchive());
            assert(result.second);
            archiveDrivable(drivable, result.first->second);
        }

        // record the information of each flow
        for (const auto &flow : engine.flows) {
            auto result = flowsArchive.emplace(&flow, FlowArchive());
            assert(result.second);
            archiveFlow(&flow, result.first->second);
        }

        //record the information of each traffic light
        for (const auto &intersection : engine.roadnet.getIntersections()) {
            const auto &light = &intersection.getTrafficLight();
            auto result = trafficLightsArchive.emplace(&intersection, TrafficLightArchive());
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
        engine.vehicleMap.clear();
        for (const auto &iter : engine.vehiclePool) {
            Vehicle *vehicle = iter.second.first;
            engine.vehicleMap.emplace(vehicle->getId(), vehicle);
        }
        engine.rnd = rnd;
        for (auto &threadVeh : engine.threadVehiclePool)
            threadVeh.clear();
        for (const auto &pair : engine.vehiclePool) {
            const auto &vehicle = pair.second.first;
            size_t threadIndex = pair.second.second;
            engine.threadVehiclePool[threadIndex].emplace(vehicle);
        }
        for (auto &drivable : engine.roadnet.getDrivables()) {
            const auto &archive = drivablesArchive.find(drivable)->second;
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
            const auto &archive = flowsArchive.find(&flow)->second;
            flow.currentTime = archive.currentTime;
            flow.nowTime = archive.nowTime;
            flow.cnt = archive.cnt;
        }
        for (auto &intersection : engine.roadnet.getIntersections()) {
            auto &light = intersection.getTrafficLight();
            const auto &archive = trafficLightsArchive.find(&intersection)->second;
            light.remainDuration = archive.remainDuration;
            light.curPhaseIndex = archive.curPhaseIndex;
        }
        engine.finishedVehicleCnt = this->finishedVehicleCnt;
        engine.cumulativeTravelTime = this->cumulativeTravelTime;
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

    void Archive::dump(const std::string &fileName) const {
        rapidjson::Document jsonRoot;
        jsonRoot.SetObject();
        auto &allocator = jsonRoot.GetAllocator();

        jsonRoot.AddMember("step", static_cast<unsigned>(this->step), allocator);
        jsonRoot.AddMember("activeVehicleCount", static_cast<unsigned>(this->activeVehicleCount), allocator);

        std::stringstream rndStringStream;
        rndStringStream << this->rnd;
        rapidjson::Value rndValue;
        rndValue.SetString(rndStringStream.str().c_str(), jsonRoot.GetAllocator());
        jsonRoot.AddMember("rnd", rndValue, allocator);
        // Serialize the mt19937 object

        dumpVehicles(jsonRoot);
        dumpDrivables(jsonRoot);
        dumpFlows(jsonRoot);
        dumpTrafficLights(jsonRoot);

        jsonRoot.AddMember("finishedVehicleCnt", finishedVehicleCnt, allocator);
        jsonRoot.AddMember("cumulativeTravelTime", cumulativeTravelTime, allocator);

        writeJsonToFile(fileName, jsonRoot);
    }

    rapidjson::Value Archive::dumpVehicle(const Vehicle &vehicle, rapidjson::Document &jsonRoot) const {
        rapidjson::Value vehicleValue(rapidjson::kObjectType);
        auto &allocator = jsonRoot.GetAllocator();

        vehicleValue.AddMember("priority", vehicle.priority, allocator);
        vehicleValue.AddMember("id",
                rapidjson::Value(vehicle.getId(), allocator).Move(),
                allocator);
        vehicleValue.AddMember("enterTime", vehicle.enterTime, allocator);

        // save vehicleInfo
        vehicleValue.AddMember("speed", vehicle.vehicleInfo.speed, allocator);
        vehicleValue.AddMember("len", vehicle.vehicleInfo.len, allocator);
        vehicleValue.AddMember("width", vehicle.vehicleInfo.width, allocator);
        vehicleValue.AddMember("maxPosAcc", vehicle.vehicleInfo.maxPosAcc, allocator);
        vehicleValue.AddMember("maxNegAcc", vehicle.vehicleInfo.maxNegAcc, allocator);
        vehicleValue.AddMember("usualPosAcc", vehicle.vehicleInfo.usualPosAcc, allocator);
        vehicleValue.AddMember("usualNegAcc", vehicle.vehicleInfo.usualNegAcc, allocator);
        vehicleValue.AddMember("minGap", vehicle.vehicleInfo.minGap, allocator);
        vehicleValue.AddMember("maxSpeed", vehicle.vehicleInfo.maxSpeed, allocator);
        vehicleValue.AddMember("headwayTime", vehicle.vehicleInfo.headwayTime, allocator);
        vehicleValue.AddMember("yieldDistance", vehicle.vehicleInfo.yieldDistance, allocator);
        vehicleValue.AddMember("turnSpeed", vehicle.vehicleInfo.turnSpeed, allocator);


        // save route
        rapidjson::Value routeValue(rapidjson::kArrayType);
        for (const auto &road : vehicle.controllerInfo.router.route) {
            pushBackObjectAsMember(routeValue, road, allocator);
        }
        vehicleValue.AddMember("route", routeValue, allocator);

        // save controllerInfo
        vehicleValue.AddMember("dis", vehicle.controllerInfo.dis, allocator);
        addObjectAsMember(vehicleValue, "drivable", vehicle.controllerInfo.drivable, allocator);
        addObjectAsMember(vehicleValue, "prevDrivable", vehicle.controllerInfo.prevDrivable, allocator);

        vehicleValue.AddMember("approachingIntersectionDistance",
                vehicle.controllerInfo.approachingIntersectionDistance, allocator);
        vehicleValue.AddMember("gap", vehicle.controllerInfo.gap, allocator);
        vehicleValue.AddMember("enterLaneLinkTime",
                static_cast<unsigned>(vehicle.controllerInfo.enterLaneLinkTime), allocator);

        addObjectAsMember(vehicleValue, "leader", vehicle.controllerInfo.leader, allocator);
        addObjectAsMember(vehicleValue, "blocker", vehicle.controllerInfo.blocker, allocator);

        vehicleValue.AddMember("end", vehicle.controllerInfo.end, allocator);
        vehicleValue.AddMember("running", vehicle.controllerInfo.running, allocator);

        // save lane change info
        vehicleValue.AddMember("partnerType", vehicle.laneChangeInfo.partnerType, allocator);
        addObjectAsMember(vehicleValue, "partner", vehicle.laneChangeInfo.partner, allocator);
        vehicleValue.AddMember("offset", vehicle.laneChangeInfo.offset, allocator);
        if (vehicle.laneChange->signalSend) {
            auto &signal = vehicle.laneChange->signalSend;
            vehicleValue.AddMember("laneChangeUrgency", signal->urgency, allocator);
            vehicleValue.AddMember("laneChangeDirection", signal->direction, allocator);
            addObjectAsMember(vehicleValue, "laneChangeTarget", signal->target, allocator);
        }
        if (vehicle.laneChange->signalRecv) {
            auto &signal = vehicle.laneChange->signalRecv;
            addObjectAsMember(vehicleValue, "laneChangeRecv", signal->source, allocator);
        }
        addObjectAsMember(vehicleValue, "laneChangeLeader", vehicle.laneChange->targetLeader, allocator);
        addObjectAsMember(vehicleValue, "laneChangeFollower", vehicle.laneChange->targetFollower, allocator);
        vehicleValue.AddMember("laneChangeWaitingTime", vehicle.laneChange->waitingTime, allocator);
        vehicleValue.AddMember("laneChanging", vehicle.laneChange->changing, allocator);
        vehicleValue.AddMember("laneChangeLastTime", vehicle.laneChange->lastChangeTime, allocator);

        return vehicleValue;
    }

    void Archive::dumpVehicles(rapidjson::Document &jsonRoot) const {
        rapidjson::Value vehicleArray(rapidjson::kArrayType);
        auto &allocator = jsonRoot.GetAllocator();
        for (const auto &iter : this->vehiclePool) {
            const auto &vehicle = iter.second.first;
            assert(vehicle);
            rapidjson::Value vehicleValue = dumpVehicle(*vehicle, jsonRoot);
            vehicleArray.PushBack(vehicleValue, allocator);
        }
        jsonRoot.AddMember("vehicles", vehicleArray, allocator);
    }

    void Archive::dumpDrivables(rapidjson::Document &jsonRoot) const {
        rapidjson::Value drivablesValue(rapidjson::kObjectType);
        auto &allocator = jsonRoot.GetAllocator();
        for (const auto &iter : drivablesArchive) {
            rapidjson::Value drivableValue(rapidjson::kObjectType);
            const Drivable *drivable = iter.first;
            const DrivableArchive &drivableArchive = iter.second;

            // save vehicles
            rapidjson::Value vehicleList(rapidjson::kArrayType);
            for (const auto &vehicle : drivableArchive.vehicles) {
                pushBackObjectAsMember(vehicleList, vehicle, allocator);
            }
            drivableValue.AddMember("vehicles", vehicleList, allocator);

            if (drivable->isLane()) {
                // save waiting buffer
                rapidjson::Value waitingBuffer(rapidjson::kArrayType);
                for (const auto &vehicle : drivableArchive.waitingBuffer) {
                    pushBackObjectAsMember(waitingBuffer, vehicle, allocator);
                }
                drivableValue.AddMember("waitingBuffer", waitingBuffer, allocator);

                //save history
                rapidjson::Value historyValue(rapidjson::kArrayType);
                for (const auto &record : drivableArchive.history) {
                    historyValue.PushBack(record.vehicleNum, allocator);
                    historyValue.PushBack(record.averageSpeed, allocator);
                }
                drivableValue.AddMember("history", historyValue, allocator);
                drivableValue.AddMember("historyVehicleNum", drivableArchive.historyVehicleNum, allocator);
                drivableValue.AddMember("historyAverageSpeed", drivableArchive.historyAverageSpeed, allocator);
            }

            drivablesValue.AddMember(
                    rapidjson::Value(drivable->getId(), allocator).Move(),
                    drivableValue,
                    allocator
                    );
        }
        jsonRoot.AddMember("drivables", drivablesValue, allocator);
    }

    void Archive::dumpFlows(rapidjson::Document &jsonRoot) const {
        rapidjson::Value flowsValue(rapidjson::kObjectType);
        auto &allocator = jsonRoot.GetAllocator();
        for (const auto &iter : flowsArchive) {
            const auto &flow = iter.first;
            const auto &flowArchive = iter.second;

            rapidjson::Value flowValue(rapidjson::kObjectType);
            flowValue.AddMember("nowTime", flowArchive.nowTime, allocator);
            flowValue.AddMember("currentTime", flowArchive.currentTime, allocator);
            flowValue.AddMember("cnt", static_cast<unsigned>(flowArchive.cnt), allocator);

            flowsValue.AddMember(
                    rapidjson::Value(flow->getId(), allocator).Move(),
                    flowValue,
                    allocator);
        }
        jsonRoot.AddMember("flows", flowsValue, allocator);
    }

    void Archive::dumpTrafficLights(rapidjson::Document &jsonRoot) const {
        rapidjson::Value trafficLightsValue(rapidjson::kObjectType);
        auto &allocator = jsonRoot.GetAllocator();
        for (const auto &iter : trafficLightsArchive) {
            const auto &intersection = iter.first;
            const auto &trafficLightArchive = iter.second;

            rapidjson::Value trafficLightValue(rapidjson::kObjectType);
            trafficLightValue.AddMember("remainDuration", trafficLightArchive.remainDuration, allocator);
            trafficLightValue.AddMember("curPhaseIndex", static_cast<unsigned>(trafficLightArchive.curPhaseIndex), allocator);

            trafficLightsValue.AddMember(
                    rapidjson::Value(intersection->getId(), allocator).Move(),
                    trafficLightValue,
                    allocator);
        }
        jsonRoot.AddMember("trafficLights", trafficLightsValue, allocator);
    }

    Archive::Archive(Engine &engine, const std::string &filename) {
        // read from file
        rapidjson::Document jsonRoot;
        readJsonFromFile(filename, jsonRoot);

        std::mt19937 rndTemp;
        // restore random seed
        std::string rndString = getJsonMember<const char *>("rnd", jsonRoot);
        std::istringstream istr(rndString);
        istr >> rnd;

        // restore engine info
        step = getJsonMember<unsigned>("step", jsonRoot);
        activeVehicleCount = getJsonMember<unsigned>("activeVehicleCount", jsonRoot);

        // restore vehiclePool
        auto &vehiclesValue = getJsonMemberArray("vehicles", jsonRoot);
        std::map<const std::string, Vehicle *> vehicleDict;
        for (auto &vehicleValue : vehiclesValue.GetArray()) {
            VehicleInfo vehicleInfo;
            vehicleInfo.speed = getJsonMember<double>("speed", vehicleValue);
            vehicleInfo.len = getJsonMember<double>("len", vehicleValue);
            vehicleInfo.width = getJsonMember<double>("width", vehicleValue);
            vehicleInfo.maxPosAcc = getJsonMember<double>("maxPosAcc", vehicleValue);
            vehicleInfo.maxNegAcc = getJsonMember<double>("maxNegAcc", vehicleValue);
            vehicleInfo.usualPosAcc = getJsonMember<double>("usualPosAcc", vehicleValue);
            vehicleInfo.usualNegAcc = getJsonMember<double>("usualNegAcc", vehicleValue);
            vehicleInfo.minGap = getJsonMember<double>("minGap", vehicleValue);
            vehicleInfo.maxSpeed = getJsonMember<double>("maxSpeed", vehicleValue);
            vehicleInfo.headwayTime = getJsonMember<double>("headwayTime", vehicleValue);
            vehicleInfo.yieldDistance = getJsonMember<double>("yieldDistance", vehicleValue);
            vehicleInfo.turnSpeed = getJsonMember<double>("turnSpeed", vehicleValue);

            // Rebuild Route
            std::vector<Road *> route;
            auto &routeValue = getJsonMemberArray("route", vehicleValue);
            for (const auto &roadValue : routeValue.GetArray()) {
                const auto &roadId = roadValue.GetString();
                route.emplace_back(engine.roadnet.getRoadById(roadId));
            }
            vehicleInfo.route = std::make_shared<Route>(route);

            Vehicle *vehicle = new Vehicle(vehicleInfo,
                    getJsonMember<const char *>("id", vehicleValue), &engine);

            auto enterTime = getJsonMember<double>("enterTime", vehicleValue);
            vehicle->enterTime = enterTime;

            auto priority = getJsonMember<int>("priority", vehicleValue);
            vehicle->priority = priority;
            vehiclePool.emplace(priority, std::make_pair(vehicle, rndTemp() % engine.threadNum));
            vehicleDict.emplace(vehicle->getId(), vehicle);

            auto &controllerInfo = vehicle->controllerInfo;
            controllerInfo.dis = getJsonMember<double>("dis", vehicleValue);
            controllerInfo.approachingIntersectionDistance =
                    getJsonMember<double>("approachingIntersectionDistance", vehicleValue);
            controllerInfo.gap = getJsonMember<double>("gap", vehicleValue);
            controllerInfo.enterLaneLinkTime = getJsonMember<int>("enterLaneLinkTime", vehicleValue);
            controllerInfo.end = getJsonMember<bool>("end", vehicleValue);
            controllerInfo.running = getJsonMember<bool>("running", vehicleValue);

            auto &laneChangeInfo = vehicle->laneChangeInfo;
            laneChangeInfo.partnerType = getJsonMember<int>("partnerType", vehicleValue);
            laneChangeInfo.offset = getJsonMember<double>("offset", vehicleValue);

            // Construct the laneChange Object
            vehicle->laneChange = std::make_shared<SimpleLaneChange>(vehicle);
            auto &laneChange = vehicle->laneChange;
            rapidjson::Value::ConstMemberIterator sendItr = vehicleValue.FindMember("laneChangeUrgency");
            if (sendItr != vehicleValue.MemberEnd()) {
                auto signal = std::make_shared<LaneChange::Signal>();
                signal->source = vehicle;
                signal->urgency = sendItr->value.GetInt();
                signal->direction = getJsonMember<int>("laneChangeDirection", vehicleValue);
                laneChange->signalSend = signal;
            }
            laneChange->waitingTime = getJsonMember<double>("laneChangeWaitingTime", vehicleValue);
            laneChange->changing = getJsonMember<bool>("laneChanging", vehicleValue);
            laneChange->lastChangeTime = getJsonMember<double>("laneChangeLastTime", vehicleValue);
        }

        // restore pointer relations
        for (auto &vehicleValue : vehiclesValue.GetArray()) {
            std::string vehicleId = getJsonMember<const char*>("id", vehicleValue);
            Vehicle *vehicle = vehicleDict[vehicleId];
            const char *drivableId = getJsonMember<const char *>("drivable", vehicleValue, nullptr);
            assert(drivableId);
            vehicle->controllerInfo.drivable = engine.roadnet.getDrivableById(std::string(drivableId));
            assert(vehicle->controllerInfo.drivable);
            const char *prevDrivableId = getJsonMember<const char *>("prevDrivable", vehicleValue, nullptr);
            if (prevDrivableId) {
                vehicle->controllerInfo.prevDrivable = engine.roadnet.getDrivableById(std::string(prevDrivableId));
            }
            const char *leaderId = getJsonMember<const char *>("leader", vehicleValue, nullptr);
            if (leaderId) {
                vehicle->controllerInfo.leader = vehicleDict[leaderId];
            }
            const char *blockerId = getJsonMember<const char *>("blocker", vehicleValue, nullptr);
            if (blockerId) {
                vehicle->controllerInfo.blocker = vehicleDict[blockerId];
            }
            const char *partnerId = getJsonMember<const char *>("partner", vehicleValue, nullptr);
            if (partnerId) {
                vehicle->laneChangeInfo.partner = vehicleDict[partnerId];
            }

            if (vehicle->laneChange->signalSend) {
                auto &signal = vehicle->laneChange->signalSend;
                const char *targetId = getJsonMember<const char *>("laneChangeTarget", vehicleValue, nullptr);
                assert(targetId);
                Drivable *target = engine.roadnet.getDrivableById(targetId);
                assert(target->isLane());
                signal->target = static_cast<Lane *>(target);
            }

            rapidjson::Value::ConstMemberIterator signalRecvIter = vehicleValue.FindMember("laneChangeRecv");
            if (signalRecvIter != vehicleValue.MemberEnd()) {
                std::string sourceId = signalRecvIter->value.GetString();
                vehicle->laneChange->signalRecv = vehicleDict[sourceId]->laneChange->signalSend;
            }

            const char *laneChangeLeaderId = getJsonMember<const char*>("laneChangeLeader", vehicleValue, nullptr);
            if (laneChangeLeaderId) {
                vehicle->laneChange->targetLeader = vehicleDict[laneChangeLeaderId];
            }

            const char *laneChangeFollowerId = getJsonMember<const char*>("laneChangeFollower", vehicleValue, nullptr);
            if (laneChangeFollowerId) {
                vehicle->laneChange->targetFollower = vehicleDict[laneChangeFollowerId];
            }
        }

        // Ensure partners in the same thread
        for (auto &iter : vehiclePool) {
            auto &vehicle = iter.second.first;
            if (!vehicle->isReal()) {
                assert(vehicle->hasPartner());
                auto partnerPriority = vehicle->getPartner()->getPriority();
                iter.second.second = vehiclePool[partnerPriority].second;
            }
        }

        // restore drivables
        auto &drivablesValue = getJsonMemberObject("drivables", jsonRoot);
        for (auto &drivable : engine.roadnet.getDrivables()) {
            auto &drivableValue = getJsonMemberObject(drivable->getId(), drivablesValue);
            auto result = drivablesArchive.emplace(drivable, DrivableArchive());
            assert(result.second);
            auto &drivableArchive = result.first->second;

            auto &vehiclesValue = getJsonMemberArray("vehicles", drivableValue);
            for (auto &vehicleValue : vehiclesValue.GetArray()) {
                std::string vehicleId = vehicleValue.GetString();
                drivableArchive.vehicles.emplace_back(vehicleDict[vehicleId]);
            }

            if (drivable->isLane()) {
                auto &waitingBufferValue = getJsonMemberArray("waitingBuffer", drivableValue);
                for (auto &vehicleValue : waitingBufferValue.GetArray()) {
                    std::string vehicleId = vehicleValue.GetString();
                    drivableArchive.waitingBuffer.emplace_back(vehicleDict[vehicleId]);
                }
                auto &historyValue = getJsonMemberArray("history", drivableValue);
                int cnt = 0;
                int vehicleNum = 0;
                for (auto &recordValue : historyValue.GetArray()) {
                    if (cnt % 2) {
                        double averageSpeed = recordValue.GetDouble();
                        drivableArchive.history.emplace_back(vehicleNum, averageSpeed);
                    } else {
                        vehicleNum = recordValue.GetInt();
                    }
                    ++cnt;
                }
                drivableArchive.historyAverageSpeed = getJsonMember<double>("historyAverageSpeed", drivableValue);
                drivableArchive.historyVehicleNum = getJsonMember<int>("historyVehicleNum", drivableValue);
            }
        }

        // restore flows
        auto &flowsValue = getJsonMemberObject("flows", jsonRoot);
        for (auto &flow : engine.flows) {
            auto &flowValue = getJsonMemberObject(flow.getId(), flowsValue);
            auto result = flowsArchive.emplace(&flow, FlowArchive());
            assert(result.second);
            auto &flowArchive = result.first->second;
            flowArchive.nowTime = getJsonMember<double>("nowTime", flowValue);
            flowArchive.currentTime = getJsonMember<double>("currentTime", flowValue);
            flowArchive.cnt = getJsonMember<int>("cnt", flowValue);
        }

        // restore trafficlights
        auto &trafficLightsValue = getJsonMemberObject("trafficLights", jsonRoot);
        for (auto &intersection : engine.roadnet.getIntersections()) {
            auto &trafficLightValue = getJsonMemberObject(intersection.getId(), trafficLightsValue);
            auto result = trafficLightsArchive.emplace(&intersection, TrafficLightArchive());
            assert(result.second);
            auto &trafficLightArchive = result.first->second;
            trafficLightArchive.remainDuration = getJsonMember<double>("remainDuration", trafficLightValue);
            trafficLightArchive.curPhaseIndex = getJsonMember<int>("curPhaseIndex", trafficLightValue);
        }

        finishedVehicleCnt = getJsonMember<int>("finishedVehicleCnt", jsonRoot);
        cumulativeTravelTime = getJsonMember<double>("cumulativeTravelTime", jsonRoot);
    }


}