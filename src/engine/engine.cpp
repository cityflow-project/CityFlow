#include "engine/engine.h"
#include "utility/utility.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <iostream>
#include <memory>

#include <ctime>
namespace CityFlow {

    Engine::Engine(const std::string &configFile, int threadNum) : threadNum(threadNum), startBarrier(threadNum + 1),
                                                                   endBarrier(threadNum + 1) {
        for (int i = 0; i < threadNum; i++) {
            threadVehiclePool.emplace_back();
            threadRoadPool.emplace_back();
            threadIntersectionPool.emplace_back();
            threadDrivablePool.emplace_back();
        }
        bool success = loadConfig(configFile);
        if (!success) {
            std::cerr << "load config failed!" << std::endl;
        }

        for (int i = 0; i < threadNum; i++) {
            threadPool.emplace_back(&Engine::threadController, this,
                                    std::ref(threadVehiclePool[i]),
                                    std::ref(threadRoadPool[i]),
                                    std::ref(threadIntersectionPool[i]),
                                    std::ref(threadDrivablePool[i]));
        }

    }


    bool Engine::loadConfig(const std::string &configFile) {
        rapidjson::Document document;
        if (!readJsonFromFile(configFile, document)) {
            std::cerr << "cannot open config file!" << std::endl;
            return false;
        }

        if (!document.IsObject()) {
            std::cerr << "wrong format of config file" << std::endl;
            return false;
        }

        try {
            interval = getJsonMember<double>("interval", document);
            warnings = false;
            rlTrafficLight = getJsonMember<bool>("rlTrafficLight", document);
            laneChange = getJsonMember<bool>("laneChange", document, false);
            seed = getJsonMember<int>("seed", document);
            rnd.seed(seed);
            dir = getJsonMember<const char*>("dir", document);
            std::string roadnetFile = getJsonMember<const char*>("roadnetFile", document);
            std::string flowFile = getJsonMember<const char*>("flowFile", document);

            if (!loadRoadNet(dir + roadnetFile)) {
                std::cerr << "loading roadnet file error!" << std::endl;
                return false;
            }

            if (!loadFlow(dir + flowFile)) {
                std::cerr << "loading flow file error!" << std::endl;
                return false;
            }

            if (warnings) checkWarning();
            saveReplayInConfig = saveReplay = getJsonMember<bool>("saveReplay", document);

            if (saveReplay) {
                std::string roadnetLogFile = getJsonMember<const char*>("roadnetLogFile", document);
                std::string replayLogFile = getJsonMember<const char*>("replayLogFile", document);
                setLogFile(dir + roadnetLogFile, dir + replayLogFile);
            }
        } catch (const JsonFormatError &e) {
            std::cerr << e.what() << std::endl;
            return false;
        }
        stepLog = "";
        return true;
    }

    bool Engine::loadRoadNet(const std::string &jsonFile) {
        bool ans = roadnet.loadFromJson(jsonFile);
        int cnt = 0;
        for (Road &road : roadnet.getRoads()) {
            threadRoadPool[cnt].push_back(&road);
            cnt = (cnt + 1) % threadNum;
        }
        for (Intersection &intersection : roadnet.getIntersections()) {
            threadIntersectionPool[cnt].push_back(&intersection);
            cnt = (cnt + 1) % threadNum;
        }
        for (Drivable *drivable : roadnet.getDrivables()) {
            threadDrivablePool[cnt].push_back(drivable);
            cnt = (cnt + 1) % threadNum;
        }
        jsonRoot.SetObject();
        jsonRoot.AddMember("static", roadnet.convertToJson(jsonRoot.GetAllocator()), jsonRoot.GetAllocator());
        return ans;
    }

    bool Engine::loadFlow(const std::string &jsonFilename) {
        rapidjson::Document root;
        if (!readJsonFromFile(jsonFilename, root)) {
            std::cerr << "cannot open flow file!" << std::endl;
            return false;
        }
        std::list<std::string> path;
        try {
            if (!root.IsArray())
                throw JsonTypeError("flow file", "array");
            for (rapidjson::SizeType i = 0; i < root.Size(); i++) {
                path.emplace_back("flow[" + std::to_string(i) + "]");
                rapidjson::Value &flow = root[i];
                std::vector<Road *> roads;
                const auto &routes = getJsonMemberArray("route", flow);
                roads.reserve(routes.Size());
                for (auto &route: routes.GetArray()) {
                    path.emplace_back("route[" + std::to_string(roads.size()) + "]");
                    if (!route.IsString())
                        throw JsonTypeError("route", "string");
                    std::string roadName = route.GetString();
                    auto road = roadnet.getRoadById(roadName);
                    if (!road)
                        throw JsonFormatError("No such road: " + roadName);
                    roads.push_back(road);
                    path.pop_back();
                }
                auto route = std::make_shared<const Route>(roads);

                VehicleInfo vehicleInfo;
                const auto &vehicle = getJsonMemberObject("vehicle", flow);
                vehicleInfo.len = getJsonMember<double>("length", vehicle);
                vehicleInfo.width = getJsonMember<double>("width", vehicle);
                vehicleInfo.maxPosAcc = getJsonMember<double>("maxPosAcc", vehicle);
                vehicleInfo.maxNegAcc = getJsonMember<double>("maxNegAcc", vehicle);
                vehicleInfo.usualPosAcc = getJsonMember<double>("usualPosAcc", vehicle);
                vehicleInfo.usualNegAcc = getJsonMember<double>("usualNegAcc", vehicle);
                vehicleInfo.minGap = getJsonMember<double>("minGap", vehicle);
                vehicleInfo.maxSpeed = getJsonMember<double>("maxSpeed", vehicle);
                vehicleInfo.headwayTime = getJsonMember<double>("headwayTime", vehicle);
                vehicleInfo.route = route;
                int startTime = getJsonMember<int>("startTime", flow, 0);
                int endTime = getJsonMember<int>("endTime", flow, -1);
                Flow newFlow(vehicleInfo, getJsonMember<double>("interval", flow), this, startTime, endTime,
                             "flow_" + std::to_string(i));
                flows.push_back(newFlow);
                path.pop_back();
            }
            assert(path.empty());
        } catch (const JsonFormatError &e) {
            std::cerr << "Error occurred when reading flow file" << std::endl;
            for (const auto &node : path) {
                std::cerr << "/" << node;
            }
            std::cerr << " " << e.what() << std::endl;
            return false;
        }
        return true;
    }

    bool Engine::checkWarning() {
        bool result = true;

        if (interval < 0.2 || interval > 1.5) {
            std::cerr << "Deprecated time interval, recommended interval between 0.2 and 1.5" << std::endl;
            result = false;
        }

        for (Lane *lane : roadnet.getLanes()) {
            if (lane->getLength() < 50) {
                std::cerr << "Deprecated road length, recommended road length at least 50 meters" << std::endl;
                result = false;
            }
            if (lane->getMaxSpeed() > 30) {
                std::cerr << "Deprecated road max speed, recommended max speed at most 30 meters/s" << std::endl;
                result = false;
            }
        }

        return result;
    }

    void Engine::vehicleControl(Vehicle &vehicle, std::vector<std::pair<Vehicle *, double>> &buffer) {
        double nextSpeed;
        if (vehicle.hasSetSpeed())
            nextSpeed = vehicle.getBufferSpeed();
        else
            nextSpeed = vehicle.getNextSpeed(interval).speed;

        if (laneChange) {
            Vehicle * partner = vehicle.getPartner();
            if (partner != nullptr && !partner->hasSetSpeed()){
                double partnerSpeed = partner->getNextSpeed(interval).speed;
                nextSpeed = min2double(nextSpeed, partnerSpeed);
                partner->setSpeed(nextSpeed);

                if (partner->hasSetEnd())
                    vehicle.setEnd(true);
            }
        }

        if (vehicle.getPartner()) {
            assert(vehicle.getDistance() == vehicle.getPartner()->getDistance());

        }

        double deltaDis, speed = vehicle.getSpeed();

        if (nextSpeed < 0) {
            deltaDis = 0.5 * speed * speed / vehicle.getMaxNegAcc();
            nextSpeed = 0;
        } else {
            deltaDis = (speed + nextSpeed) * interval / 2;
        }
        vehicle.setSpeed(nextSpeed);
        vehicle.setDeltaDistance(deltaDis);

        if (laneChange) {
            if (!vehicle.isReal() && vehicle.getChangedDrivable() != nullptr) {
                vehicle.abortLaneChange();
            }

            if (vehicle.isChanging()) {
                assert(vehicle.isReal());

                int dir = vehicle.getLaneChangeDirection();
                double newOffset = fabs(vehicle.getOffset() + max2double(0.2 * nextSpeed, 1) * interval * dir);
                newOffset = min2double(newOffset, vehicle.getMaxOffset());
                vehicle.setOffset(newOffset * dir);

                if (newOffset >= vehicle.getMaxOffset()) {
                    std::lock_guard<std::mutex> guard(lock);
                    vehicleMap.erase(vehicle.getPartner()->getId());
                    vehicleMap[vehicle.getId()] = vehicle.getPartner();
                    vehicle.finishChanging();
                }

            }
        }


        if (!vehicle.hasSetEnd() && vehicle.hasSetDrivable()) {
            buffer.emplace_back(&vehicle, vehicle.getBufferDis());
        }

    }

    void Engine::threadController(std::set<Vehicle *> &vehicles, 
                                  std::vector<Road *> &roads,
                                  std::vector<Intersection *> &intersections,
                                  std::vector<Drivable *> &drivables) {
        while (!finished) {
            threadPlanRoute(roads);
            if (laneChange) {
                threadInitSegments(roads);
                threadPlanLaneChange(vehicles);
                threadUpdateLeaderAndGap(drivables);
            }
            threadNotifyCross(intersections);
            threadGetAction(vehicles);
            threadUpdateLocation(drivables);
            threadUpdateAction(vehicles);
            threadUpdateLeaderAndGap(drivables);
        }
    }

    void Engine::threadPlanRoute(const std::vector<Road *> &roads) {
        startBarrier.wait();
        for (auto &road : roads) {
            for (auto &vehicle : road->getPlanRouteBuffer()) {
                vehicle->updateRoute();
            }
        }
        endBarrier.wait();
    }

    void Engine::threadUpdateLocation(const std::vector<Drivable *> &drivables) {
        startBarrier.wait();
        for (Drivable *drivable : drivables) {
            auto &vehicles   = drivable->getVehicles();
            auto vehicleItr = vehicles.begin();
            while (vehicleItr != vehicles.end()) {
                Vehicle *vehicle = *vehicleItr;

                if ((vehicle->getChangedDrivable()) != nullptr || vehicle->hasSetEnd()) {
                    vehicleItr = vehicles.erase(vehicleItr);
                }else{
                    vehicleItr++;
                }

                if (vehicle->hasSetEnd()) {
                    std::lock_guard<std::mutex> guard(lock);
                    vehicleRemoveBuffer.insert(vehicle);
                    if (!vehicle->getLaneChange()->hasFinished()) {
                        vehicleMap.erase(vehicle->getId());
                        finishedVehicleCnt += 1;
                        cumulativeTravelTime += getCurrentTime() - vehicle->getEnterTime();
                    }
                    auto iter = vehiclePool.find(vehicle->getPriority());
                    threadVehiclePool[iter->second.second].erase(vehicle);
//                    assert(vehicle->getPartner() == nullptr);
                    delete vehicle;
                    vehiclePool.erase(iter);
                    activeVehicleCount--;
                }

            }
        }
        endBarrier.wait();
    }

    void Engine::threadNotifyCross(const std::vector<Intersection *> &intersections) {
        //TODO: iterator for laneLink
        startBarrier.wait();
        for (Intersection *intersection : intersections)
            for (Cross &cross : intersection->getCrosses())
                cross.clearNotify();

        for (Intersection *intersection : intersections)
            for (LaneLink *laneLink : intersection->getLaneLinks()) {
                // XXX: no cross in laneLink?
                const auto &crosses = laneLink->getCrosses();
                auto rIter = crosses.rbegin();

                // first check the vehicle on the end lane
                Vehicle *vehicle = laneLink->getEndLane()->getLastVehicle();
                if (vehicle && static_cast<LaneLink *>(vehicle->getPrevDrivable()) == laneLink) {
                    double vehDistance = vehicle->getDistance() - vehicle->getLen();
                    while (rIter != crosses.rend()) {
                        double crossDistance = laneLink->getLength() - (*rIter)->getDistanceByLane(laneLink);
                        if (crossDistance + vehDistance < (*rIter)->getLeaveDistance()) {
                            (*rIter)->notify(laneLink, vehicle, -(vehicle->getDistance() + crossDistance));
                            ++rIter;
                        } else break;
                    }
                }

                // check each vehicle on laneLink
                for (Vehicle *linkVehicle : laneLink->getVehicles()) {
                    double vehDistance = linkVehicle->getDistance();

                    while (rIter != crosses.rend()) {
                        double crossDistance = (*rIter)->getDistanceByLane(laneLink);
                        if (vehDistance > crossDistance) {
                            if (vehDistance - crossDistance - linkVehicle->getLen() <=
                                (*rIter)->getLeaveDistance()) {
                                (*rIter)->notify(laneLink, linkVehicle, crossDistance - vehDistance);
                            } else break;
                        } else {
                            (*rIter)->notify(laneLink, linkVehicle, crossDistance - vehDistance);
                        }
                        ++rIter;
                    }
                }

                // check vehicle on the incoming lane
                vehicle = laneLink->getStartLane()->getFirstVehicle();
                if (vehicle && static_cast<LaneLink *>(vehicle->getNextDrivable()) == laneLink && laneLink->isAvailable()) {
                    double vehDistance = laneLink->getStartLane()->getLength() - vehicle->getDistance();
                    while (rIter != crosses.rend()) {
                        (*rIter)->notify(laneLink, vehicle, vehDistance + (*rIter)->getDistanceByLane(laneLink));
                        ++rIter;
                    }
                }
            }
        endBarrier.wait();
    }

    void Engine::threadPlanLaneChange(const std::set<CityFlow::Vehicle *> &vehicles) {
        startBarrier.wait();
        std::vector<CityFlow::Vehicle *> buffer;

        for (auto vehicle : vehicles)
            if (vehicle->isRunning() && vehicle->isReal()) {
                vehicle->makeLaneChangeSignal(interval);
                if (vehicle->planLaneChange()){
                    buffer.emplace_back(vehicle);
                }
            }
        {
            std::lock_guard<std::mutex> guard(lock);
            laneChangeNotifyBuffer.insert(laneChangeNotifyBuffer.end(), buffer.begin(), buffer.end());
        }
        endBarrier.wait();
    }

    void Engine::threadInitSegments(const std::vector<Road *> &roads) {
        startBarrier.wait();
        for (Road *road : roads)
            for (Lane &lane : road->getLanes()) {
                lane.initSegments();
            }
        endBarrier.wait();
    }


    void Engine::threadGetAction(std::set<Vehicle *> &vehicles) {
        startBarrier.wait();
        std::vector<std::pair<Vehicle *, double>> buffer;
        for (auto vehicle: vehicles)
            if (vehicle->isRunning()) 
                vehicleControl(*vehicle, buffer);
        {
            std::lock_guard<std::mutex> guard(lock);
            pushBuffer.insert(pushBuffer.end(), buffer.begin(), buffer.end());
        }
        endBarrier.wait();
    }

    void Engine::threadUpdateAction(std::set<Vehicle *> &vehicles) {
        startBarrier.wait();
        for (auto vehicle: vehicles)
            if (vehicle->isRunning()) {
                if (vehicleRemoveBuffer.count(vehicle->getBufferBlocker())){
                    vehicle->setBlocker(nullptr);
                }

                vehicle->update();
                vehicle->clearSignal();
            }
        endBarrier.wait();
    }

    void Engine::threadUpdateLeaderAndGap(const std::vector<Drivable *> &drivables) {
        startBarrier.wait();
        for (Drivable *drivable : drivables) {
            Vehicle *leader = nullptr;
            for (Vehicle *vehicle : drivable->getVehicles()) {
                vehicle->updateLeaderAndGap(leader);
                leader = vehicle;
            }
            if (drivable->isLane()){
                static_cast<Lane *>(drivable)->updateHistory();
            }
        }
        endBarrier.wait();
    }

    void Engine::planLaneChange() {
        startBarrier.wait();
        endBarrier.wait();
        scheduleLaneChange();
    }

    void Engine::planRoute() {
        startBarrier.wait();
        endBarrier.wait();
        for (auto &road : roadnet.getRoads()) {
            for (auto &vehicle : road.getPlanRouteBuffer())
                if (vehicle->isRouteValid()) {
                    vehicle->setFirstDrivable();
                    vehicle->getCurLane()->pushWaitingVehicle(vehicle);
                }else {
                    Flow *flow = vehicle->getFlow();
                    if (flow) flow->setValid(false);

                    //remove this vehicle
                    auto iter = vehiclePool.find(vehicle->getPriority());
                    threadVehiclePool[iter->second.second].erase(vehicle);
                    delete vehicle;
                    vehiclePool.erase(iter);
                }
            road.clearPlanRouteBuffer();
        }
    }

    void Engine::getAction() {
        startBarrier.wait();
        endBarrier.wait();
    }

    void Engine::updateLocation() {
        startBarrier.wait();
        endBarrier.wait();
        std::sort(pushBuffer.begin(), pushBuffer.end(), vehicleCmp);
        for (auto &vehiclePair : pushBuffer) {
            Vehicle *vehicle = vehiclePair.first;
            Drivable *drivable = vehicle->getChangedDrivable();
            if (drivable != nullptr) {
                drivable->pushVehicle(vehicle);
                if (drivable->isLaneLink()) {
                    vehicle->setEnterLaneLinkTime(step);
                } else {
                    vehicle->setEnterLaneLinkTime(std::numeric_limits<int>::max());
                }
            }
        }
        pushBuffer.clear();
    }

    void Engine::updateAction() {
        startBarrier.wait();
        endBarrier.wait();
        vehicleRemoveBuffer.clear();
    }

    void Engine::handleWaiting() {
        for (Lane *lane : roadnet.getLanes()) {
            auto &buffer = lane->getWaitingBuffer();
            if (buffer.empty()) continue;
            auto &vehicle = buffer.front();
            if (lane->available(vehicle)) {
                vehicle->setRunning(true);
                activeVehicleCount += 1;
                Vehicle * tail = lane->getLastVehicle();
                lane->pushVehicle(vehicle);
                vehicle->updateLeaderAndGap(tail);
                buffer.pop_front();
            }
        }
    }

    void Engine::updateLog() {
        std::string result;
        for (const Vehicle* vehicle: getRunningVehicles()) {
            Point pos = vehicle->getPoint();
            Point dir = vehicle->getCurDrivable()->getDirectionByDistance(vehicle->getDistance());

            int lc = vehicle->lastLaneChangeDirection();
            result.append(
                    double2string(pos.x) + " " + double2string(pos.y) + " " + double2string(atan2(dir.y, dir.x)) + " "
                            + vehicle->getId() + " " + std::to_string(lc) + " " + double2string(vehicle->getLen()) + " "
                            + double2string(vehicle->getWidth()) + ",");
        }
        result.append(";");

        for (const Road &road : roadnet.getRoads()) {
            if (road.getEndIntersection().isVirtualIntersection())
                continue;
            result.append(road.getId());
            for (const Lane &lane : road.getLanes()) {
                if (lane.getEndIntersection()->isImplicitIntersection()){
                    result.append(" i");
                    continue;
                }

                bool can_go = true;
                for (LaneLink *laneLink : lane.getLaneLinks()) {
                    if (!laneLink->isAvailable()) {
                        can_go = false;
                        break;
                    }
                }
                result.append(can_go ? " g" : " r");
            }
            result.append(",");
        }
        logOut << result << std::endl;
    }

    void Engine::updateLeaderAndGap() {
        startBarrier.wait();
        endBarrier.wait();
    }

    void Engine::notifyCross() {
        startBarrier.wait();
        endBarrier.wait();
    }

    void Engine::nextStep() {
        for (auto &flow : flows)
            flow.nextStep(interval);
        planRoute();
        handleWaiting();
        if (laneChange) {
            initSegments();
            planLaneChange();
            updateLeaderAndGap();
        }
        notifyCross();

        getAction();
        updateLocation();
        updateAction();
        updateLeaderAndGap();

        if (!rlTrafficLight) {
            std::vector<Intersection> &intersections = roadnet.getIntersections();
            for (auto &intersection : intersections)
                intersection.getTrafficLight().passTime(interval);
        }

        if (saveReplay) {
            updateLog();
        }

        step += 1;
    }

    void Engine::initSegments() {
        startBarrier.wait();
        endBarrier.wait();
    }

    bool Engine::checkPriority(int priority) {
        return vehiclePool.find(priority) != vehiclePool.end();
    }

    void Engine::pushVehicle(Vehicle *const vehicle, bool pushToDrivable) {
        size_t threadIndex = rnd() % threadNum;
        vehiclePool.emplace(vehicle->getPriority(), std::make_pair(vehicle, threadIndex));
        vehicleMap.emplace(vehicle->getId(), vehicle);
        threadVehiclePool[threadIndex].insert(vehicle);

        if (pushToDrivable)
            ((Lane *) vehicle->getCurDrivable())->pushWaitingVehicle(vehicle);
    }

    size_t Engine::getVehicleCount() const {
        return activeVehicleCount;
    }

    std::vector<std::string> Engine::getVehicles(bool includeWaiting) const {
        std::vector<std::string> ret;
        ret.reserve(activeVehicleCount);
        for (const Vehicle* vehicle : getRunningVehicles(includeWaiting)) {
            ret.emplace_back(vehicle->getId());
        }
        return ret;
    }

    std::map<std::string, int> Engine::getLaneVehicleCount() const {
        std::map<std::string, int> ret;
        for (const Lane *lane : roadnet.getLanes()) {
            ret.emplace(lane->getId(), lane->getVehicleCount());
        }
        return ret;
    }

    std::map<std::string, int> Engine::getLaneWaitingVehicleCount() const {
        std::map<std::string, int> ret;
        for (const Lane *lane : roadnet.getLanes()) {
            int cnt = 0;
            for (Vehicle *vehicle : lane->getVehicles()) {
                if (vehicle->getSpeed() < 0.1) { //TODO: better waiting critera
                    cnt += 1;
                }
            }
            ret.emplace(lane->getId(), cnt);
        }
        return ret;
    }

    std::map<std::string, std::vector<std::string>> Engine::getLaneVehicles() {
        std::map<std::string, std::vector<std::string>> ret;
        for (const Lane *lane : roadnet.getLanes()) {
            std::vector<std::string> vehicles;
            for (Vehicle *vehicle : lane->getVehicles()) {
                vehicles.push_back(vehicle->getId());
            }
            ret.emplace(lane->getId(), vehicles);
        }
        return ret;
    }

    std::map<std::string, double> Engine::getVehicleSpeed() const {
        std::map<std::string, double> ret;
        for (const Vehicle* vehicle : getRunningVehicles()) {
            ret.emplace(vehicle->getId(), vehicle->getSpeed());
        }
        return ret;
    }

    std::map<std::string, double> Engine::getVehicleDistance() const {
        std::map<std::string, double> ret;
        for (const Vehicle* vehicle : getRunningVehicles()) {
            ret.emplace(vehicle->getId(), vehicle->getDistance());
        }
        return ret;
    }

    double Engine::getCurrentTime() const {
        return step * interval;
    }

    double Engine::getAverageTravelTime() const {
        double tt = cumulativeTravelTime;
        int n = finishedVehicleCnt;
        for (auto &vehicle_pair : vehiclePool) {
            auto &vehicle = vehicle_pair.second.first;
            tt += getCurrentTime() - vehicle->getEnterTime();
            n++;
        }
        return n == 0 ? 0 : tt / n;
    }

    void Engine::pushVehicle(const std::map<std::string, double> &info, const std::vector<std::string> &roads) {
        VehicleInfo vehicleInfo;
        std::map<std::string, double>::const_iterator it;
        if ((it = info.find("speed")) != info.end()) vehicleInfo.speed = it->second;
        if ((it = info.find("length")) != info.end()) vehicleInfo.len = it->second;
        if ((it = info.find("width")) != info.end()) vehicleInfo.width = it->second;
        if ((it = info.find("maxPosAcc")) != info.end()) vehicleInfo.maxPosAcc = it->second;
        if ((it = info.find("maxNegAcc")) != info.end()) vehicleInfo.maxNegAcc = it->second;
        if ((it = info.find("usualPosAcc")) != info.end()) vehicleInfo.usualPosAcc = it->second;
        if ((it = info.find("usualNegAcc")) != info.end()) vehicleInfo.usualNegAcc = it->second;
        if ((it = info.find("minGap")) != info.end()) vehicleInfo.minGap = it->second;
        if ((it = info.find("maxSpeed")) != info.end()) vehicleInfo.maxSpeed = it->second;
        if ((it = info.find("headwayTime")) != info.end()) vehicleInfo.headwayTime = it->second;

        std::vector<Road *> routes;
        routes.reserve(roads.size());
        for (auto &road: roads) routes.emplace_back(roadnet.getRoadById(road));
        auto route = std::make_shared<const Route>(routes);
        vehicleInfo.route = route;
        
        Vehicle *vehicle = new Vehicle(vehicleInfo,
            "manually_pushed_" + std::to_string(manuallyPushCnt++), this);
        pushVehicle(vehicle, false);
        vehicle->getFirstRoad()->addPlanRouteVehicle(vehicle);
    }

    void Engine::setTrafficLightPhase(const std::string &id, int phaseIndex) {
        if (!rlTrafficLight) {
            std::cerr << "please set rlTrafficLight to true to enable traffic light control" << std::endl;
            return;
        }
        roadnet.getIntersectionById(id)->getTrafficLight().setPhase(phaseIndex);
    }

    void Engine::setReplayLogFile(const std::string &logFile) {
        if (!saveReplayInConfig) {
            std::cerr << "saveReplay is not set to true in config file!" << std::endl;
            return;
        }
        if (logOut.is_open()) logOut.close();
        logOut.open(dir + logFile);
    }

    void Engine::setSaveReplay(bool open) {
        if (!saveReplayInConfig) {
            std::cerr << "saveReplay is not set to true in config file!" << std::endl;
            return;
        }
        saveReplay = open;
    }
    
    void Engine::reset(bool resetRnd) {
        for (auto &vehiclePair : vehiclePool) delete vehiclePair.second.first;
        for (auto &pool : threadVehiclePool) pool.clear();
        vehiclePool.clear();
        vehicleMap.clear();
        roadnet.reset();

        finishedVehicleCnt = 0;
        cumulativeTravelTime = 0;

        for (auto &flow : flows) flow.reset();
        step = 0;
        activeVehicleCount = 0;
        if (resetRnd) {
            rnd.seed(seed);
        }
    }

    Engine::~Engine() {
        logOut.close();
        finished = true;
        for (int i = 0; i < (laneChange ? 9 : 6); ++i) {
            startBarrier.wait();
            endBarrier.wait();
        }
        for (auto &thread : threadPool) thread.join();
        for (auto &vehiclePair : vehiclePool) delete vehiclePair.second.first;
    }
    
    void Engine::setLogFile(const std::string &jsonFile, const std::string &logFile) {
        if (!writeJsonToFile(jsonFile, jsonRoot)) {
            std::cerr << "write roadnet log file error" << std::endl;
        }
        logOut.open(logFile);
    }

    std::vector<const Vehicle *> Engine::getRunningVehicles(bool includeWaiting) const {
        std::vector<const Vehicle *> ret;
        ret.reserve(activeVehicleCount);
        for (const auto &vehiclePair: vehiclePool) {
            const Vehicle *vehicle = vehiclePair.second.first;
            if (vehicle->isReal() && (includeWaiting || vehicle->isRunning())) {
                ret.emplace_back(vehicle);
            }
        }
        return ret;
    }

    void Engine::scheduleLaneChange() {
        std::sort(laneChangeNotifyBuffer.begin(), laneChangeNotifyBuffer.end(),
                [](Vehicle *a, Vehicle *b){return a->laneChangeUrgency() > b->laneChangeUrgency();});
        for (auto v : laneChangeNotifyBuffer){
            v->updateLaneChangeNeighbor();
            v->sendSignal();
            // Lane Change
            // Insert a shadow vehicle
            if (v->planLaneChange() && v->canChange() && !v->isChanging()) {
                std::shared_ptr<LaneChange> lc = v->getLaneChange();
                if (lc->isGapValid() && v->getCurDrivable()->isLane()) {
//                    std::cerr << getCurrentTime() <<" " << v->getId() << " dis: "<< v->getDistance() <<" Can Change from "
//                              << ((Lane*)v->getCurDrivable())->getId()<< " to " << lc->getTarget()->getId() << std::endl;
                insertShadow(v);
                }
            }
        }
        laneChangeNotifyBuffer.clear();
    }

    void Engine::insertShadow(Vehicle *vehicle) {
        size_t threadIndex = vehiclePool.at(vehicle->getPriority()).second;
        Vehicle *shadow = new Vehicle(*vehicle, vehicle->getId() + "_shadow", this);
        vehicleMap.emplace(shadow->getId(), shadow);
        vehiclePool.emplace(shadow->getPriority(), std::make_pair(shadow, threadIndex));
        threadVehiclePool[threadIndex].insert(shadow);
        vehicle->insertShadow(shadow);
        activeVehicleCount++;
    }

    void Engine::loadFromFile(const char *fileName) {
        Archive archive(*this, fileName);
        archive.resume(*this);
    }

    void Engine::setVehicleSpeed(const std::string &id, double speed) {
        auto iter = vehicleMap.find(id);
        if (iter == vehicleMap.end()) {
            throw std::runtime_error("Vehicle '" + id + "' not found");
        }else {
            iter->second->setCustomSpeed(speed);
        }
    }

    std::string Engine::getLeader(const std::string &vehicleId) const {
        auto iter = vehicleMap.find(vehicleId);
        if (iter == vehicleMap.end()) {
            throw std::runtime_error("Vehicle '" + vehicleId + "' not found");
        }else {
            Vehicle *vehicle = iter->second;
            if (laneChange) {
                if (!vehicle->isReal())
                    vehicle = vehicle->getPartner();
            }
            Vehicle *leader = vehicle->getLeader();
            if (leader) return leader->getId();
            else return "";
        }
    }

    bool Engine::setRoute(const std::string &vehicle_id, const std::vector<std::string> &anchor_id) {
        auto vehicle_itr = vehicleMap.find(vehicle_id);
        if (vehicle_itr == vehicleMap.end()) return false;
        Vehicle *vehicle = vehicle_itr->second;

        std::vector<Road *> anchors;
        for (const auto &id : anchor_id) {
            auto anchor = roadnet.getRoadById(id);
            if (!anchor)
                return false;
            anchors.emplace_back(anchor);
        }

        return vehicle->setRoute(anchors);
    }

    std::map<std::string, std::string> Engine::getVehicleInfo(const std::string &id) const {
        auto iter = vehicleMap.find(id);
        if (iter == vehicleMap.end()) {
            throw std::runtime_error("Vehicle '" + id + "' not found");
        }else {
            Vehicle *vehicle = iter->second;
            return vehicle->getInfo();
        }
    }

}
