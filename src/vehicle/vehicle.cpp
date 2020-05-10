#include "vehicle/vehicle.h"
#include "engine/engine.h"

#include <iostream>
#include <limits>
#include <random>

namespace CityFlow {

    Vehicle::ControllerInfo::ControllerInfo(Vehicle *vehicle, std::shared_ptr<const Route> route, std::mt19937 *rnd)
        : router(vehicle, route, rnd) {
        enterLaneLinkTime = std::numeric_limits<int>::max();
    }

    Vehicle::ControllerInfo::ControllerInfo(Vehicle *vehicle, const Vehicle::ControllerInfo &other): ControllerInfo(other) {
        router.setVehicle(vehicle);
    }

    Vehicle::Vehicle(const Vehicle &vehicle, Flow *flow)
        : vehicleInfo(vehicle.vehicleInfo), controllerInfo(this, vehicle.controllerInfo),
          laneChangeInfo(vehicle.laneChangeInfo), buffer(vehicle.buffer), priority(vehicle.priority),
          id(vehicle.id), engine(vehicle.engine),
          laneChange(std::make_shared<SimpleLaneChange>(this, *vehicle.laneChange)),
          flow(flow){
        enterTime = vehicle.enterTime;
    }

    Vehicle::Vehicle(const Vehicle &vehicle, const std::string &id, Engine *engine, Flow *flow)
        : vehicleInfo(vehicle.vehicleInfo), controllerInfo(this, vehicle.controllerInfo),
          laneChangeInfo(vehicle.laneChangeInfo), buffer(vehicle.buffer), 
          id(id), engine(engine), laneChange(std::make_shared<SimpleLaneChange>(this)),
          flow(flow){
        while (engine->checkPriority(priority = engine->rnd()));
        controllerInfo.router.setVehicle(this);
        enterTime = vehicle.enterTime;
    }

    Vehicle::Vehicle(const VehicleInfo &vehicleInfo, const std::string &id, Engine *engine, Flow *flow)
        : vehicleInfo(vehicleInfo), controllerInfo(this, vehicleInfo.route, &(engine->rnd)),
          id(id), engine(engine), laneChange(std::make_shared<SimpleLaneChange>(this)),
          flow(flow){
        controllerInfo.approachingIntersectionDistance =
            vehicleInfo.maxSpeed * vehicleInfo.maxSpeed / vehicleInfo.usualNegAcc / 2 +
            vehicleInfo.maxSpeed * engine->getInterval() * 2;
        while (engine->checkPriority(priority = engine->rnd()));
        enterTime = engine->getCurrentTime();
    }

    void Vehicle::setDeltaDistance(double dis) {
        if (!buffer.isDisSet || dis < buffer.deltaDis) {
            unSetEnd();
            unSetDrivable();
            buffer.deltaDis = dis;
            dis = dis + controllerInfo.dis;
            Drivable *drivable = getCurDrivable();
            for (int i = 0; drivable && dis > drivable->getLength(); ++i) {
                dis -= drivable->getLength();
                Drivable *nextDrivable = controllerInfo.router.getNextDrivable(i);
                if (nextDrivable == nullptr) {
                    assert(controllerInfo.router.isLastRoad(drivable));
                    setEnd(true);
                }
                drivable = nextDrivable;
                setDrivable(drivable);
            }
            setDis(dis);
        }
    }

    void Vehicle::setSpeed(double speed) {
        buffer.speed = speed;
        buffer.isSpeedSet = true;
    }

    Drivable *Vehicle::getChangedDrivable() const {
        if (!buffer.isDrivableSet)
            return nullptr;
        return buffer.drivable;
    }

    Point Vehicle::getPoint() const {
        if (fabs(laneChangeInfo.offset) < eps || !controllerInfo.drivable->isLane()) {
            return controllerInfo.drivable->getPointByDistance(controllerInfo.dis);
        } else {
            assert(controllerInfo.drivable->isLane());
            const Lane *lane = static_cast<const Lane*>(controllerInfo.drivable);
            Point origin = lane->getPointByDistance(controllerInfo.dis);
            Point next;
            double percentage;
            std::vector<Lane> &lans = lane->getBelongRoad()->getLanes();
            if (laneChangeInfo.offset > 0) {
                next = lans[lane->getLaneIndex() + 1].getPointByDistance(controllerInfo.dis);
                percentage = 2 * laneChangeInfo.offset / (lane->getWidth() +
                                                          lans[lane->getLaneIndex() + 1].getWidth());
            } else {
                next = lans[lane->getLaneIndex() - 1].getPointByDistance(controllerInfo.dis);
                percentage = -2 * laneChangeInfo.offset / (lane->getWidth() +
                                                           lans[lane->getLaneIndex() - 1].getWidth());
            }
            Point cur;
            cur.x = next.x * percentage + origin.x * (1 - percentage);
            cur.y = next.y * percentage + origin.y * (1 - percentage);
            return cur;
        }
    }

    void Vehicle::update() { // TODO: use something like reflection?
        if (buffer.isEndSet) {
            controllerInfo.end = buffer.end;
            buffer.isEndSet = false;
        }
        if (buffer.isDisSet) {
            controllerInfo.dis = buffer.dis;
            buffer.isDisSet = false;
        }
        if (buffer.isSpeedSet) {
            vehicleInfo.speed = buffer.speed;
            buffer.isSpeedSet = false;
        }
        if (buffer.isCustomSpeedSet) {
            buffer.isCustomSpeedSet = false;
        }
        if (buffer.isDrivableSet) {
            controllerInfo.prevDrivable = controllerInfo.drivable;
            controllerInfo.drivable = buffer.drivable;
            buffer.isDrivableSet = false;
            controllerInfo.router.update();
        }
        if (buffer.isEnterLaneLinkTimeSet) {
            controllerInfo.enterLaneLinkTime = buffer.enterLaneLinkTime;
            buffer.isEnterLaneLinkTimeSet = false;
        }
        if (buffer.isBlockerSet) {
            controllerInfo.blocker = buffer.blocker;
            buffer.isBlockerSet = false;
        } else {
            controllerInfo.blocker = nullptr;
        }
        if (buffer.isNotifiedVehicles) {
            buffer.notifiedVehicles.clear();
            buffer.isNotifiedVehicles = false;
        }
    }


    std::pair<Point, Point> Vehicle::getCurPos() const {
        std::pair<Point, Point> ret;
        ret.first = controllerInfo.drivable->getPointByDistance(controllerInfo.dis);
        Point direction = controllerInfo.drivable->getDirectionByDistance(controllerInfo.dis);
        Point tail(ret.first);
        tail.x -= direction.x * vehicleInfo.len;
        tail.y -= direction.y * vehicleInfo.len;
        ret.second = tail;
        return ret;
    }

    void Vehicle::updateLeaderAndGap(Vehicle *leader) {
        if (leader != nullptr && leader->getCurDrivable() == getCurDrivable()) {
            controllerInfo.leader = leader;
            controllerInfo.gap = leader->getDistance() - leader->getLen() - controllerInfo.dis;
        } else {
            controllerInfo.leader = nullptr;
            Drivable *drivable = nullptr;
            Vehicle *candidateLeader = nullptr;
            double candidateGap = 0;
            double dis = controllerInfo.drivable->getLength() - controllerInfo.dis;
            for (int i = 0; ; ++i) {
                drivable = getNextDrivable(i);
                if (drivable == nullptr) return;
                if (drivable->isLaneLink()) { // if laneLink, check all laneLink start from previous lane, because lanelinks may overlap 
                    for (auto laneLink : static_cast<LaneLink *>(drivable)->getStartLane()->getLaneLinks()) {
                        if ((candidateLeader = laneLink->getLastVehicle()) != nullptr) {
                            candidateGap = dis + candidateLeader->getDistance() - candidateLeader->getLen();
                            if (controllerInfo.leader == nullptr || candidateGap < controllerInfo.gap) {
                                controllerInfo.leader = candidateLeader;
                                controllerInfo.gap = candidateGap;
                            }
                        }
                    }
                    if (controllerInfo.leader) return;
                } else {
                    if ((controllerInfo.leader = drivable->getLastVehicle()) != nullptr) {
                        controllerInfo.gap =
                                dis + controllerInfo.leader->getDistance() - controllerInfo.leader->getLen();
                        return;
                    }
                }

                dis += drivable->getLength();
                if (dis > vehicleInfo.maxSpeed * vehicleInfo.maxSpeed / vehicleInfo.usualNegAcc / 2 +
                          vehicleInfo.maxSpeed * engine->getInterval() * 2)
                    return;
            }
            return;
        }
    }



    double Vehicle::getNoCollisionSpeed(double vL, double dL, double vF, double dF, double gap, double interval,
                                        double targetGap) const {
        double c = vF * interval / 2 + targetGap - 0.5 * vL * vL / dL - gap;
        double a = 0.5 / dF;
        double b = 0.5 * interval;
        if (b * b < 4 * a * c) return -100;
        double v1 = 0.5 / a * (sqrt(b * b - 4 * a * c) - b);
        double v2 = 2 * vL - dL * interval + 2 * (gap - targetGap) / interval;
        return min2double(v1, v2);
    }

    // should be move to seperate CarFollowing (Controller?) class later?
    double Vehicle::getCarFollowSpeed(double interval) {
        Vehicle *leader = getLeader();
        if (leader == nullptr) return hasSetCustomSpeed() ? buffer.customSpeed : vehicleInfo.maxSpeed;

        // collision free
        double v = getNoCollisionSpeed(leader->getSpeed(), leader->getMaxNegAcc(), vehicleInfo.speed,
                                       vehicleInfo.maxNegAcc, controllerInfo.gap, interval, 0);

        if (hasSetCustomSpeed())
            return min2double(buffer.customSpeed, v);

        // safe distance
        // get relative decel (mimic real scenario)
        double assumeDecel = 0, leaderSpeed = leader->getSpeed();
        if (vehicleInfo.speed > leaderSpeed) {
            assumeDecel = vehicleInfo.speed - leaderSpeed;
        }
        v = min2double(v, getNoCollisionSpeed(leader->getSpeed(), leader->getUsualNegAcc(), vehicleInfo.speed,
                                              vehicleInfo.usualNegAcc, controllerInfo.gap, interval,
                                              vehicleInfo.minGap
                                              ));
        v = min2double(v,
                       (controllerInfo.gap + (leaderSpeed + assumeDecel / 2) * interval -
                        vehicleInfo.speed * interval / 2) / (vehicleInfo.headwayTime + interval / 2));

        return v;
    }

    double Vehicle::getStopBeforeSpeed(double distance, double interval) const {
        assert(distance >= 0);
        if (getBrakeDistanceAfterAccel(vehicleInfo.usualPosAcc, vehicleInfo.usualNegAcc, interval) < distance)
            return vehicleInfo.speed + vehicleInfo.usualPosAcc * interval;
        double takeInterval = 2 * distance / (vehicleInfo.speed + eps) / interval;
        if (takeInterval >= 1) {
            return vehicleInfo.speed - vehicleInfo.speed / (int) takeInterval;
        } else {
            return vehicleInfo.speed - vehicleInfo.speed / takeInterval;
        }
    }

    int Vehicle::getReachSteps(double distance, double targetSpeed, double acc) const {
        if (distance <= 0) {
            return 0;
        }
        if (vehicleInfo.speed > targetSpeed) {
            return std::ceil(distance / vehicleInfo.speed);
        }
        double distanceUntilTargetSpeed = getDistanceUntilSpeed(targetSpeed, acc);
        double interval = engine->getInterval();
        if (distanceUntilTargetSpeed > distance) {
            return std::ceil((std::sqrt(
                    vehicleInfo.speed * vehicleInfo.speed + 2 * acc * distance) - vehicleInfo.speed) / acc / interval);
        } else {
            return std::ceil((targetSpeed - vehicleInfo.speed) / acc / interval) + std::ceil(
                    (distance - distanceUntilTargetSpeed) / targetSpeed / interval);
        }
    }

    int Vehicle::getReachStepsOnLaneLink(double distance, LaneLink *laneLink) const {
        return getReachSteps(distance, laneLink->isTurn() ? vehicleInfo.turnSpeed : vehicleInfo.maxSpeed,
                             vehicleInfo.usualPosAcc);
    }

    double Vehicle::getDistanceUntilSpeed(double speed, double acc) const {
        if (speed <= vehicleInfo.speed) return 0;
        double interval = engine->getInterval();
        int stage1steps = std::floor((speed - vehicleInfo.speed) / acc / interval);
        double stage1speed = vehicleInfo.speed + stage1steps * acc / interval;
        double stage1dis = (vehicleInfo.speed + stage1speed) * (stage1steps * interval) / 2;
        return stage1dis + (stage1speed < speed ? ((stage1speed + speed) * interval / 2) : 0);
    }

    bool Vehicle::canYield(double dist) const {
        return (dist > 0 && getMinBrakeDistance() < dist - vehicleInfo.yieldDistance) ||
               (dist < 0 && dist + vehicleInfo.len < 0);
    }

    bool Vehicle::isIntersectionRelated() {
        if (controllerInfo.drivable->isLaneLink())
            return true;
        if (controllerInfo.drivable->isLane()) {
            Drivable *drivable = getNextDrivable();
            if (drivable && drivable->isLaneLink() && controllerInfo.drivable->getLength() - controllerInfo.dis <=
                                                      controllerInfo.approachingIntersectionDistance) {
                return true;
            }
        }
        return false;
    }

    double Vehicle::getBrakeDistanceAfterAccel(double acc, double dec, double interval) const {
        double currentSpeed = vehicleInfo.speed;
        double nextSpeed = currentSpeed + acc * interval;
        return (currentSpeed + nextSpeed) * interval / 2 + (nextSpeed * nextSpeed / dec / 2);
    }

    ControlInfo Vehicle::getNextSpeed(double interval) { // TODO: pass as parameter or not?
        ControlInfo controlInfo;
        Drivable *drivable = controllerInfo.drivable;
        double v = vehicleInfo.maxSpeed;
        v = min2double(v, vehicleInfo.speed + vehicleInfo.maxPosAcc * interval); // TODO: random???

        v = min2double(v, drivable->getMaxSpeed());

        // car follow
        v = min2double(v, getCarFollowSpeed(interval));

        if (isIntersectionRelated()) {
            v = min2double(v, getIntersectionRelatedSpeed(interval));
        }

        if (laneChange){
            v = min2double(v, laneChange->yieldSpeed(interval));
            if (!controllerInfo.router.onValidLane()) {
                double vn = getNoCollisionSpeed(0,1,getSpeed(), getMaxNegAcc(), getCurDrivable()->getLength() - getDistance(), interval, getMinGap());
                v = min2double(v, vn);
            }
        }

        v = max2double(v, vehicleInfo.speed - vehicleInfo.maxNegAcc * interval);
        controlInfo.speed = v;

        return controlInfo;
    }

    double Vehicle::getIntersectionRelatedSpeed(double interval) {
        double v = vehicleInfo.maxSpeed;
        Drivable *nextDrivable = getNextDrivable();
        const LaneLink *laneLink = nullptr;
        if (nextDrivable && nextDrivable->isLaneLink()) {
            laneLink = (LaneLink *) nextDrivable;
            if (!laneLink->isAvailable() || !laneLink->getEndLane()->canEnter(
                    this)) { // not only the first vehicle should follow intersection logic
                if (getMinBrakeDistance() > controllerInfo.drivable->getLength() - controllerInfo.dis) {
                    // TODO: what if it cannot brake before red light?
                } else {
                    v = min2double(v, getStopBeforeSpeed(controllerInfo.drivable->getLength() - controllerInfo.dis,
                                                         interval));
                    return v;
                }
            }
            if (laneLink->isTurn()) {
                v = min2double(v, vehicleInfo.turnSpeed); // TODO: define turn speed
            }
        }
        if (laneLink == nullptr && controllerInfo.drivable->isLaneLink())
            laneLink = static_cast<const LaneLink*>(controllerInfo.drivable);
        double distanceToLaneLinkStart = controllerInfo.drivable->isLane()
                                         ? -(controllerInfo.drivable->getLength() - controllerInfo.dis)
                                         : controllerInfo.dis;
        double distanceOnLaneLink;
        for (auto &cross : laneLink->getCrosses()) {
            distanceOnLaneLink = cross->getDistanceByLane(laneLink);
            if (distanceOnLaneLink < distanceToLaneLinkStart)
                continue;
            if (!cross->canPass(this, laneLink, distanceToLaneLinkStart)) {
                v = min2double(v, getStopBeforeSpeed(
                        distanceOnLaneLink - distanceToLaneLinkStart - vehicleInfo.yieldDistance,
                        interval)); // TODO: headway distance
                setBlocker(cross->getFoeVehicle(laneLink));
                break;
            }
        }
        return v;
    }

    void Vehicle::finishChanging() {
        laneChange->finishChanging();
        setEnd(true);
    }

    void Vehicle::setLane(Lane *nextLane) {
        controllerInfo.drivable = nextLane;
    }

    Drivable *Vehicle::getCurDrivable() const {
        return controllerInfo.drivable;
    }

    void Vehicle::receiveSignal(Vehicle *sender) {

        if (laneChange->changing) return;
        auto signal_recv = laneChange->signalRecv;
        auto signal_send = laneChange->signalSend;
        int curPriority = signal_recv ? signal_recv->source->getPriority() : -1;
        int newPriority = sender->getPriority();

        if ((!signal_recv || curPriority < newPriority) && (!signal_send || priority < newPriority) )
            laneChange->signalRecv = sender->laneChange->signalSend;
    }

    std::list<Vehicle *>::iterator Vehicle::getListIterator() {
        assert(getCurDrivable()->isLane());
        Segment *seg = ((Lane *)getCurDrivable())->getSegment(getSegmentIndex());


        auto result = seg->findVehicle(this);
        return result;
    }

    void Vehicle::abortLaneChange() {
        assert(laneChangeInfo.partner);
        this->setEnd(true);
        laneChange->abortChanging();
    }

    Road *Vehicle::getFirstRoad() {
        return controllerInfo.router.getFirstRoad();
    }

    void Vehicle::setFirstDrivable() {
        controllerInfo.drivable = controllerInfo.router.getFirstDrivable();
    }

    void Vehicle::updateRoute() {
        routeValid = controllerInfo.router.updateShortestPath();
    }

    bool Vehicle::setRoute(const std::vector<Road *> &anchor) {
        return controllerInfo.router.setRoute(anchor);
    }


    std::map<std::string, std::string> Vehicle::getInfo() const{
        std::map<std::string, std::string> info;
        info["running"] = std::to_string(isRunning());
        if (!isRunning()) return info;

        info["distance"] = std::to_string(getDistance());
        info["speed"] = std::to_string(getSpeed());
        const auto &drivable = getCurDrivable();
        info["drivable"] = drivable->getId();
        const auto &road = drivable->isLane() ? getCurLane()->getBelongRoad() : nullptr;
        if (road) {
            info["road"] = road->getId();
            info["intersection"] = road->getEndIntersection().getId();
        }
        // add routing info
        std::string route;
        for (const auto &r : controllerInfo.router.getFollowingRoads()) {
            route += r->getId() + " ";
        }
        info["route"] = route;

        return info;
    }
}
