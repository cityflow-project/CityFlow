#include "vehicle/vehicle.h"
#include "roadnet/roadnet.h"
#include "engine/engine.h"

#include <iostream>
#include <cstdlib>
#include <ctime>
#include <cassert>
#include <limits>
#include <random>

namespace CityFlow {

    Vehicle::ControllerInfo::ControllerInfo(const Vehicle *vehicle, std::shared_ptr<const Route> route, std::mt19937 *rnd) 
        : router(vehicle, route, rnd) {
        enterLaneLinkTime = std::numeric_limits<int>::max();
        drivable = router.getFirstDrivable();
    }

    Vehicle::Vehicle(const Vehicle &vehicle, const std::string &id, Engine *engine) 
        : vehicleInfo(vehicle.vehicleInfo), controllerInfo(vehicle.controllerInfo),
          laneChangeInfo(vehicle.laneChangeInfo), buffer(vehicle.buffer), 
          priority(priority), id(id), engine(engine) {
        while (engine->checkPriority(priority = engine->rnd()));
    }

    Vehicle::Vehicle(const VehicleInfo &vehicleInfo, const std::string &id, Engine *engine) 
        : vehicleInfo(vehicleInfo), controllerInfo(this, vehicleInfo.route, &(engine->rnd)),
          priority(priority), id(id), engine(engine) {
        controllerInfo.approachingIntersectionDistance =
            vehicleInfo.maxSpeed * vehicleInfo.maxSpeed / vehicleInfo.usualNegAcc / 2 +
            vehicleInfo.maxSpeed * engine->getInterval() * 2;
        while (engine->checkPriority(priority = engine->rnd()));
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
        if (!buffer.isSpeedSet) {
            buffer.speed = speed;
            buffer.isSpeedSet = true;
        } else {
            buffer.speed = min2double(buffer.speed, speed);
        }
    }

    Drivable *Vehicle::getChangedDrivable() const {
        if (!buffer.isDrivableSet)
            return nullptr;
        return buffer.drivable;
    }

    void Vehicle::addLaneChangeNotify(CityFlow::Vehicle *vehicle) {
        buffer.isNotifiedVehicles = true;
        buffer.notifiedVehicles.emplace_back(vehicle);
    }

    Point Vehicle::getPoint() const {
        if (fabs(laneChangeInfo.offset) < eps) {
            return controllerInfo.drivable->getPointByDistance(controllerInfo.dis);
        } else {
            assert(controllerInfo.drivable->isLane());
            const Lane* lane = static_cast<const Lane*>(controllerInfo.drivable);
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
        if (buffer.isDrivableSet) {
            controllerInfo.prevDrivable = controllerInfo.drivable;
            controllerInfo.drivable = buffer.drivable;
            buffer.isDrivableSet = false;
            if (controllerInfo.drivable->isLane()) {
                laneChangeInfo.changed = !controllerInfo.router.onValidLane();
            }
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
            laneChangeInfo.notifiedVehicles = buffer.notifiedVehicles;
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
        if (leader != nullptr) {
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
        if (c > 0) return -100;
        double a = 0.5 / dF;
        double b = 0.5 * interval;
        return 0.5 / a * (sqrt(b * b - 4 * a * c) - b);
    }

    // should be move to seperate CarFollowing (Controller?) class later?
    double Vehicle::getCarFollowSpeed(double interval) {
        Vehicle *leader = getLeader();
        if (leader == nullptr) return vehicleInfo.maxSpeed;

        // collision free
        double v = getNoCollisionSpeed(leader->getSpeed(), leader->getMaxNegAcc(), vehicleInfo.speed,
                                       vehicleInfo.maxNegAcc, controllerInfo.gap, interval, 0);
        int cnt = 0;
        for (auto vehicle : laneChangeInfo.notifiedVehicles) {
            double _v = getNoCollisionSpeed(vehicle->getSpeed(), vehicle->getUsualNegAcc(), vehicleInfo.speed,
                                            vehicleInfo.maxNegAcc,
                                            vehicle->getDistance() - vehicle->getLen() - controllerInfo.dis, interval, 0);
            if (_v != -100) {
                v = min2double(v, _v);
                ++cnt;
            }
        }
        // safe distance
        // get relative decel (mimic real scenario)
        double assumeDecel = 0, leaderSpeed = leader->getSpeed();
        if (vehicleInfo.speed > leaderSpeed) {
            assumeDecel = vehicleInfo.speed - leaderSpeed;
        }
        v = min2double(v, getNoCollisionSpeed(leader->getSpeed(), leader->getUsualNegAcc(), vehicleInfo.speed,
                                              vehicleInfo.usualNegAcc, controllerInfo.gap, interval,
                                              vehicleInfo.minGap));
        v = min2double(v,
                       (controllerInfo.gap + (leaderSpeed - assumeDecel / 2) * interval -
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
        if (v == -100 || v < vehicleInfo.speed - vehicleInfo.maxNegAcc * interval) {
            controlInfo.collision = true;
        }
        if (isIntersectionRelated()) {
            v = min2double(v, getIntersectionRelatedSpeed(interval));
        }
        v = max2double(v, vehicleInfo.speed - vehicleInfo.maxNegAcc * interval);
        controlInfo.speed = v;
        // todo: lane change related speed
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

    //Hard Check stands for a two-interval worst situation check, easy check will just check in one interval;
    bool Vehicle::checkSegment(Lane *lan, size_t index, double interval, double nextSpeed, bool isHard) const {
        if (index >= lan->getSegmentNum() || index < 0)
            return true;
        Segment *seg = lan->getSegment(index);
        std::list<std::list<Vehicle *>::iterator> &vehicles = seg->getVehicles();
        if (!vehicles.empty())
            for (auto veh: vehicles) {
                if ((*veh)->getDistance() > this->getDistance()) {
                    double
                            front_gap =
                            (*veh)->getDistance() - this->getDistance() - (*veh)->getLen() - nextSpeed * interval +
                            ((*veh)->getSpeed() * (*veh)->getSpeed() -
                             max2double(0, (*veh)->getSpeed() - (*veh)->getMaxNegAcc() * interval) *
                             max2double(0, (*veh)->getSpeed() - (*veh)->getMaxNegAcc() * interval)) /
                            (2 * (*veh)->getMaxNegAcc());
                    if (!isHard) {
                        if (front_gap < 0)
                            return false;
                    } else {
                        if (front_gap < 0)
                            return false;
                        double v = getNoCollisionSpeed((*veh)->getSpeed(), (*veh)->getMaxNegAcc(), nextSpeed,
                                                       vehicleInfo.maxNegAcc,
                                                       (*veh)->getDistance() - (*veh)->getLen() - controllerInfo.dis,
                                                       interval, 0);
                        if (v == -100 || nextSpeed - v > vehicleInfo.maxNegAcc * interval)
                            return false;
                    }
                } else {
                    double back_gap =
                            this->getDistance() - (*veh)->getDistance() - this->getLen() + nextSpeed * interval -
                            (*veh)->getSpeed() * interval - 0.5 * (*veh)->getUsualPosAcc() * interval * interval;
                    if (!isHard) {
                        if (back_gap < 0)
                            return false;
                    } else {
                        if (back_gap < 0)
                            return false;
                        double v = getNoCollisionSpeed(this->getSpeed(), this->getMaxNegAcc(),
                                                       (*veh)->getSpeed() + (*veh)->getMaxPosAcc() * interval,
                                                       (*veh)->getMaxNegAcc(),
                                                       this->getDistance() - this->getLen() - (*veh)->getDistance(),
                                                       interval, 0);
                        if (v == -100 ||
                            (*veh)->getSpeed() - v >
                            (*veh)->getMaxNegAcc() * interval)
                            return false;
                    }
                }
            }
        return true;
    }

    //0 for no change, 1 for left; 2 for right; 3 for wait for left; 4 for wait for right; 56 for try to change when wait for changing
    int Vehicle::toChange(double interval, double nextSpeed) {
        if (laneChangeInfo.partnerType == 2)
            return 0;
        if (controllerInfo.drivable->isLane())
            return 0;
        if (fabs(laneChangeInfo.offset) > eps && !laneChangeInfo.changed) {
            if (laneChangeInfo.offset > 0)
                return 1;
            else
                return 2;
        }
        if (fabs(laneChangeInfo.offset) > eps && laneChangeInfo.changed) {
            if (laneChangeInfo.offset > 0)
                return 5;
            else
                return 6;
        }
        auto curLane = static_cast<const Lane*>(controllerInfo.drivable);
        auto &segmentIndex = laneChangeInfo.segmentIndex;
        std::vector<Lane> &lanes = curLane->getBelongRoad()->getLanes();
        if (laneChangeInfo.changed) {
            if (curLane->getLength() - controllerInfo.dis - vehicleInfo.speed * interval <
                laneChangeInfo.bufferLength * 2) {
                if (curLane->getLaneIndex() - 1 >= 0 && lanes.size() > 0 &&
                    controllerInfo.router.onLastRoad() &&
                    lanes[curLane->getLaneIndex() - 1].getLaneLinks()[0]->getRoadLinkType() ==
                    ((Lane *) controllerInfo.drivable)->getLaneLinks()[0]->getRoadLinkType()) {
//                    if (checkSegment(&lanes[curLane->getLaneIndex() - 1], segmentIndex, interval,
//                                     nextSpeed, true) &&
//                        checkSegment(&lanes[curLane->getLaneIndex() - 1], segmentIndex - 1,
//                                     interval, nextSpeed, true) && checkSegment(
//                            &lanes[curLane->getLaneIndex() - 1], segmentIndex + 1, interval, nextSpeed, true)) {
//                        return 2;
//                    } else
                    return 4;
                } else if (curLane->getLaneIndex() + 1 < lanes.size()) {
//                    if (checkSegment(&lanes[curLane->getLaneIndex() + 1], segmentIndex, interval,
//                                     nextSpeed, true) &&
//                        checkSegment(&lanes[curLane->getLaneIndex() + 1], segmentIndex - 1,
//                                     interval, nextSpeed, true) && checkSegment(
//                            &lanes[curLane->getLaneIndex() + 1], segmentIndex + 1, interval, nextSpeed, true)) {
//                        return 1;
//                    } else
                    return 3;
                }
            }
            if (controllerInfo.leader != nullptr && tryChange()) {
                if (curLane->getLaneIndex() - 1 >= 0 && lanes.size() > 0 &&
                    controllerInfo.router.onLastRoad() &&
                    lanes[curLane->getLaneIndex() - 1].getLaneLinks()[0]->getRoadLinkType() ==
                    ((Lane *) controllerInfo.drivable)->getLaneLinks()[0]->getRoadLinkType()) {
                    if (checkSegment(&lanes[curLane->getLaneIndex() - 1], segmentIndex, interval,
                                     nextSpeed, true) &&
                        checkSegment(&lanes[curLane->getLaneIndex() - 1], segmentIndex - 1,
                                     interval, nextSpeed, true) && checkSegment(
                            &lanes[curLane->getLaneIndex() - 1], segmentIndex + 1, interval, nextSpeed, true)) {
                        return 2;
                    }
                } else if (curLane->getLaneIndex() + 1 < lanes.size() &&
                           lanes[curLane->getLaneIndex() + 1].getLaneLinks()[0]->getRoadLinkType() ==
                           ((Lane *) controllerInfo.drivable)->getLaneLinks()[0]->getRoadLinkType()) {
                    if (checkSegment(&lanes[curLane->getLaneIndex() + 1], segmentIndex, interval,
                                     nextSpeed, true) &&
                        checkSegment(&lanes[curLane->getLaneIndex() + 1], segmentIndex - 1,
                                     interval, nextSpeed, true) && checkSegment(
                            &lanes[curLane->getLaneIndex() + 1], segmentIndex + 1, interval, nextSpeed, true)) {
                        return 1;
                    }
                }
            }

        } else if (controllerInfo.leader != nullptr && tryChange()) {
            if (controllerInfo.drivable->getLength() - controllerInfo.dis < 6 * laneChangeInfo.bufferLength ||
                controllerInfo.dis < laneChangeInfo.bufferLength * 2)
                return 0;
            if (curLane->getLaneIndex() - 1 >= 0 && !lanes.empty()) {
                if (checkSegment(&lanes[curLane->getLaneIndex() - 1], segmentIndex, interval,
                                 nextSpeed, true) && checkSegment(&lanes[curLane->getLaneIndex() - 1], segmentIndex - 1,
                                                                  interval, nextSpeed, true) && checkSegment(
                        &lanes[curLane->getLaneIndex() - 1], segmentIndex + 1, interval, nextSpeed, true)) {
                    return 2;
                }
            }
            if (curLane->getLaneIndex() + 1 < lanes.size()) {
                if (checkSegment(&lanes[curLane->getLaneIndex() + 1], segmentIndex, interval,
                                 nextSpeed, true) && checkSegment(&lanes[curLane->getLaneIndex() + 1], segmentIndex - 1,
                                                                  interval, nextSpeed, true) && checkSegment(
                        &lanes[curLane->getLaneIndex() + 1], segmentIndex + 1, interval, nextSpeed, true)) {
                    return 1;
                }
            }
        }
        return 0;
    }

    double Vehicle::findNextGap(double dis, Lane *lane, size_t segmentIndex) {
        auto &vehs = lane->getSegment(segmentIndex)->getVehicles();
        double last = 0.0, first = lane->getLength() / lane->getSegments().size();
        for (auto &veh : vehs) {
//            if ((*veh)->getDistance() < dis && (*veh)->getDistance() > last)
//                last = (*veh)->getDistance();
            if ((*veh)->getDistance() > dis && (*veh)->getDistance() < first)
                first = (*veh)->getDistance();
        }
        if (segmentIndex > 0) {
            auto &vehs = lane->getSegment(segmentIndex - 1)->getVehicles();
            for (auto &veh : vehs) {
//                if ((*veh)->getDistance() < dis && (*veh)->getDistance() > last)
//                    last = (*veh)->getDistance();
                if ((*veh)->getDistance() > dis && (*veh)->getDistance() < first)
                    first = (*veh)->getDistance();
            }
        }
        if (segmentIndex < lane->getSegmentNum() - 1) {
            auto &vehs = lane->getSegment(segmentIndex + 1)->getVehicles();
            for (auto &veh : vehs) {
//                if ((*veh)->getDistance() < dis && (*veh)->getDistance() > last)
//                    last = (*veh)->getDistance();
                if ((*veh)->getDistance() > dis && (*veh)->getDistance() < first)
                    first = (*veh)->getDistance();
            }
        }
        return (first - dis);
    }

    bool Vehicle::tryChange() {
        assert(controllerInfo.drivable->isLane());
        const Lane *lane = static_cast<const Lane*>(controllerInfo.drivable);
        double gap = controllerInfo.leader->getDistance() - controllerInfo.dis;
        if (vehicleInfo.speed > controllerInfo.leader->getSpeed()) {
            return (gap / (vehicleInfo.speed - controllerInfo.leader->getSpeed()) < 5.0);
        } else if (vehicleInfo.speed == 0) {
            double nextGap = 0.0;
            if (lane->getLaneIndex() > 0)
                nextGap = max2double(nextGap, findNextGap(controllerInfo.dis,
                                                          &lane->getBelongRoad()->getLanes()[
                                                                  lane->getLaneIndex() - 1],
                                                          laneChangeInfo.segmentIndex));
            if (lane->getLaneIndex() < lane->getBelongRoad()->getLanes().size() - 1)
                nextGap = max2double(nextGap, findNextGap(controllerInfo.dis,
                                                          &lane->getBelongRoad()->getLanes()[
                                                                  lane->getLaneIndex() + 1],
                                                          laneChangeInfo.segmentIndex));
            return (gap + getMinGap() < nextGap);
        }
    }

    void Vehicle::finishChanging() {
        if (controllerInfo.drivable->isLane()) {
            laneChangeInfo.changed = !controllerInfo.router.onValidLane();
        }
        laneChangeInfo.partnerType = 0;
        laneChangeInfo.offset = 0;
    }

    void Vehicle::setLane(Lane *nextLane) {
        controllerInfo.drivable = nextLane;
    }

    bool Vehicle::isChanged() {
        return laneChangeInfo.changed;
    }

    Drivable *Vehicle::getCurDrivable() const {
        return controllerInfo.drivable;
    }

    bool Vehicle::isLaneChanged(Lane *curLane, LaneLink *originLaneLink) {
        if (curLane->getLaneLinks().empty())
            return false;
        for (auto &laneLink : curLane->getLaneLinks())
            if (laneLink->getRoadLinkType() == originLaneLink->getRoadLinkType())
                return false;
        return true;
    }

    void Vehicle::resetLaneChange() {
        laneChangeInfo.partnerType = 0;
        laneChangeInfo.partner = nullptr;
        laneChangeInfo.target = nullptr;
        laneChangeInfo.offset = 0;
    }
}
