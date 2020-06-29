#include "vehicle/lanechange.h"
#include "engine/engine.h"

#include <iostream>

namespace CityFlow{
    LaneChange::LaneChange(Vehicle * vehicle, const LaneChange &other)
        : lastDir(other.lastDir), signalRecv(other.signalRecv),
          vehicle(vehicle), targetLeader(other.targetLeader), targetFollower(other.targetFollower), // useless in archive
          leaderGap(other.leaderGap), followerGap(other.followerGap), waitingTime(other.waitingTime),
          changing(other.changing), lastChangeTime(other.lastChangeTime) {
        if (other.signalSend) {
            signalSend = std::make_shared<Signal>(*other.signalSend);
            signalSend->source = vehicle;
        }
    }

    Lane *LaneChange::getTarget() const {
        assert(vehicle->getCurDrivable()->isLane());
        return signalSend ? signalSend->target : (Lane *)vehicle->getCurDrivable();
    }

    bool LaneChange::planChange() const {
        return (signalSend && signalSend->target && signalSend->target != vehicle->getCurDrivable()) || changing;
    }

    void LaneChange::updateLeaderAndFollower() {
        targetLeader = targetFollower = nullptr;
        Lane *target  = signalSend->target;
        targetLeader   = target->getVehicleAfterDistance(vehicle->getDistance(), vehicle->getSegmentIndex());
        Lane *curLane = dynamic_cast<Lane *>(vehicle->getCurDrivable());
        leaderGap = followerGap = std::numeric_limits<double>::max();
        if (!targetLeader){
            // Find target leader in following lanelinks
            double rest = curLane->getLength() - vehicle->getDistance();
            leaderGap = rest;
            double gap = std::numeric_limits<double>::max();
            for (auto lanelink: signalSend->target->getLaneLinks()){
                Vehicle *leader = lanelink->getLastVehicle();
                if (leader && leader->getDistance() + rest < gap ){
                    gap = leader->getDistance() + rest;
                    if (gap < leader->getLen()) {
                        targetLeader = leader;
                        leaderGap = rest - (leader->getLen() - gap);
                    }
                }
            }
        }else{
            leaderGap = targetLeader->getDistance() - vehicle->getDistance() - targetLeader->getLen();
        }

        targetFollower = target->getVehicleBeforeDistance(vehicle->getDistance(), vehicle->getSegmentIndex());

        //TODO : potential bug here: a vehicle entering the lane is too close.

        if (targetFollower)
            followerGap = vehicle->getDistance() - targetFollower->getDistance() - vehicle->getLen();
        else
            followerGap = std::numeric_limits<double>::max();
    }

    double LaneChange::gapBefore() const {
        return followerGap;
    }


    double LaneChange::gapAfter() const {
        return leaderGap;
    }

    void LaneChange::insertShadow(Vehicle *shadow) {

        assert(!changing);
        assert(vehicle->getOffset() == 0);
        changing = true;
        waitingTime = 0;

        assert(vehicle->getCurDrivable()->isLane());
        Lane *targetLane = signalSend->target;
        int  segId = vehicle->getSegmentIndex();
        auto targetSeg = targetLane->getSegment(segId);
        auto followerItr = (vehicle->getListIterator());
        followerItr++;

        shadow->setParent(vehicle);
        vehicle->setShadow(shadow);
        shadow->controllerInfo.blocker = nullptr;
        shadow->controllerInfo.drivable = targetLane;
        shadow->controllerInfo.router.update();

        auto targetFollowerItr = targetFollower ?  targetFollower->getListIterator() : targetLane->getVehicles().end();

        auto newItr = targetLane->getVehicles().insert(targetFollowerItr, shadow);

        targetSeg->insertVehicle(newItr);

        shadow->updateLeaderAndGap(targetLeader);
        if (targetFollower)
            targetFollower->updateLeaderAndGap(shadow);


    }

    int LaneChange::getDirection() {
        if (!vehicle->getCurDrivable()->isLane()) return 0;
        Lane *curLane  = dynamic_cast<Lane *>(vehicle->getCurDrivable());
        if (!signalSend) return 0;
        if (!signalSend->target) return 0;
        if (signalSend->target == curLane->getOuterLane()) return 1;
        if (signalSend->target == curLane->getInnerLane()) return -1;
        return 0;

    }

    void LaneChange::finishChanging() {
        changing = false;
        angle = 0;
        finished = true;
        lastChangeTime = vehicle->engine->getCurrentTime();
        Vehicle *partner = vehicle->getPartner();
        if (!partner->isReal())
            partner->setId(vehicle->getId());
        partner->laneChangeInfo.partnerType = 0;
        partner->laneChangeInfo.offset = 0;
        partner->laneChangeInfo.partner = nullptr;
        vehicle->laneChangeInfo.partner = nullptr;
        clearSignal();
    }

    void LaneChange::clearSignal() {
        targetLeader = nullptr;
        targetFollower = nullptr;
        if (signalSend)
            lastDir = signalSend->direction;
        else
            lastDir = 0;
        if (changing) return;
        signalSend = nullptr;
        signalRecv = nullptr;
        clearCustomDirection();
    }

    Lane *LaneChange::nearestAvailableLane() const {
        Lane *curLane = vehicle->getCurLane();
        Road *curRoad = curLane->getBelongRoad();
        const Router &router = vehicle->controllerInfo.router;
        int left, right, laneCnt = curRoad->getLanes().size();
        for (left = right = curLane->getLaneIndex() ; left >=0 || right < laneCnt ; --left, ++right){
            if (left >= 0) {
                Lane *leftLane = curRoad->getLanePointers()[left];
                if (router.getNextDrivable(leftLane))
                    return leftLane;
            }
            if (right < laneCnt) {
                Lane *rightLane = curRoad->getLanePointers()[right];
                if (router.getNextDrivable(rightLane))
                    return rightLane;
            }
        }
        return nullptr;
    }

    double LaneChange::safeDistance(Lane *lane) const {
        Lane *targetLane = nearestAvailableLane();
        assert(targetLane);
        int direction = targetLane->getLaneIndex() > lane->getLaneIndex() ? 1 : -1;
        double lcReplacement = 0;
        const auto &lanes = lane->getBelongRoad()->getLanePointers();
        for (size_t i = lane->getLaneIndex() ; i != targetLane->getLaneIndex() ; i += direction) {
            lcReplacement += (lanes[i]->getWidth() + lanes[i + direction]->getWidth()) / 2;
        }
        int dIndex = static_cast<int>(targetLane->getLaneIndex()) - static_cast<int>(lane->getLaneIndex());
        return lcReplacement / ratio + abs(dIndex) * vehicle->getMaxSpeed() * vehicle->engine->getInterval();
    }

    double LaneChange::safeDistance() const {
        return safeDistance(vehicle->getCurLane());
    }

    double LaneChange::forceLaneChangeSpeed() const {
        auto &router = vehicle->controllerInfo.router;
        if (!vehicle->getCurDrivable()->isLane() || router.onValidLane() || changing || !vehicle->isReal())
            return vehicle->getMaxSpeed();
        Lane *curLane = vehicle->getCurLane();
        double safeDistance = curLane->getLength() - this->safeDistance() - vehicle->getDistance();
        if (planChange() && !signalRecv) {
            if (targetFollower && targetFollower->getLaneChange().planChange() && targetFollower->getSpeed() == 0 &&
                vehicle->getDistance() - targetFollower->getDistance() < vehicle->getLen()) {
                if (safeDistance > 0)
//                    return vehicle->getStopBeforeSpeed(safeDistance, vehicle->engine->getInterval());
                    return vehicle->getMaxSpeed();
                else return -100;
            }
        }
        if (safeDistance -  vehicle->getLen() > 0)
            return vehicle->getStopBeforeSpeed(safeDistance -  vehicle->getLen(), vehicle->engine->getInterval());
        else return -100;
    }

    double LaneChange::getDeltaOffset(double deltaDis) const {
        double deltaOffset = ratio * deltaDis;
        double minimalDeltaOffset = minimalLaneChangeSpeed * vehicle->engine->getInterval();
        if (deltaOffset > 0) return deltaOffset;
        else {
            std::uniform_real_distribution<double> dis(0.0, 1.0);
            double chance = dis(vehicle->engine->rnd);
            if (chance < minimalSpeedChance)
                return minimalDeltaOffset;
            else
                return 0;
        }
    }

    double LaneChange::safeDistance(double deltaOffset) const {
        return deltaOffset / ratio + vehicle->getMaxSpeed() * vehicle->engine->getInterval();
    }

    void LaneChange::changeToInner() {
        if (vehicle->getDistance() + vehicle->getMinBrakeDistance() >= vehicle->getCurDrivable()->getLength()) return;
        signalSend = std::make_shared<Signal>();
        signalSend->source = vehicle;
        signalSend->target = vehicle->getCurLane()->getInnerLane();
        signalSend->direction = getDirection();
    }

    void LaneChange::changeToOuter() {
        if (vehicle->getDistance() + vehicle->getMinBrakeDistance() >= vehicle->getCurDrivable()->getLength()) return;
        signalSend = std::make_shared<Signal>();
        signalSend->source = vehicle;
        signalSend->target = vehicle->getCurLane()->getOuterLane();
        signalSend->direction = getDirection();
    }

    double LaneChange::laneChangeAngle() const {
        if (!changing) return 0;
        return angle;
    }

    bool LaneChange::isGapValid() const {
        return gapAfter() >= safeGapAfter() && gapBefore() >= safeGapBefore();
    }

    void LaneChange::updateAngle(double dx, double dy) {
        if (!changing) angle = 0;
        double maxAngle = atan2(0.6, 1);
        double p = fabs(vehicle->laneChangeInfo.offset) / vehicle->getMaxOffset();
        double angleByPos = (0.9 - 1.6 * min2double(fabs(p - 0.5), 0.5)) * maxAngle;
        double angleBySpeed = min2double(atan2(dx, dy), maxAngle);
        double newAngle = (dy < 0.1) ? angleByPos: min2double(angleByPos, angleBySpeed);

        angle = (angle - signalSend->direction * newAngle) / 2;
    }

    void LaneChange::makeSignal(double interval) {
        if (changing) return;

        if (vehicle->getCurDrivable()->isLane()) {
            Lane *curLane = (Lane *)vehicle->getCurDrivable();
            const Road *curRoad = curLane->getBelongRoad();
            if (isCustomDirectionSet) {
                if (customDirection == -1 && curLane->getLaneIndex() > 0) {
                    changeToInner();
                }else if (customDirection == 1 && curLane->getLaneIndex() < curRoad->getLanes().size() - 1) {
                    changeToOuter();
                }
                return ;
            }

            Router &router = vehicle->controllerInfo.router;
            if (!router.onValidLane() &&
            safeDistance() + 50 + vehicle->getLen() + vehicle->getDistance() >= vehicle->getCurLane()->getLength()) {
                // forced lane change
                // when the vehicle is on a wrong lane
                Lane *finalTarget = nearestAvailableLane();
                if (finalTarget->getLaneIndex() > curLane->getLaneIndex()) changeToOuter();
                else changeToInner();
            }else{
                // optional lane change

                if (vehicle->engine->getCurrentTime() - lastChangeTime < coolingTime) return;
                double outerWeight = std::numeric_limits<double>::lowest();
                double outerChance = laneChangeChance;
                double innerWeight = std::numeric_limits<double>::lowest();
                double innerChance = laneChangeChance;
                double curEst = vehicle->getLeader() ? vehicle->getGap() : curLane->getLength() - vehicle->getDistance()
                        - (vehicle->onValidLane() ? 0 : wrongLanePenalty);

                if (curLane->getLaneIndex() < curRoad->getLanes().size() - 1) {
                    Lane *outerLane = curLane->getOuterLane();
                    outerWeight = estimateGap(outerLane) - curEst - changeLanePenalty;
                    if (!router.onLastRoad() && !router.getNextDrivable(outerLane)) {
                        double safeDis = safeDistance(outerLane) +
                                         safeDistance((curLane->getWidth() + outerLane->getWidth()) / 2) +
                                         vehicle->getLen();
                        if (safeDis + vehicle->getDistance() > curLane->getLength())
                            outerWeight = std::numeric_limits<double>::lowest();
                        else {
                            outerWeight -= wrongLanePenalty;
                            outerChance = aggressiveChance;
                        }
                    }
                }

                if (curLane->getLaneIndex() > 0) {
                    Lane *innerLane = curLane->getInnerLane();
                    innerWeight = estimateGap(innerLane) - curEst - changeLanePenalty;
                    if (!router.onLastRoad() && !router.getNextDrivable(innerLane)) {
                        double safeDis = safeDistance(innerLane) +
                                         safeDistance((curLane->getWidth() + innerLane->getWidth()) / 2) +
                                         vehicle->getLen();
                        if (safeDis + vehicle->getDistance() > curLane->getLength())
                            innerWeight = std::numeric_limits<double>::lowest();
                        else {
                            innerWeight -= wrongLanePenalty;
                            innerChance = aggressiveChance;
                        }
                    }
                }
                if (innerWeight < 0 && outerWeight < 0) return;
                std::uniform_real_distribution<double> dis(0.0, 1.0);
                double chance = dis(vehicle->engine->rnd);
                if (innerWeight < outerWeight) {
                    if (chance <= outerChance) changeToOuter();
                } else
                    if (chance <= innerChance) changeToInner();
            }
        }
    }

    double LaneChange::yieldSpeed(double interval) {
        if (planChange()) waitingTime += interval;
        if (signalRecv) {
            Vehicle *source = signalRecv->source;
            double srcSpeed = source->getSpeed();
            double gap = source->laneChange.gapBefore() - source->laneChange.safeGapBefore() ;
            if (gap < 0) return vehicle->getMaxSpeed();
            // If the follower is too fast, let it go.

            return vehicle->getNoCollisionSpeed(srcSpeed, source->getMaxNegAcc(), vehicle->getSpeed(),
                                                vehicle->getMaxNegAcc(), gap, interval, vehicle->getMinGap());

        }
        return vehicle->getMaxSpeed();
    }

    void LaneChange::sendSignal() {
        if (targetFollower) {
            auto followers = signalSend->target->getVehiclesBeforeDistance(vehicle->getDistance(),
                    vehicle->getSegmentIndex(), notifyRange);
            for (auto &follower : followers){
                follower->receiveSignal(vehicle);
            }
        }
    }

    double LaneChange::safeGapBefore() const {
        return targetFollower ? targetFollower->getMinBrakeDistance() : 0;
    }

    double LaneChange::safeGapAfter() const {
        return vehicle->getMinBrakeDistance();
    }

    double LaneChange::estimateGap(const Lane *lane) const {
        int curSegIndex = vehicle->getSegmentIndex();
        Vehicle *leader = lane->getVehicleAfterDistance(vehicle->getDistance(), curSegIndex);
        if (!leader) return lane->getLength() - vehicle->getDistance();
        else return leader->getDistance() - vehicle->getDistance() - leader->getLen();
    }

}