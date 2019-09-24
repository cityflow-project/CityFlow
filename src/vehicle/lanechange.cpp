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
    }

    void LaneChange::abortChanging() {
        Vehicle *partner = vehicle->getPartner();
        partner->laneChange->changing = false;
        partner->laneChangeInfo.partnerType = 0;
        partner->laneChangeInfo.offset = 0;
        partner->laneChangeInfo.partner = nullptr;
        clearSignal();
    }


    void SimpleLaneChange::makeSignal(double interval) {
        if (changing) return;
        if (vehicle->engine->getCurrentTime() - lastChangeTime < coolingTime) return;
        signalSend = std::make_shared<Signal>();
        signalSend->source = vehicle;
        if (vehicle->getCurDrivable()->isLane()) {
            Lane *curLane = (Lane *)vehicle->getCurDrivable();

            if (curLane->getLength() - vehicle->getDistance() < 30) return;
            double curEst = vehicle->getGap();
            double outerEst = 0;
            double expectedGap = 2 * vehicle->getLen() + 4 * interval * vehicle->getMaxSpeed();
            if (vehicle->getGap() > expectedGap || vehicle->getGap() < 1.5 * vehicle->getLen()) return;

            Router &router = vehicle->controllerInfo.router;
            if (curLane->getLaneIndex() < curLane->getBelongRoad()->getLanes().size() - 1){
                if (router.onLastRoad() || router.getNextDrivable(curLane->getOuterLane())) {
                    outerEst = estimateGap(curLane->getOuterLane());
                    if (outerEst > curEst + vehicle->getLen())
                        signalSend->target = curLane->getOuterLane();
                }
            }

            if (curLane->getLaneIndex() > 0){
                if (router.onLastRoad() || router.getNextDrivable(curLane->getInnerLane())) {
                    double innerEst = estimateGap(curLane->getInnerLane());
                    if (innerEst > curEst + vehicle->getLen() && innerEst > outerEst)
                        signalSend->target = curLane->getInnerLane();
                }
            }
            signalSend->urgency = 1;
        }
        LaneChange::makeSignal(interval);
    }

    double SimpleLaneChange::yieldSpeed(double interval) {
        if (planChange()) waitingTime += interval;
        if (signalRecv) {
            if (vehicle == signalRecv->source->getTargetLeader()) {
                return 100;
            } else {
                Vehicle *source = signalRecv->source;
                double srcSpeed = source->getSpeed();
                double gap = source->laneChange->gapBefore() - source->laneChange->safeGapBefore();

                double v = vehicle->getNoCollisionSpeed(srcSpeed, source->getMaxNegAcc(), vehicle->getSpeed(),
                                                        vehicle->getMaxNegAcc(), gap, interval, 0);

                if (v < 0) v = 100;
                // If the follower is too fast, let it go.

                return v;
            }
        }
        return 100;
    }

    void SimpleLaneChange::sendSignal() {
        if (targetLeader) targetLeader->receiveSignal(vehicle);
        if (targetFollower) targetFollower->receiveSignal(vehicle);
    }

    double SimpleLaneChange::safeGapBefore() const {
        return targetFollower ? targetFollower->getMinBrakeDistance() : 0;
    }

    double SimpleLaneChange::safeGapAfter() const {
        return vehicle->getMinBrakeDistance();
    }

    double SimpleLaneChange::estimateGap(const Lane *lane) const {
        int curSegIndex = vehicle->getSegmentIndex();
        Vehicle *leader = lane->getVehicleAfterDistance(vehicle->getDistance(), curSegIndex);
        if (!leader) return lane->getLength()-vehicle->getDistance();
        else return leader->getDistance() - vehicle->getDistance() - leader->getLen();
    }

}