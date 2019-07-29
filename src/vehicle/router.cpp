#include "vehicle/router.h"
#include "vehicle/vehicle.h"
#include "utility/utility.h"
#include "flow/route.h"
#include "roadnet/roadnet.h"
#include <random>
#include <cassert>
#include <limits>
#include <cstdlib>

namespace CityFlow {

    Router::Router(const Router &other) : vehicle(other.vehicle), route(other.route),
                                          rnd(other.rnd) {
        iCurRoad = route.begin();
    }

    Router::Router(Vehicle *vehicle, std::shared_ptr<const Route> route, std::mt19937 *rnd)
        : vehicle(vehicle), route(route->getRoute()), rnd(rnd) {
        assert(this->route.size() > 0);
        iCurRoad = this->route.begin();
    }

    Drivable *Router::getFirstDrivable() const {
        const std::vector<Lane *> &lanes = route[0]->getLanePointers();
        if (route.size() == 1) {
            return selectLane(nullptr, lanes);
        } else {
            std::vector<Lane *> candidateLanes;
            for (auto lane : lanes) {
                if (lane->getLaneLinksToRoad(route[1]).size() > 0) {
                    candidateLanes.push_back(lane);
                }
            }
            assert(candidateLanes.size() > 0);
            return selectLane(nullptr, candidateLanes);
        }
    }

    Drivable *Router::getNextDrivable(int i) const {
        if (i < planned.size() && i >= 0) {
            return planned[i];
        } else {
            Drivable *ret = getNextDrivable(planned.size() ? planned.back() : vehicle->getCurDrivable());
            planned.push_back(ret);
            return ret;
        }
    }

    Drivable *Router::getNextDrivable(const Drivable *curDrivable) const {
        if (curDrivable->isLaneLink()) {
            return static_cast<const LaneLink*>(curDrivable)->getEndLane();
        } else {
            const Lane *curLane = static_cast<const Lane *>(curDrivable);
            auto tmpCurRoad = iCurRoad;
            while ((*tmpCurRoad) != curLane->getBelongRoad() && tmpCurRoad != route.end()) {
                tmpCurRoad++;
            }
            assert(tmpCurRoad != route.end() && curLane->getBelongRoad() == (*tmpCurRoad));
            if (tmpCurRoad == route.end() - 1) {
                return nullptr;
            } else if (tmpCurRoad == route.end() - 2) {
                std::vector<LaneLink *> laneLinks = curLane->getLaneLinksToRoad(*(tmpCurRoad+1));
                return selectLaneLink(curLane, laneLinks);
            } else {
                std::vector<LaneLink *> laneLinks = curLane->getLaneLinksToRoad(*(tmpCurRoad+1));
                std::vector<LaneLink *> candidateLaneLinks;
                for (auto laneLink : laneLinks) {
                    Lane *nextLane = laneLink->getEndLane();
                    if (nextLane->getLaneLinksToRoad(*(tmpCurRoad+2)).size()) {
                        candidateLaneLinks.push_back(laneLink);
                    }
                }
                return selectLaneLink(curLane, candidateLaneLinks);
            }
        }
    }

    void Router::update() {
        const Drivable *curDrivable = vehicle->getCurDrivable();
        if (curDrivable->isLane()) {
            while (iCurRoad < route.end() && static_cast<const Lane*>(curDrivable)->getBelongRoad() != (*iCurRoad)) {
                iCurRoad++;
            }
            assert(iCurRoad < route.end());
        }
        for (auto it = planned.begin(); it != planned.end();) {
            if ((*it) != curDrivable) {
                it = planned.erase(it);
            } else {
                it = planned.erase(it);
                break;
            }
        }
    }

    int Router::selectLaneIndex(const Lane *curLane, const std::vector<Lane *> &lanes) const {
        assert(lanes.size() > 0);
        if (curLane == nullptr) {
            return (*rnd)() % lanes.size();
        }
        int laneDiff = std::numeric_limits<int>::max();
        int selected = -1;
        for (int i = 0;i < lanes.size(); ++i) {
            if (std::abs(lanes[i]->getLaneIndex() - curLane->getLaneIndex()) < laneDiff) {
                laneDiff = std::abs(lanes[i]->getLaneIndex() - curLane->getLaneIndex());
                selected = i;
            }
        }
        return selected;
    }

    Lane *Router::selectLane(const Lane *curLane, const std::vector<Lane *> &lanes) const {
        if (lanes.size() == 0) {
            return nullptr;
        }
        return lanes[selectLaneIndex(curLane, lanes)];
    }

    LaneLink *Router::selectLaneLink(const Lane *curLane, const std::vector<LaneLink*> &laneLinks) const {
        if (laneLinks.size() == 0) {
            return nullptr;
        }
        std::vector<Lane *> lanes;
        for (auto laneLink : laneLinks) {
            lanes.push_back(laneLink->getEndLane());
        }
        return laneLinks[selectLaneIndex(curLane, lanes)];
    }

    bool Router::isLastRoad(const Drivable *drivable) const {
        if (drivable->isLaneLink()) return false;
        return static_cast<const Lane*>(drivable)->getBelongRoad() == route.back();
    }

    bool Router::onLastRoad() const {
        return isLastRoad(vehicle->getCurDrivable());
    }

    Lane *Router::getValidLane(const Lane *curLane)  const{
        if (isLastRoad(curLane)) return nullptr;
        auto nextRoad = iCurRoad;
        nextRoad++;

        int min_diff = curLane->getBelongRoad()->getLanes().size();
        Lane * chosen = nullptr;
        for (auto lane : curLane->getBelongRoad()->getLanePointers()){
            if (lane->getLaneLinksToRoad(*nextRoad).size() > 0 &&
            abs(lane->getLaneIndex() - curLane->getLaneIndex()) < min_diff){
                min_diff = abs(lane->getLaneIndex() - curLane->getLaneIndex());
                chosen = lane;
            }
        }
        assert(chosen->getBelongRoad() == curLane->getBelongRoad());
        return chosen;
    }


}
