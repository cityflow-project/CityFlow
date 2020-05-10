#include "vehicle/router.h"
#include "vehicle/vehicle.h"
#include "flow/route.h"
#include "roadnet/roadnet.h"

#include <limits>
#include <queue>
#include <set>

namespace CityFlow {
    Router::Router(const Router &other) : vehicle(other.vehicle), route(other.route), anchorPoints(other.anchorPoints),
                                          rnd(other.rnd) {
        iCurRoad = this->route.begin();
    }

    Router::Router(Vehicle *vehicle, std::shared_ptr<const Route> route, std::mt19937 *rnd)
        : vehicle(vehicle), anchorPoints(route->getRoute()), rnd(rnd) {
        assert(this->anchorPoints.size() > 0);
        this->route = route->getRoute();
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

    Drivable *Router::getNextDrivable(size_t i) const {
        if (i < planned.size()) {
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
        for (size_t i = 0 ; i < lanes.size() ; ++i) {
            int curLaneDiff = lanes[i]->getLaneIndex() - curLane->getLaneIndex();
            if (abs(curLaneDiff) < laneDiff) {
                laneDiff = abs(curLaneDiff);
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

    Lane *Router::getValidLane(const Lane *curLane)  const {
        if (isLastRoad(curLane)) return nullptr;
        auto nextRoad = iCurRoad;
        nextRoad++;

        int min_diff = curLane->getBelongRoad()->getLanes().size();
        Lane * chosen = nullptr;
        for (auto lane : curLane->getBelongRoad()->getLanePointers()){
            int curLaneDiff = lane->getLaneIndex() - curLane->getLaneIndex();
            if (lane->getLaneLinksToRoad(*nextRoad).size() > 0 &&
            abs(curLaneDiff) < min_diff){
                min_diff = abs(curLaneDiff);
                chosen = lane;
            }
        }
        assert(chosen->getBelongRoad() == curLane->getBelongRoad());
        return chosen;
    }


    bool Router::dijkstra(Road *start, Road *end, std::vector<Road *> &buffer) {
        std::map<Road *, double> dis;
        std::map<Road *, Road *> from;
        std::set<Road *>         visited;
        bool success = false;
        using pair = std::pair<Road *, double>;

        auto cmp = [](const pair &a, const pair &b){ return a.second > b.second; };

        std::priority_queue<pair, std::vector<pair>,decltype(cmp) > queue(cmp);

        dis[start] = 0;
        queue.push(std::make_pair(start, 0));
        while (!queue.empty()) {
            auto curRoad = queue.top().first;
            if (curRoad == end) {
                success = true;
                break;
            }
            queue.pop();
            if (visited.count(curRoad)) continue;
            visited.insert(curRoad);
            double curDis = dis.find(curRoad)->second;
            dis[curRoad] = curDis;
            for (const auto &adjRoad : curRoad->getEndIntersection().getRoads()) {
                if (!curRoad->connectedToRoad(adjRoad)) continue;
                auto iter = dis.find(adjRoad);
                double newDis;

                switch (type) {
                    case RouterType::LENGTH:
                        newDis = curDis + adjRoad->averageLength();
                        break;
                    case RouterType::DURATION: {
                        double avgDur;
                        avgDur = adjRoad->getAverageDuration();
                        if (avgDur < 0) {
                            avgDur = adjRoad->getLength() / vehicle->getMaxSpeed();
                        }
                        newDis = curDis + avgDur;
                    }
                        break;
                    default:
                        assert(false); // under construction
                        break;
                }

                if (iter == dis.end() || newDis < iter->second) {
                    from[adjRoad] = curRoad;
                    dis[adjRoad]  = newDis;
                    queue.emplace(std::make_pair(adjRoad, newDis));
                }
            }
        }

        std::vector<Road *> path;
        path.push_back(end);

        auto iter = from.find(end);
        while (iter != from.end() && iter->second != start) {
            path.emplace_back(iter->second);
            iter = from.find(iter->second);
        }

        buffer.insert(buffer.end(), path.rbegin(), path.rend());
        return success;
    }

    bool Router::updateShortestPath() {
        //Dijkstra
        planned.clear();
        route.clear();
        route.push_back(anchorPoints[0]);
        for (size_t i = 1 ; i < anchorPoints.size() ; ++i){
            if (anchorPoints[i - 1] == anchorPoints[i])
                continue;
            if (!dijkstra(anchorPoints[i - 1], anchorPoints[i], route))
                return false;
        }
        if (route.size() <= 1)
            return false;
        iCurRoad = this->route.begin();
        return true;
    }

    bool Router::setRoute(const std::vector<Road *> &anchor) {
        if (vehicle->getCurDrivable()->isLaneLink()) return false;
        Road *cur_road = *iCurRoad;
        auto backup = std::move(anchorPoints);
        auto backup_route = std::move(route);
        anchorPoints.clear();
        anchorPoints.emplace_back(cur_road);
        anchorPoints.insert(anchorPoints.end(), anchor.begin(), anchor.end());
        bool result = updateShortestPath();
        if (result && onValidLane()) {
            return true;
        } else {
            anchorPoints = std::move(backup);
            route = std::move(backup_route);
            planned.clear();
            iCurRoad = route.begin();
            for (iCurRoad = route.begin(); *iCurRoad != cur_road && iCurRoad != route.end(); ++iCurRoad);
            return false;
        }
    }

    std::vector<Road *> Router::getFollowingRoads() const {
        std::vector<Road *> ret;
        ret.insert(ret.end(), iCurRoad, route.end());
        return ret;
    }

}
