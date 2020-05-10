#ifndef CITYFLOW_ROUTER
#define CITYFLOW_ROUTER

#include "engine/archive.h"

#include <vector>
#include <random>
#include <memory>

namespace CityFlow {
    class Road;
    class Drivable;
    class Route;
    class Lane;
    class LaneLink;
    class Vehicle;

    class Router {
    friend Archive;
    private:
        
        Vehicle* vehicle = nullptr;
        std::vector<Road *> route;
        std::vector<Road *> anchorPoints;
        std::vector<Road *>::const_iterator iCurRoad;
        std::mt19937 *rnd = nullptr;

        mutable std::deque<Drivable *> planned;
        
        int selectLaneIndex(const Lane *curLane, const std::vector<Lane *> &lanes) const;

        LaneLink *selectLaneLink(const Lane *curLane, const std::vector<LaneLink*> &laneLinks) const;

        Lane *selectLane(const Lane *curLane, const std::vector<Lane *> &lanes) const;

        enum class RouterType{
            LENGTH,
            DURATION,
            DYNAMIC // TODO: dynamic routing
        };

        RouterType type = RouterType::LENGTH;

    public:

        Router(const Router &other);

        Router(Vehicle *vehicle, std::shared_ptr<const Route> route, std::mt19937 *rnd);

        Road *getFirstRoad() {
            return anchorPoints[0];
        }

        Drivable *getFirstDrivable() const;

        Drivable *getNextDrivable(size_t i = 0) const;

        Drivable *getNextDrivable(const Drivable *curDrivable) const;

        void update();

        bool isLastRoad(const Drivable *drivable) const;

        bool onLastRoad() const;

        bool onValidLane()  const{
            return !(getNextDrivable() == nullptr && !onLastRoad());
        }

        Lane * getValidLane(const Lane *curLane) const;

        void setVehicle(Vehicle *vehicle) {
            this->vehicle = vehicle;
        }

        bool dijkstra(Road *start, Road *end, std::vector<Road *> &buffer);

        bool updateShortestPath();

        bool setRoute(const std::vector<Road *> &anchor);

        std::vector<Road *> getFollowingRoads() const;
    };
}

#endif