#ifndef CITYFLOW_ROUTER
#define CITYFLOW_ROUTER

#include <vector>
#include <random>
#include <memory>
#include <deque>

namespace CityFlow {
    class Road;
	class Drivable;
	class Route;
	class Lane;
	class LaneLink;
    class Vehicle;

	class Router {

    private:
        
        const Vehicle* vehicle = nullptr;
        const std::vector<Road *> route;
        std::vector<Road *>::const_iterator iCurRoad;
        std::mt19937 *rnd = nullptr;

        mutable std::deque<Drivable *> planned;
        
        int selectLaneIndex(const Lane *curLane, const std::vector<Lane *> &lanes) const;
	
        LaneLink *selectLaneLink(const Lane *curLane, const std::vector<LaneLink*> &laneLinks) const;

        Lane *selectLane(const Lane *curLane, const std::vector<Lane *> &lanes) const;
	
    public:

        Router(const Vehicle *vehicle, std::shared_ptr<const Route> route, std::mt19937 *rnd);
		
        Drivable *getFirstDrivable() const;

        Drivable *getNextDrivable(int i = 0) const;

        Drivable *getNextDrivable(const Drivable *curDrivable) const;
	
		void update();

        bool isLastRoad(const Drivable *drivable) const;

        bool onLastRoad() const;

        bool onValidLane() {
            return !(getNextDrivable() == nullptr && !onLastRoad());
        }
    };
}

#endif