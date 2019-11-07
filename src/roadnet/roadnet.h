#ifndef CITYFLOW_ROADNET_H
#define CITYFLOW_ROADNET_H

#include "roadnet/trafficlight.h"
#include "utility/utility.h"

#include <list>
#include <map>
#include <queue>
#include <iostream>

namespace CityFlow {
    class RoadNet;

    class Intersection;

    class Road;

    class Lane;

    class LaneLink;

    class Vehicle;

    class Cross;

    class Segment {
        friend Lane;
    public:
        //Vehicle * tryChange = nullptr;

        Segment() = default;

        Segment(size_t index, Lane *belongLane, double startPos, double endPos) : index(index), belongLane(belongLane),
                                                                                  startPos(startPos), endPos(endPos) { }

        double getStartPos() const { return this->startPos; }

        double getEndPos() const { return this->endPos; }

        size_t getIndex() const { return this->index; }

        const std::list<std::list<Vehicle *>::iterator> &getVehicles() const { return this->vehicles; }

        std::list<std::list<Vehicle *>::iterator> &getVehicles() { return this->vehicles; }

//        std::list<Vehicle *>::iterator getPrevVehicleIter() const { return this->prev_vehicle_iter; }

        std::list<Vehicle *>::iterator findVehicle(Vehicle * vehicle);

        void removeVehicle(Vehicle * vehicle);

        void insertVehicle(std::list<Vehicle *> ::iterator &vehicle);

    private:
        size_t index = 0;
        Lane *belongLane = nullptr;
        double startPos = 0;
        double endPos = 0;
        std::list<std::list<Vehicle *>::iterator> vehicles;
        std::list<Vehicle *>::iterator prev_vehicle_iter;
    };

    class Intersection {
        friend class RoadNet;

        friend class RoadLink;

        friend class Road;

        friend class TrafficLight;

    private:
        std::string id;
        bool isVirtual;
        double width = 0.0;
        Point point;
        TrafficLight trafficLight;
        std::vector<Road *> roads;
        std::vector<RoadLink> roadLinks;
        std::vector<Cross> crosses;
        std::vector<LaneLink *> laneLinks;

        void initCrosses();

    public:
        std::string getId() const { return this->id; }

        const TrafficLight &getTrafficLight() const { return trafficLight; }

        TrafficLight &getTrafficLight() { return trafficLight; }

        const std::vector<Road *> &getRoads() const { return this->roads; }

        std::vector<Road *> &getRoads() { return this->roads; }

        const std::vector<RoadLink> &getRoadLinks() const { return this->roadLinks; }

        std::vector<RoadLink> &getRoadLinks() { return this->roadLinks; }

        std::vector<Cross> &getCrosses() { return crosses; }

        bool isVirtualIntersection() const { return this->isVirtual; }

        const std::vector<LaneLink *> &getLaneLinks();

        void reset();

        std::vector<Point> getOutline();

        bool isImplicitIntersection();

        const Point &getPosition() const { return point; }
    };

    class Cross {
        friend class RoadNet;

        friend class Intersection;

    private:
        LaneLink *laneLinks[2];
        Vehicle *notifyVehicles[2];
        double notifyDistances[2];
        double distanceOnLane[2];
        double leaveDistance = 0, arriveDistance = 30; // TODO
        double ang;
        double safeDistances[2];

    public:
        double getLeaveDistance() const { return leaveDistance; }

        double getArriveDistance() const { return arriveDistance; }

        void notify(LaneLink *laneLink, Vehicle *vehicle, double notifyDistance);

        bool canPass(const Vehicle *vehicle, const LaneLink *laneLink,
                     double distanceToLaneLinkStart) const; // XXX: change to LaneLink based?

        void clearNotify() { notifyVehicles[0] = notifyVehicles[1] = nullptr; }

        Vehicle *getFoeVehicle(const LaneLink *laneLink) const {
            assert(laneLink == laneLinks[0] || laneLink == laneLinks[1]);
            return laneLink == laneLinks[0] ? notifyVehicles[1] : notifyVehicles[0];
        }

        double getDistanceByLane(const LaneLink *laneLink) const {
            // XXX: lanelink not in cross?
            assert(laneLink == laneLinks[0] || laneLink == laneLinks[1]);
            return laneLink == laneLinks[0] ? distanceOnLane[0] : distanceOnLane[1];
        }

        double getNotifyDistanceByLane(LaneLink *laneLink) const {
            // XXX: lanelink not in cross?
            assert(laneLink == laneLinks[0] || laneLink == laneLinks[1]);
            return laneLink == laneLinks[0] ? notifyDistances[0] : notifyDistances[1];
        }

        double getSafeDistanceByLane(LaneLink *laneLink) {
            assert(laneLink == laneLinks[0] || laneLink == laneLinks[1]);
            return laneLink == laneLinks[0] ? safeDistances[0] : safeDistances[1];
        }

        double getAng() const {
            return ang;
        }

        LaneLink *getLaneLink(int i) const { return laneLinks[i]; }

        void reset();
    };

    class Road {
        friend class RoadNet;

        friend class Lane;

    private:
        std::string id;
        Intersection *startIntersection = nullptr;
        Intersection *endIntersection = nullptr;
        std::vector<Lane> lanes;
        std::vector<Point> points;

        std::vector<Lane *> lanePointers;

        std::vector<Vehicle *> planRouteBuffer;

        void initLanesPoints();

    public:
        std::string getId() const { return id; }

        const Intersection &getStartIntersection() const { return *(this->startIntersection); }

        const Intersection &getEndIntersection() const { return *(this->endIntersection); }

        const std::vector<Lane> &getLanes() const { return lanes; }

        std::vector<Lane> &getLanes() { return lanes; }

        const std::vector<Lane *> &getLanePointers();

        void buildSegmentationByInterval(double interval);

        bool connectedToRoad(const Road * road) const;

        void reset();

        double getWidth() const;

        double getLength() const;

        double averageLength() const;

        double getAverageSpeed() const;

        double getAverageDuration() const;

        const std::vector<Vehicle *> &getPlanRouteBuffer() const {
            return planRouteBuffer;
        }

        void addPlanRouteVehicle(Vehicle *vehicle) {
            planRouteBuffer.emplace_back(vehicle);
        }

        void clearPlanRouteBuffer() {
            planRouteBuffer.clear();
        }
    };

    class Drivable {
        friend class RoadNet;
        friend class Archive;
    public:
        enum DrivableType {
            LANE = 0, LANELINK = 1
        };

    protected:
        double length;
        double width;
        double maxSpeed;
        std::list<Vehicle *> vehicles;
        std::vector<Point> points;
        DrivableType drivableType;

    public:
        virtual ~Drivable() = default;

        const std::list<Vehicle *> &getVehicles() const { return vehicles; }

        std::list<Vehicle *> &getVehicles() { return vehicles; }

        double getLength() const { return length; }

        double getWidth() const { return width; }

        double getMaxSpeed() const { return maxSpeed; }

        size_t getVehicleCount() const { return vehicles.size(); }

        DrivableType getDrivableType() const { return drivableType; }

        bool isLane() const { return drivableType == LANE; }

        bool isLaneLink() const { return drivableType == LANELINK; }

        Vehicle *getFirstVehicle() const {
            if (!vehicles.empty()) return vehicles.front();
            return nullptr;
        }

        Vehicle *getLastVehicle() const {
            if (!vehicles.empty()) return vehicles.back();
            return nullptr;
        }

        Point getPointByDistance(double dis) const;

        Point getDirectionByDistance(double dis) const;

        void pushVehicle(Vehicle *vehicle) {
            vehicles.push_back(vehicle);
        }

        void popVehicle() { vehicles.pop_front(); }

        virtual std::string getId() const = 0;
    };

    class Lane : public Drivable {

        friend class RoadNet;
        friend class Road;
        friend class Archive;

    private:
        int laneIndex;
        std::vector<Segment> segments;
        std::vector<LaneLink *> laneLinks;
        Road *belongRoad = nullptr;
        std::deque<Vehicle *> waitingBuffer;

        struct HistoryRecord {
            int vehicleNum;
            double averageSpeed;
            HistoryRecord(int vehicleNum, double averageSpeed) : vehicleNum(vehicleNum), averageSpeed(averageSpeed) {}
        };
        std::list<HistoryRecord> history;

        int    historyVehicleNum = 0;
        double historyAverageSpeed = 0;

        static constexpr int historyLen = 240;

    public:
        Lane();

        Lane(double width, double maxSpeed, int laneIndex, Road *belongRoad);

        std::string getId() const override{
            return belongRoad->getId() + '_' + std::to_string(getLaneIndex());
        }

        Road *getBelongRoad() const { return this->belongRoad; }

        bool available(const Vehicle *vehicle) const;

        bool canEnter(const Vehicle *vehicle) const;

        size_t getLaneIndex() const { return this->laneIndex; }

        Lane *getInnerLane() const {
            return laneIndex > 0 ? &(belongRoad->lanes[laneIndex - 1]) : nullptr;
        }

        Lane *getOuterLane() const {
            int lane_num = belongRoad->lanes.size();
            return laneIndex < lane_num - 1? &(belongRoad->lanes[laneIndex + 1]) : nullptr;
        }

        const std::vector<LaneLink *> &getLaneLinks() const { return this->laneLinks; }

        std::vector<LaneLink *> &getLaneLinks() { return this->laneLinks; }

        Intersection *getStartIntersection() const {
            return belongRoad->startIntersection;
        }

        Intersection *getEndIntersection() const {
            return belongRoad->endIntersection;
        }

        std::vector<LaneLink *> getLaneLinksToRoad(const Road *road) const;

        void reset();

        /* waiting buffer */
        const std::deque<Vehicle *> &getWaitingBuffer() const { return waitingBuffer; }

        std::deque<Vehicle *> &getWaitingBuffer() { return waitingBuffer; }

        void pushWaitingVehicle(Vehicle *vehicle) {
            waitingBuffer.emplace_back(vehicle);
        }

        /* segmentation */
        void buildSegmentation(size_t numSegs);

        void initSegments();

        const Segment *getSegment(size_t index) const { return &segments[index]; }

        Segment *getSegment(size_t index) { return &segments[index]; }

        const std::vector<Segment> &getSegments() const { return segments; }

        std::vector<Segment> &getSegments() { return segments; }

        size_t getSegmentNum() const{ return segments.size(); }

        std::vector<Vehicle*> getVehiclesBeforeDistance(double dis, size_t segmentIndex, double deltaDis = 50);

        /* history */
        void updateHistory();

        int getHistoryVehicleNum() const;

        double getHistoryAverageSpeed() const;


        Vehicle* getVehicleBeforeDistance(double dis, size_t segmentIndex) const; //TODO: set a limit, not too far way

        Vehicle* getVehicleAfterDistance(double dis, size_t segmentIndex) const;

    };


    enum RoadLinkType {
        go_straight = 3, turn_left = 2, turn_right = 1
    };

    class RoadLink {
        friend class RoadNet;

        friend class LaneLink;

    private:
        Intersection *intersection = nullptr;
        Road *startRoad = nullptr;
        Road *endRoad = nullptr;
        RoadLinkType type;
        std::vector<LaneLink> laneLinks;
        std::vector<LaneLink *> laneLinkPointers;
        int index;
    public:
        const std::vector<LaneLink> &getLaneLinks() const { return this->laneLinks; }

        std::vector<LaneLink> &getLaneLinks() { return this->laneLinks; }

        const std::vector<LaneLink *> &getLaneLinkPointers();

        Road *getStartRoad() const { return this->startRoad; }

        Road *getEndRoad() const { return this->endRoad; }

        bool isAvailable() const {
            return this->intersection->trafficLight.getCurrentPhase().roadLinkAvailable[this->index];
        }

        bool isTurn() const {
            return type == turn_left || type == turn_right;
        }

        void reset();

    };

    class LaneLink : public Drivable {

        friend class RoadNet;

        friend class Intersection;

    private:
        RoadLink *roadLink = nullptr;
        Lane *startLane = nullptr;
        Lane *endLane = nullptr;
        std::vector<Cross *> crosses;

    public:
        LaneLink() {
            width = 4;
            maxSpeed = 10000; //TODO
            drivableType = LANELINK;
        }

        RoadLink *getRoadLink() const { return this->roadLink; }

        RoadLinkType getRoadLinkType() const { return this->roadLink->type; }

        const std::vector<Cross *> &getCrosses() const { return this->crosses; }

        std::vector<Cross *> &getCrosses() { return this->crosses; }

        Lane *getStartLane() const { return startLane; }

        Lane *getEndLane() const { return endLane; }

        bool isAvailable() const { return roadLink->isAvailable(); }

        bool isTurn() const { return roadLink->isTurn(); }

        void reset();

        std::string getId() const override {
            return (startLane ? startLane->getId() : "") + "_TO_" + (endLane ? endLane->getId() : "");
        }
    };


    class RoadNet {
    private:
        std::vector<Road> roads;
        std::vector<Intersection> intersections;
        std::map<std::string, Road *> roadMap;
        std::map<std::string, Intersection *> interMap;
        std::map<std::string, Drivable *> drivableMap;

        std::vector<Lane *> lanes;
        std::vector<LaneLink *> laneLinks;
        std::vector<Drivable *> drivables;
        Point getPoint(const Point &p1, const Point &p2, double a);

    public:
        bool loadFromJson(std::string jsonFileName);

        rapidjson::Value convertToJson(rapidjson::Document::AllocatorType &allocator);

        const std::vector<Road> &getRoads() const { return this->roads; }

        std::vector<Road> &getRoads() { return this->roads; }

        const std::vector<Intersection> &getIntersections() const { return this->intersections; }

        std::vector<Intersection> &getIntersections() { return this->intersections; }

        Road *getRoadById(const std::string &id) const {
            return roadMap.count(id) > 0 ? roadMap.at(id) : nullptr;
        }

        Intersection *getIntersectionById(const std::string &id) const {
            return interMap.count(id) > 0 ? interMap.at(id) : nullptr;
        }

        Drivable *getDrivableById(const std::string &id) const {
            return drivableMap.count(id) > 0 ? drivableMap.at(id) : nullptr;
        }

        const std::vector<Lane *> &getLanes() const {
            return lanes;
        }

        const std::vector<LaneLink *> &getLaneLinks() const {
            return laneLinks;
        }

        const std::vector<Drivable *> &getDrivables() const {
            return drivables;
        }

        void reset();
    };
}


#endif //CITYFLOW_ROADNET_H
