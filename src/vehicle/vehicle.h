#ifndef CITYFLOW_VEHICLE
#define CITYFLOW_VEHICLE

#include "utility/utility.h"
#include "flow/route.h"
#include "vehicle/router.h"
#include "vehicle/lanechange.h"

#include <utility>
#include <memory>

namespace CityFlow {
    class Lane;

    class LaneLink;

    class Intersection;

    class Route;

    class Cross;

    class Drivable;

    class Engine;

    class Point;

    class Flow;

    struct VehicleInfo {
        double speed = 0;
        double len = 5;
        double width = 2;
        double maxPosAcc = 4.5;
        double maxNegAcc = 4.5;
        double usualPosAcc = 2.5;
        double usualNegAcc = 2.5;
        double minGap = 2;
        double maxSpeed = 16.66667;
        double headwayTime = 1;
        double yieldDistance = 5;
        double turnSpeed = 8.3333;
        std::shared_ptr<const Route> route = nullptr;
    };


    class Vehicle {
        friend class Router;
        friend class LaneChange;
        friend class SimpleLaneChange;
        friend class Archive;
    private:
        struct Buffer {
            bool isDisSet = false;
            bool isSpeedSet = false;
            bool isDrivableSet = false;
            bool isNotifiedVehicles = false;
            bool isEndSet = false;
            bool isEnterLaneLinkTimeSet = false;
            bool isBlockerSet = false;
            bool isCustomSpeedSet = false;
            double dis;
            double deltaDis;
            double speed;
            double customSpeed;
            Drivable *drivable;
            std::vector<Vehicle *> notifiedVehicles;
            bool end;
            Vehicle *blocker = nullptr;
            size_t enterLaneLinkTime;
        };

        struct LaneChangeInfo {
            short partnerType = 0; //0 for no partner; 1 for real vehicle; 2 for shadow vehicle;
            Vehicle *partner = nullptr;
            double offset = 0;
            size_t segmentIndex = 0;
        };

        struct ControllerInfo {
            double dis = 0;
            Drivable *drivable = nullptr;
            Drivable *prevDrivable = nullptr;
            double approachingIntersectionDistance;
            double gap;
            size_t enterLaneLinkTime;
            Vehicle *leader = nullptr;
            Vehicle *blocker = nullptr;
            bool end = false;
            bool running = false;
            Router router;
            ControllerInfo(Vehicle *vehicle, std::shared_ptr<const Route> route, std::mt19937 *rnd);
            ControllerInfo(Vehicle *vehicle, const ControllerInfo &other);
        };

        VehicleInfo vehicleInfo;
        ControllerInfo controllerInfo;
        LaneChangeInfo laneChangeInfo;

        Buffer buffer;

        int priority;
        std::string id;
        double enterTime;

        Engine *engine;

        std::shared_ptr<LaneChange> laneChange;

        bool routeValid = false;
        Flow *flow;

    public:

        bool isStraightHold = false;

        Vehicle(const Vehicle &vehicle, Flow *flow = nullptr);

        Vehicle(const Vehicle &vehicle, const std::string &id, Engine *engine, Flow *flow = nullptr);

        Vehicle(const VehicleInfo &init, const std::string &id, Engine *engine, Flow *flow = nullptr);

        void setDeltaDistance(double dis);

        void setSpeed(double speed);

        void setCustomSpeed(double speed) {
            buffer.customSpeed = speed;
            buffer.isCustomSpeedSet = true;
        }

        void setDis(double dis) {
            buffer.dis = dis;
            buffer.isDisSet = true;
        }

        void setDrivable(Drivable *drivable) {
            buffer.drivable = drivable;
            buffer.isDrivableSet = true;
        }

        bool hasSetDrivable() const { return buffer.isDrivableSet; }

        bool hasSetSpeed() const { return buffer.isSpeedSet; }

        bool hasSetCustomSpeed() const { return buffer.isCustomSpeedSet; }

        double getBufferSpeed() const { return buffer.speed; };

        bool hasSetEnd() const { return buffer.isEndSet; }

        void setEnd(bool end) {
            buffer.end = end;
            buffer.isEndSet = true;
        }

        void unSetEnd() { buffer.isEndSet = false; }

        void unSetDrivable() { buffer.isDrivableSet = false; }

        void setEnterLaneLinkTime(size_t enterLaneLinkTime) {
            buffer.enterLaneLinkTime = enterLaneLinkTime;
            buffer.isEnterLaneLinkTimeSet = true;
        }

        void setBlocker(Vehicle *blocker) {
            buffer.blocker = blocker;
            buffer.isBlockerSet = true;
        }

        double getBufferDis() const { return buffer.dis; }

        void update();

        void setPriority(int priority) { this->priority = priority; }

        inline std::string getId() const { return id; }

        inline double getSpeed() const { return vehicleInfo.speed; }

        inline double getLen() const { return vehicleInfo.len; }

        inline double getWidth() const { return vehicleInfo.width; }

        inline double getDistance() const { return controllerInfo.dis; }

        Point getPoint() const;

        inline double getMaxPosAcc() const { return vehicleInfo.maxPosAcc; }

        inline double getMaxNegAcc() const { return vehicleInfo.maxNegAcc; }

        inline double getUsualPosAcc() const { return vehicleInfo.usualPosAcc; }

        inline double getUsualNegAcc() const { return vehicleInfo.usualNegAcc; }

        inline double getMinGap() const { return vehicleInfo.minGap; }

        inline double getYieldDistance() const { return vehicleInfo.yieldDistance; }

        inline double getTurnSpeed() const { return vehicleInfo.turnSpeed; }

        inline Vehicle *getBlocker() const { return controllerInfo.blocker; }

        inline Vehicle *getBufferBlocker() { return buffer.blocker; }

        Drivable *getCurDrivable() const;

        Lane * getCurLane() const {
            if (getCurDrivable()->isLane()) return (Lane *)getCurDrivable();
            else return nullptr;
        }

        inline Drivable *getNextDrivable(int i = 0) {
            return controllerInfo.router.getNextDrivable(i);
        }

        inline Drivable *getPrevDrivable(int i = 1) const {
            return controllerInfo.prevDrivable;
        }

        inline int getPriority() const { return priority; }

        std::pair<Point, Point> getCurPos() const;

        ControlInfo getNextSpeed(double interval);

        Drivable *getChangedDrivable() const;

        double getEnterTime() const { return enterTime; }

        bool isEnd() const { return controllerInfo.end; }

        bool isIntersectionRelated();

        double getBrakeDistanceAfterAccel(double acc, double dec, double interval) const;

        inline double getMinBrakeDistance() const { return 0.5 * vehicleInfo.speed * vehicleInfo.speed / vehicleInfo.maxNegAcc; }

        inline double getUsualBrakeDistance() const { return 0.5 * vehicleInfo.speed * vehicleInfo.speed / vehicleInfo.usualNegAcc; }

        double getNoCollisionSpeed(double vL, double dL, double vF, double dF, double gap, double interval,
            double targetGap) const;

        double getCarFollowSpeed(double interval);

        double getStopBeforeSpeed(double distance, double interval) const;

        int getReachSteps(double distance, double targetSpeed, double acc) const;

        int getReachStepsOnLaneLink(double distance, LaneLink* laneLink) const;

        double getDistanceUntilSpeed(double speed, double acc) const;

        bool canYield(double dist) const;

        void updateLeaderAndGap(Vehicle *leader);

        Vehicle *getLeader() const { return controllerInfo.leader; }

        inline double getEnterLaneLinkTime() const { return controllerInfo.enterLaneLinkTime; }

        inline double getHeadwayTime() const { return vehicleInfo.headwayTime; }

        inline double getMaxSpeed() const { return vehicleInfo.maxSpeed; }

        inline double getApproachingIntersectionDistance() const { return 0.0; }

        double getIntersectionRelatedSpeed(double interval);

        inline bool isRunning() const { return controllerInfo.running; }

        inline void setRunning(bool isRunning = true) { controllerInfo.running = isRunning; }

        inline bool hasPartner() const { return laneChangeInfo.partnerType > 0; }

        inline bool isReal() const { return laneChangeInfo.partnerType != 2; }

        inline size_t getSegmentIndex() const { return laneChangeInfo.segmentIndex; }

        inline void setSegmentIndex(int segmentIndex) { laneChangeInfo.segmentIndex = segmentIndex; }

        inline void setShadow(Vehicle *veh) { laneChangeInfo.partnerType = 1, laneChangeInfo.partner = veh; }

        inline void setParent(Vehicle *veh) { laneChangeInfo.partnerType = 2, laneChangeInfo.partner = veh; }

        void setLane(Lane *nextLane);

        void finishChanging();

        inline void setOffset(double offset) { laneChangeInfo.offset = offset; }

        inline double getOffset() const { return laneChangeInfo.offset; }

        inline Vehicle *getPartner() const { return laneChangeInfo.partner; }

        inline void setId(const std::string & id) { this->id = id; }

        //for lane change
        inline void makeLaneChangeSignal(double interval){
            laneChange->makeSignal(interval);
        }

        inline bool planLaneChange(){
            return laneChange->planChange();
        }

        void receiveSignal(Vehicle * sender);

        void sendSignal() { laneChange->sendSignal(); }

        void clearSignal(){ laneChange->clearSignal(); }

        void updateLaneChangeNeighbor(){ laneChange->updateLeaderAndFollower(); }

        std::shared_ptr<LaneChange> getLaneChange(){ return laneChange; }

        std::list<Vehicle *>::iterator getListIterator();

        void insertShadow(Vehicle *shadow) { laneChange->insertShadow(shadow); }

        bool onValidLane() const{ return controllerInfo.router.onValidLane(); }

        Lane * getValidLane() const{
            assert(getCurDrivable()->isLane());
            return controllerInfo.router.getValidLane(dynamic_cast<Lane *>(getCurDrivable()));
        }

        bool canChange() const{ return laneChange->canChange(); }

        double getGap() const{ return controllerInfo.gap; }

        int laneChangeUrgency() const { return laneChange->signalSend->urgency; }

        Vehicle *getTargetLeader() const { return laneChange->targetLeader; }

        int lastLaneChangeDirection() const { return laneChange->lastDir; }

        int getLaneChangeDirection() const {
            if (!laneChange->signalSend) return 0;
            return laneChange->signalSend->direction;
        }

        bool isChanging() const { return laneChange->changing; }

        double getMaxOffset() const {
            auto target = laneChange->signalSend->target;
            return (target->getWidth() + getCurLane()->getWidth()) / 2;
        }

        void abortLaneChange() ;

        void updateRoute();

        Road *getFirstRoad();

        void setFirstDrivable();

        bool isRouteValid() const { return this->routeValid; }

        Flow *getFlow() { return flow; }

        bool setRoute(const std::vector<Road *> &anchor);

        std::map<std::string, std::string> getInfo() const;

     };

}

#endif