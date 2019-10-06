#ifndef CITYFLOW_LANECHANGE_H
#define CITYFLOW_LANECHANGE_H

#include "roadnet/roadnet.h"

#include <memory>

namespace CityFlow {

    class Vehicle;
    class Lane;

    class LaneChange {
        friend class Vehicle;
        friend class Archive;
    //The interface of lane changing
     protected:
        struct Signal{
            int urgency;
            int direction; // -1 for left , 1 for right, 0 for unchanged
            Lane * target;
            Vehicle * source;
            int response = 0;
            double extraSpace = 0;
        };

        int lastDir;

        std::shared_ptr<Signal> signalRecv;
        std::shared_ptr<Signal> signalSend;

        Vehicle * vehicle;
        Vehicle * targetLeader = nullptr;
        Vehicle * targetFollower = nullptr;

        double leaderGap;
        double followerGap;
        double waitingTime = 0;

        bool changing = false;
        bool finished = false;
        double lastChangeTime = 0;

        double notifyRange = 30;

        double ratio = 0.8;
        // Assumption: the ratio of the lane changing replacement and the distance going straight is fixed.

        //         ^    |
        //       / |    |
        //     /   |  [dy]
        //   /     |   |
        //  *------+   |
        //     [dx]
        //
        //   dx/dy = ratio

        double minimalLaneChangeSpeed = 0.4;
        double minimalSpeedChance = 0.2;
        double changeLanePenalty = 20;
        double wrongLanePenalty = 50;
        double laneChangeChance = 0.8;
        double aggressiveChance = 0.2;

        double angle = 0;

        static constexpr double coolingTime = 10;

        double customDirection;
        bool isCustomDirectionSet = false;

    public:
        LaneChange(Vehicle * vehicle, const LaneChange &other);

        explicit LaneChange(Vehicle * vehicle) : vehicle(vehicle) {};

        void updateLeaderAndFollower();

        Lane *getTarget() const;

        Vehicle *getTargetLeader() const {
            return targetLeader;
        }

        Vehicle *getTargetFollower() const {
            return targetFollower;
        }

        double gapBefore() const;

        double gapAfter() const;

        void insertShadow(Vehicle *shadow) ;

        double safeGapBefore() const;

        double safeGapAfter() const;

        virtual void makeSignal(double interval);

        bool planChange() const;

        bool canChange() const { return signalSend && !signalRecv; }

        bool isGapValid() const;

        void finishChanging();

        double yieldSpeed(double interval);

        void sendSignal();

        int getDirection();

        void clearSignal();

        bool hasFinished() const { return this->finished; }

        double safeDistance() const;

        double safeDistance(Lane * lane) const; // distance long enough for a vehicle to take lane change to a correct lane

        double safeDistance(double deltaOffset) const;

        Lane *nearestAvailableLane() const;

        double forceLaneChangeSpeed() const;

        double getDeltaOffset(double deltaDis) const;

        void changeToInner();

        void changeToOuter();

        void updateAngle(double dx, double dy);

        double laneChangeAngle() const;

        void setCustomDirection(int direction) {
            isCustomDirectionSet = true;
            customDirection = direction;
        }

        void clearCustomDirection() {
            isCustomDirectionSet = false;
        }

        double estimateGap(const Lane *lane) const;
    };
}

#endif //CITYFLOW_LANECHANGE_H
