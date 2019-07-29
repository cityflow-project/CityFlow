//
// Created by qidongsu on 19-7-7.
//

#ifndef CITYFLOW_LANECHANGE_H
#define CITYFLOW_LANECHANGE_H

#include <memory>
#include <roadnet/roadnet.h>

namespace CityFlow {

    class Vehicle;
    class Lane;

    class LaneChange {
        friend class Vehicle;
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
        double lastChangeTime = 0;

        static constexpr double coolingTime = 3;

    public:

        LaneChange(Vehicle * vehicle) : vehicle(vehicle) {};

        void updateLeaderAndFollower();

        Lane *getTarget() const;

        Vehicle *getTargetLeader() const {
            return targetLeader;
        }

        Vehicle *getTargetFollower() const {
            return targetFollower;
        }

        double gapBefore() const ;

        double gapAfter() const ;

        void insertShadow(Vehicle *shadow) ;

        virtual double safeGapBefore() = 0;
        virtual double safeGapAfter() = 0;

        virtual void makeSignal(double interval) { if (signalSend) signalSend->direction = getDirection(); };

        bool planChange();

        bool canChange() { return signalSend && !signalRecv; }

        bool isGapValid() { return gapAfter() >= safeGapAfter() && gapBefore() >= safeGapBefore(); }

        void finishChanging();

        virtual double yieldSpeed(double interval) = 0;

        virtual void sendSignal() = 0;

        virtual ~LaneChange() = default;

        int getDirection();

        void clearSignal();

    };

    class SimpleLaneChange : public LaneChange {
    private:
        double estimateGap(Lane *lane);
        std::mt19937 *rnd;
    public:
        SimpleLaneChange(Vehicle * vehicle, std::mt19937 *rnd) : LaneChange(vehicle), rnd(rnd) {};

        void makeSignal(double interval);
        void sendSignal();

        double yieldSpeed(double interval);

        double safeGapBefore() override;

        double safeGapAfter() override;

    };
}

#endif //CITYFLOW_LANECHANGE_H
