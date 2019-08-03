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

        virtual double safeGapBefore() const = 0;
        virtual double safeGapAfter() const = 0;

        virtual void makeSignal(double interval) { if (signalSend) signalSend->direction = getDirection(); };

        bool planChange() const;

        bool canChange() const { return signalSend && !signalRecv; }

        bool isGapValid() const { return gapAfter() >= safeGapAfter() && gapBefore() >= safeGapBefore(); }

        void finishChanging();

        virtual double yieldSpeed(double interval) = 0;

        virtual void sendSignal() = 0;

        virtual ~LaneChange() = default;

        int getDirection();

        void clearSignal();

    };

    class SimpleLaneChange : public LaneChange {
    private:
        double estimateGap(const Lane *lane) const;
    public:
        SimpleLaneChange(Vehicle * vehicle) : LaneChange(vehicle) {};

        void makeSignal(double interval) override;
        void sendSignal() override;

        double yieldSpeed(double interval) override;

        double safeGapBefore() const override;

        double safeGapAfter() const override;

    };
}

#endif //CITYFLOW_LANECHANGE_H
