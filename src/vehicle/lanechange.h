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

        static constexpr double coolingTime = 3;

    public:
        LaneChange(Vehicle * vehicle, const LaneChange &other);

        explicit LaneChange(Vehicle * vehicle) : vehicle(vehicle) {};

        virtual ~LaneChange() = default;

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

        void abortChanging();

        virtual double yieldSpeed(double interval) = 0;

        virtual void sendSignal() = 0;

        int getDirection();

        void clearSignal();

        bool hasFinished() const { return this->finished; }

    };

    class SimpleLaneChange : public LaneChange {
    private:
        double estimateGap(const Lane *lane) const;
    public:
        explicit SimpleLaneChange(Vehicle * vehicle) : LaneChange(vehicle) {};
        explicit SimpleLaneChange(Vehicle * vehicle, const LaneChange &other) : LaneChange(vehicle, other) {};

        void makeSignal(double interval) override;
        void sendSignal() override;

        double yieldSpeed(double interval) override;

        double safeGapBefore() const override;

        double safeGapAfter() const override;

    };
}

#endif //CITYFLOW_LANECHANGE_H
