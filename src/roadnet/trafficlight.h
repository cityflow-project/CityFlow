#ifndef CITYFLOW_TRAFFICLIGHT_H
#define CITYFLOW_TRAFFICLIGHT_H

#include <vector>

namespace CityFlow {
    class Intersection;

    class RoadLink;

    class RoadNet;

    class TrafficLight;

    class LightPhase {
        friend class RoadNet;
        friend class RoadLink;
        friend class TrafficLight;
    private:
        unsigned int phase = 0;
        double time = 0.0;
        std::vector<bool> roadLinkAvailable;
    };

    class TrafficLight {
        friend class RoadNet;
        friend class Archive;
    private:
        Intersection *intersection = nullptr;
        std::vector<LightPhase> phases;
        std::vector<int> roadLinkIndices;
        double remainDuration = 0.0;
        int curPhaseIndex = 0;
    public:
        void init(int initPhaseIndex);

        int getCurrentPhaseIndex();

        LightPhase &getCurrentPhase();

        Intersection &getIntersection();

        std::vector<LightPhase> &getPhases();

        void passTime(double seconds);

        void setPhase(int phaseIndex);

        void reset();
    };
}

#endif //CITYFLOW_TRAFFICLIGHT_H