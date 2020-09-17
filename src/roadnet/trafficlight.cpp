#include "roadnet/trafficlight.h"
#include "roadnet/roadnet.h"

namespace CityFlow {

    void TrafficLight::init(int initPhaseIndex) {
        if (intersection->isVirtual)
            return;
        this->curPhaseIndex = initPhaseIndex;
        this->remainDuration = phases[initPhaseIndex].time;
    }

    int TrafficLight::getCurrentPhaseIndex() {
        return this->curPhaseIndex;
    }

    LightPhase &TrafficLight::getCurrentPhase() {
        return this->phases.at(this->curPhaseIndex);
    }

    Intersection &TrafficLight::getIntersection() {
        return *this->intersection;
    }

    std::vector<LightPhase> &TrafficLight::getPhases() {
        return phases;
    }

    void TrafficLight::passTime(double seconds) {
        if(intersection->isVirtual)
            return;
        remainDuration -= seconds;
        while (remainDuration <= 0.0) {
            curPhaseIndex = (curPhaseIndex + 1) % (int) phases.size();
            remainDuration += phases[curPhaseIndex].time;
        }
    }

    void TrafficLight::setPhase(int phaseIndex) {
        curPhaseIndex = phaseIndex;
    }

    void TrafficLight::reset() {
        init(0);
    }

}
