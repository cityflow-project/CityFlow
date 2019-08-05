#ifndef CITYFLOW_ROUTE_H
#define CITYFLOW_ROUTE_H

#include <vector>

namespace CityFlow {
    class Road;

    class Route {
    private:
        std::vector<Road *> route;

    public:

        Route() = default;

        explicit Route(const std::vector<Road *> &route) : route(route) { }

        std::vector<Road *> getRoute() const { return route; }
    };
}
#endif //CITYFLOW_ROUTE_H
