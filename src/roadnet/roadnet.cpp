#include "roadnet/roadnet.h"
#include "roadnet/trafficlight.h"
#include "utility/config.h"
#include "vehicle/vehicle.h"
#include "json/json.h"

#include <iostream>
#include <fstream>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <deque>

using std::map;
using std::string;

namespace CityFlow {
    static double getLengthOfPoints(const std::vector<Point> &points);

    static Point getPointByDistance(const std::vector<Point> &points, double dis) {
        dis = min2double(max2double(dis, 0), getLengthOfPoints(points));
        if (dis <= 0.0)
            return points[0];
        for (size_t i = 1; i < points.size(); i++) {
            double len = (points[i - 1] - points[i]).len();
            if (dis > len)
                dis -= len;
            else
                return points[i - 1] + (points[i] - points[i - 1]) * (dis / len);
        }
        return points.back();
    }

    static double getLengthOfPoints(const std::vector<Point> &points) {
        double length = 0.0;
        for (size_t i = 0; i + 1 < points.size(); i++)
            length += (points[i + 1] - points[i]).len();
        return length;
    }

    Point *RoadNet::getPoint(Point *p1, Point *p2, double a) {
        return new Point((p2->x - p1->x) * a + p1->x, (p2->y - p1->y) * a + p1->y);
    }

    bool RoadNet::loadFromJson(std::string jsonFileName) {
        Json::Value root;

        std::ifstream jsonfile(jsonFileName, std::ifstream::binary);
        if (!jsonfile.is_open()) {
            std::cerr << "cannot open roadnet file" << std::endl;
            return false;
        }
        jsonfile >> root;
        jsonfile.close();

        //std::clog << root << std::endl;

        Json::Value interValues = root["intersections"];
        Json::Value roadValues = root["roads"];

        //  build mapping
        roads.resize(roadValues.size());
        intersections.resize(interValues.size());
        for (int i = 0; i < (int) roadValues.size(); i++) {
            roadMap[roadValues[i]["id"].asString()] = &roads[i];
            roads[i].id = roadValues[i]["id"].asString();
        }
        for (int i = 0; i < (int) interValues.size(); i++) {
            interMap[interValues[i]["id"].asString()] = &intersections[i];
            intersections[i].id = interValues[i]["id"].asString();
        }

        //  read roads
        for (int i = 0; i < (int) roadValues.size(); i++) {
            //  read startIntersection, endIntersection
            roads[i].startIntersection = interMap[roadValues[i]["startIntersection"].asString()];
            roads[i].endIntersection = interMap[roadValues[i]["endIntersection"].asString()];
            //  read lanes
            Json::Value lanesValue = roadValues[i]["lanes"];
            int laneIndex = 0;
            for (auto &laneValue : lanesValue) {
                roads[i].lanes.emplace_back(laneValue["width"].asDouble(), laneValue["maxSpeed"].asDouble(), laneIndex,
                                            &roads[i]);
                laneIndex++;
            }
            //  read points
            Json::Value pointsValue = roadValues[i]["points"];
            for (auto &pointValue : pointsValue)
                roads[i].points.emplace_back(pointValue["x"].asDouble(), pointValue["y"].asDouble());
        }
        for (int i = 0; i < (int) roadValues.size(); i++) {
            roads[i].initLanesPoints();
        }
        //  read intersections
        std::map<std::string, RoadLinkType> typeMap = {{ "turn_left",   turn_left },
                                                       { "turn_right",  turn_right },
                                                       { "go_straight", go_straight }};
        for (int i = 0; i < (int) interValues.size(); i++) {
            //  read point
            Json::Value pointValue = interValues[i]["point"];
            intersections[i].isVirtual = interValues[i]["virtual"].asBool();
            intersections[i].point = Point(pointValue["x"].asDouble(), pointValue["y"].asDouble());

            //  read roads
            Json::Value roadsValue = interValues[i]["roads"];
            for (auto &roadNameValue : roadsValue)
                intersections[i].roads.push_back(roadMap[roadNameValue.asString()]);
            //  skip other information if intersection is virtual
            intersections[i].trafficLight.intersection = &intersections[i];
            if (intersections[i].isVirtual)
                continue;
            //  read width
            intersections[i].width = interValues[i]["width"].asDouble();
            //  read laneLinks
            Json::Value roadLinksValue = interValues[i]["roadLinks"];
            intersections[i].roadLinks.resize(roadLinksValue.size());
            int roadLinkIndex = 0;
            for (Json::Value &roadLinkValue : roadLinksValue) {
                RoadLink &roadLink = intersections[i].roadLinks[roadLinkIndex];
                roadLink.index = roadLinkIndex++;
                roadLink.type = typeMap[roadLinkValue["type"].asString()];
                roadLink.startRoad = roadMap[roadLinkValue["startRoad"].asString()];
                roadLink.endRoad = roadMap[roadLinkValue["endRoad"].asString()];
                roadLink.laneLinks.resize(roadLinkValue["laneLinks"].size());
                int laneLinkIndex = 0;
                for (Json::Value laneLinkValue : roadLinkValue["laneLinks"]) {
                    LaneLink &laneLink = roadLink.laneLinks[laneLinkIndex++];
                    assert(laneLinkValue["startLaneIndex"].asInt() < (int) roadLink.startRoad->lanes.size());
                    assert(laneLinkValue["endLaneIndex"].asInt() < (int) roadLink.endRoad->lanes.size());
                    Lane *startLane = &roadLink.startRoad->lanes[laneLinkValue["startLaneIndex"].asInt()];
                    Lane *endLane = &roadLink.endRoad->lanes[laneLinkValue["endLaneIndex"].asInt()];
                    if (!laneLinkValue["points"].empty())
                        for (Json::Value pValue : laneLinkValue["points"]) {
                            laneLink.points.emplace_back(pValue["x"].asDouble(), pValue["y"].asDouble());
                        }
                    else {
                        Point *start = new Point(startLane->getPointByDistance(
                                startLane->getLength() - startLane->getEndIntersection()->width));
                        Point *end = new Point(
                                endLane->getPointByDistance(0.0 + endLane->getStartIntersection()->width));
                        double len = (new Point(end->x - start->x, end->y - start->y))->len();
                        Point startDirection = startLane->getDirectionByDistance(
                                startLane->getLength() - startLane->getEndIntersection()->width);
                        Point endDirection = endLane->getDirectionByDistance(
                                0.0 + endLane->getStartIntersection()->width);
                        double minGap = 5;
                        double gap1X = startDirection.x * len * 0.5;
                        double gap1Y = startDirection.y * len * 0.5;
                        double gap2X = -endDirection.x * len * 0.5;
                        double gap2Y = -endDirection.y * len * 0.5;
                        if (gap1X * gap1X + gap1Y * gap1Y < 25 && startLane->getEndIntersection()->width >= 5) {
                            gap1X = minGap * startDirection.x;
                            gap1Y = minGap * startDirection.y;
                        }
                        if (gap2X * gap2X + gap2Y * gap2Y < 25 && endLane->getStartIntersection()->width >= 5) {
                            gap2X = minGap * endDirection.x;
                            gap2Y = minGap * endDirection.y;
                        }
                        Point *mid1 = new Point(start->x + gap1X,
                                                start->y + gap1Y);
                        Point *mid2 = new Point(end->x + gap2X,
                                                end->y + gap2Y);
                        int numPoints = 10;
                        for (int i = 0; i <= numPoints; i++) {
                            Point *p1 = getPoint(start, mid1, i / double(numPoints));
                            Point *p2 = getPoint(mid1, mid2, i / double(numPoints));
                            Point *p3 = getPoint(mid2, end, i / double(numPoints));
                            Point *p4 = getPoint(p1, p2, i / double(numPoints));
                            Point *p5 = getPoint(p2, p3, i / double(numPoints));
                            Point *p6 = getPoint(p4, p5, i / double(numPoints));
                            laneLink.points.emplace_back(p6->x, p6->y);
                        }
                        delete (start);
                        delete (end);
                        delete (mid1);
                        delete (mid2);
                    }
                    laneLink.roadLink = &roadLink;

                    laneLink.startLane = startLane;
                    laneLink.endLane = endLane;
                    laneLink.length = getLengthOfPoints(laneLink.points);
                    startLane->laneLinks.push_back(&laneLink);
                }
                roadLink.intersection = &intersections[i];
            }
            //  read trafficLight
            Json::Value trafficLightValue = interValues[i]["trafficLight"];
            for (Json::Value &lightPhaseValue : trafficLightValue["lightphases"]) {
                LightPhase lightPhase;
                lightPhase.phase = (unsigned int) lightPhaseValue["phase"].asInt();
                lightPhase.time = lightPhaseValue["time"].asDouble();
                lightPhase.roadLinkAvailable = std::vector<bool>(intersections[i].roadLinks.size(), false);
                for (int index = 0; index < (int) lightPhaseValue["availableRoadLinks"].size(); index++) {
                    int indexInRoadLinks = lightPhaseValue["availableRoadLinks"][index].asInt();
                    lightPhase.roadLinkAvailable[indexInRoadLinks] = true;
                }
                intersections[i].trafficLight.phases.push_back(lightPhase);
            }
            intersections[i].trafficLight.init(0);
        }
        for (int i = 0; i < (int) intersections.size(); i++)
            intersections[i].initCrosses();
        VehicleInfo vehicleTemplate;
        for (int i = 0; i < (int) roadValues.size(); i++) {
            roads[i].initLanesPoints();
        }
        for (auto &road : roads) {
            road.buildSegmentationByInterval((vehicleTemplate.len + vehicleTemplate.minGap) * MAX_NUM_CARS_ON_SEGMENT);
        }

        for (auto &road : roads) {
            auto &roadLanes = road.getLanePointers();
            lanes.insert(lanes.end(), roadLanes.begin(), roadLanes.end());
            drivables.insert(drivables.end(), roadLanes.begin(), roadLanes.end());
        }
        for (auto &intersection : intersections) {
            auto &intersectionLaneLinks = intersection.getLaneLinks();
            laneLinks.insert(laneLinks.end(), intersectionLaneLinks.begin(), intersectionLaneLinks.end());
            drivables.insert(drivables.end(), intersectionLaneLinks.begin(), intersectionLaneLinks.end());
        }
        return true;
    }

    Json::Value RoadNet::convertToJson() {
        Json::Value jsonRoot;

        // write nodes
        Json::Value jsonNodes;
        for (int i = 0; i < (int) intersections.size(); ++i) {
            Json::Value jsonNode, jsonPoint(Json::arrayValue);
            jsonNode["id"] = intersections[i].id;
            jsonPoint.append(Json::Value(intersections[i].point.x));
            jsonPoint.append(Json::Value(intersections[i].point.y));
            jsonNode["point"] = jsonPoint;
            jsonNode["virtual"] = intersections[i].isVirtual;
            if (!intersections[i].isVirtual)
                jsonNode["width"] = intersections[i].width;

            Json::Value jsonOutline(Json::arrayValue);
            for (auto &point: intersections[i].getOutline()) {
                jsonOutline.append(Json::Value(point.x));
                jsonOutline.append(Json::Value(point.y));
            }

            jsonNode["outline"] = jsonOutline;

            jsonNodes.append(jsonNode);
        }
        jsonRoot["nodes"] = jsonNodes;

        //write edges
        Json::Value jsonEdges;
        for (int i = 0; i < (int) roads.size(); ++i) {
            Json::Value jsonEdge;
            Json::Value jsonPoints(Json::arrayValue);
            Json::Value jsonLaneWidths(Json::arrayValue);
            Json::Value jsonDirs(Json::arrayValue);
            jsonEdge["id"] = roads[i].id;
            if (roads[i].startIntersection)
                jsonEdge["from"] = roads[i].startIntersection->id;
            else
                jsonEdge["from"] = "null";
            if (roads[i].endIntersection)
                jsonEdge["to"] = roads[i].endIntersection->id;
            else
                jsonEdge["from"] = "null";
            for (int j = 0; j < (int) roads[i].points.size(); ++j) {
                Json::Value jsonPoint(Json::arrayValue);
                jsonPoint.append(Json::Value(roads[i].points[j].x));
                jsonPoint.append(Json::Value(roads[i].points[j].y));
                jsonPoints.append(jsonPoint);
            }
            jsonEdge["points"] = jsonPoints;
            jsonEdge["nLane"] = (int) roads[i].lanes.size();
            for (int j = 0; j < (int) roads[i].lanes.size(); ++j) {
                jsonLaneWidths.append(Json::Value(roads[i].lanes[j].width));
            }
            jsonEdge["laneWidths"] = jsonLaneWidths;
            jsonEdges.append(jsonEdge);
        }
        jsonRoot["edges"] = jsonEdges;

        return jsonRoot;
    }

    Point Drivable::getPointByDistance(double dis) const {
        return CityFlow::getPointByDistance(points, dis);
    }

    Point Drivable::getDirectionByDistance(double dis) const {
        double remain = dis;
        for (int i = 0; i + 1 < (int) points.size(); i++) {
            double len = (points[i + 1] - points[i]).len();
            if (remain < len)
                return (points[i + 1] - points[i]).unit();
            else
                remain -= len;
        }
        return (points[points.size() - 1] - points[points.size() - 2]).unit();
    }

    Lane::Lane() {
        width = 0;
        maxSpeed = 0;
        laneIndex = -1;
        belongRoad = 0;
        drivableType = LANE;
    }

    Lane::Lane(double width, double maxSpeed, int laneIndex, Road *belongRoad) {
        this->width = width;
        this->maxSpeed = maxSpeed;
        this->laneIndex = laneIndex;
        this->belongRoad = belongRoad;
        drivableType = LANE;
    }

    bool Lane::available(const Vehicle *vehicle) const {
        if (!vehicles.empty()) {
            Vehicle *tail = vehicles.back();
            return tail->getDistance() > tail->getLen() + vehicle->getMinGap();
        } else {
            return true;
        }
    }

    bool Lane::canEnter(const Vehicle *vehicle) const {
        if (!vehicles.empty()) {
            Vehicle *tail = vehicles.back();
            return tail->getDistance() > tail->getLen() + vehicle->getLen() ||
                   tail->getSpeed() >= 2; //todo: speed > 2 or?
        } else {
            return true;
        }
    }

    std::vector<LaneLink *> Lane::getLaneLinksToRoad(Road *road) const {
        std::vector<LaneLink *> ret;
        for (auto &laneLink : laneLinks) {
            if (laneLink->getEndLane()->getBelongRoad() == road)
                ret.push_back(laneLink);
        }
        return ret;
    }

    void Road::initLanesPoints() {
        double dsum = 0.0;
        std::vector<Point> roadPoints = this->points;

        assert(roadPoints.size() >= 2);

        if (!startIntersection->isVirtualIntersection()) {
            double width = startIntersection->width;
            Point p1 = roadPoints[0];
            Point p2 = roadPoints[1];
            roadPoints[0] = p1 + (p2 - p1).unit() * width;
        }

        if (!endIntersection->isVirtualIntersection()) {
            double width = endIntersection->width;
            Point p1 = roadPoints[roadPoints.size() - 2];
            Point p2 = roadPoints[roadPoints.size() - 1];
            roadPoints[roadPoints.size() - 1] = p2 - (p2 - p1).unit() * width;
        }

        for (Lane &lane : lanes) {
            double dmin = dsum;
            double dmax = dsum + lane.width;
            lane.points.clear();
            for (int j = 0; j < (int) roadPoints.size(); j++) {
                // TODO: the '(dmin + dmax) / 2.0' is wrong
                std::vector<Point> &lanePoints = lane.points;
                if (j == 0) {
                    Vector u = (roadPoints[1] - roadPoints[0]).unit();
                    Vector v = -u.normal();
                    Point startPoint = roadPoints[j] + v * ((dmin + dmax) / 2.0);
                    lanePoints.push_back(startPoint);
                } else if (j + 1 == (int) roadPoints.size()) {
                    Vector u = (roadPoints[j] - roadPoints[j - 1]).unit();
                    Vector v = -u.normal();
                    Point endPoint = roadPoints[j] + v * ((dmin + dmax) / 2.0);
                    lanePoints.push_back(endPoint);
                } else {
                    Vector u1 = (roadPoints[j + 1] - roadPoints[j]).unit();
                    Vector u2 = (roadPoints[j] - roadPoints[j - 1]).unit();
                    Vector u = (u1 + u2).unit();
                    Vector v = -u.normal();
                    Point interPoint = roadPoints[j] + v * ((dmin + dmax) / 2.0);
                    lanePoints.push_back(interPoint);
                }
            }
            lane.length = getLengthOfPoints(lane.points);
            dsum += lane.width;
        }
    }

    const std::vector<Lane *> &Road::getLanePointers() {
        if (lanePointers.size()) return lanePointers;
        for (auto &lane : lanes) {
            lanePointers.push_back(&lane);
        }
        return lanePointers;
    }

    void Intersection::initCrosses() {
        std::vector<LaneLink *> allLaneLinks;
        for (auto &roadLink : roadLinks) {
            for (auto &laneLink : roadLink.getLaneLinks())
                allLaneLinks.push_back(&laneLink);
        }
        int n = (int) allLaneLinks.size();

        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                LaneLink *la = allLaneLinks[i];
                LaneLink *lb = allLaneLinks[j];
                std::vector<Point> &va = la->points;
                std::vector<Point> &vb = lb->points;
                double disa = 0.0;
                for (int ia = 0; ia + 1 < (int) va.size(); ia++) {
                    double disb = 0.0;
                    for (int ib = 0; ib + 1 < (int) vb.size(); ib++) {
                        Point A1 = va[ia], A2 = va[ia + 1];
                        Point B1 = vb[ib], B2 = vb[ib + 1];
                        if (Point::sign(crossMultiply(A2 - A1, B2 - B1)) == 0) continue;
                        Point P = calcIntersectPoint(A1, A2, B1, B2);
                        if (onSegment(A1, A2, P) && onSegment(B1, B2, P)) {
                            Cross cross;
                            cross.laneLinks[0] = la;
                            cross.laneLinks[1] = lb;
                            cross.notifyVehicles[0] = nullptr;
                            cross.notifyVehicles[1] = nullptr;
                            cross.distanceOnLane[0] = disa + (P - A1).len();
                            cross.distanceOnLane[1] = disb + (P - B1).len();
                            cross.ang = calcAng(A2 - A1, B2 - B1);
                            //assert(cross.ang > 0 && cross.ang < M_PI / 2); // assert cannot pass why?
                            double w1 = la->getWidth();
                            double w2 = lb->getWidth();
                            double c1 = w1 / sin(cross.ang);
                            double c2 = w2 / sin(cross.ang);
                            double diag = (c1 * c1 + c2 * c2 + 2 * c1 * c2 * cos(cross.ang)) / 4;
                            cross.safeDistances[0] = sqrt(diag - w2 * w2 / 4);
                            cross.safeDistances[1] = sqrt(diag - w1 * w1 / 4);
                            this->crosses.push_back(cross);
                            goto FOUND;
                        }
                        disb += (vb[ib + 1] - vb[ib]).len();
                    }
                    disa += (va[ia + 1] - va[ia]).len();
                }
FOUND:;
            }
        }
        for (Cross &cross : this->crosses) {
            cross.laneLinks[0]->getCrosses().push_back(&cross);
            cross.laneLinks[1]->getCrosses().push_back(&cross);
        }
        for (auto laneLink : allLaneLinks) {
            std::vector<Cross *> &crosses = laneLink->getCrosses();
            sort(crosses.begin(), crosses.end(), [laneLink](Cross *ca, Cross *cb) -> bool {
                double da = ca->distanceOnLane[ca->laneLinks[0] != laneLink];
                double db = cb->distanceOnLane[cb->laneLinks[0] != laneLink];
                return da < db;
            });
        }
    }

    const std::vector<LaneLink *> &Intersection::getLaneLinks() {
        if (laneLinks.size() > 0) return laneLinks;
        for (auto &roadLink : roadLinks) {
            auto &roadLaneLinks = roadLink.getLaneLinkPointers();
            laneLinks.insert(laneLinks.end(), roadLaneLinks.begin(), roadLaneLinks.end());
        }
        return laneLinks;
    }

    const std::vector<LaneLink *> &RoadLink::getLaneLinkPointers() {
        if (laneLinkPointers.size() > 0) return laneLinkPointers;
        for (auto &laneLink : laneLinks) {
            laneLinkPointers.push_back(&laneLink);
        }
        return laneLinkPointers;
    }

    void Cross::notify(LaneLink *laneLink, Vehicle *vehicle, double notifyDistance) {
        assert(laneLink == laneLinks[0] || laneLink == laneLinks[1]);
        int i = (laneLink == laneLinks[0]) ? 0 : 1;
        assert(notifyVehicles[i] == nullptr);
        notifyVehicles[i] = vehicle;
        notifyDistances[i] = notifyDistance;
    }

    bool Cross::canPass(const Vehicle *vehicle, const LaneLink *laneLink, double distanceToLaneLinkStart) const {
        // TODO: should be improved
        assert(laneLink == laneLinks[0] || laneLink == laneLinks[1]);
        int i = (laneLink == laneLinks[0]) ? 0 : 1;

        Vehicle *foeVehicle = notifyVehicles[1 - i];
        RoadLinkType t1 = laneLinks[i]->getRoadLinkType();
        RoadLinkType t2 = laneLinks[1 - i]->getRoadLinkType();
        double d1 = distanceOnLane[i] - distanceToLaneLinkStart, d2 = notifyDistances[1 - i];

        if (foeVehicle == nullptr) return true;

        if (!vehicle->canYield(d1)) return true;

        int yield = 0;
        if (!foeVehicle->canYield(d2)) yield = 1;

        if (yield == 0) {
            if (t1 > t2) {
                yield = -1;
            } else if (t1 < t2) {
                if (d2 > 0) {
                    // todo: can be improved, check if higher priority vehicle is blocked by other vehicles, hard!
                    int foeVehicleReachSteps = foeVehicle->getReachStepsOnLaneLink(d2, laneLinks[1 - i]);
                    int reachSteps = vehicle->getReachStepsOnLaneLink(d1, laneLinks[i]);
                    if (foeVehicleReachSteps > reachSteps) {
                        yield = -1;
                    }
                } else {
                    if (d2 + foeVehicle->getLen() < 0) {
                        yield = -1;
                    }
                }
                if (yield == 0) yield = 1;
            } else {
                if (d2 > 0) {
                    int foeVehicleReachSteps = foeVehicle->getReachStepsOnLaneLink(d2, laneLinks[1 - i]);
                    int reachSteps = vehicle->getReachStepsOnLaneLink(d1, laneLinks[i]);
                    if (foeVehicleReachSteps > reachSteps) {
                        yield = -1;
                    } else if (foeVehicleReachSteps < reachSteps) {
                        yield = 1;
                    } else {
                        if (vehicle->getEnterLaneLinkTime() == foeVehicle->getEnterLaneLinkTime()) {
                            if (d1 == d2) {
                                yield = vehicle->getPriority() > foeVehicle->getPriority() ? -1 : 1;
                            } else {
                                yield = d1 < d2 ? -1 : 1;
                            }
                        } else {
                            yield = vehicle->getEnterLaneLinkTime() < foeVehicle->getEnterLaneLinkTime() ? -1 : 1;
                        }
                    }
                } else {
                    yield = d2 + foeVehicle->getLen() < 0 ? -1 : 1;
                }
            }
        }
        assert(yield != 0);
        if (yield == 1) {
            Vehicle *fastPointer = foeVehicle;
            Vehicle *slowPointer = foeVehicle;
            while (fastPointer != nullptr && fastPointer->getBlocker() != nullptr) {
                slowPointer = slowPointer->getBlocker();
                fastPointer = fastPointer->getBlocker()->getBlocker();
                if (slowPointer == fastPointer) {
                    // deadlock detected
                    yield = -1;
                    break;
                }
            }
        }
        return yield == -1;
    }

    void RoadNet::reset() {
        for (auto &road : roads) road.reset();
        for (auto &intersection : intersections) intersection.reset();
    }

    void Road::reset() {
        for (auto &lane : lanes) lane.reset();
    }

    void Road::buildSegmentationByInterval(double interval) {
        size_t numSegs = std::max((size_t) ceil(getLengthOfPoints(this->points) / interval), (size_t) 1);
        for (Lane &lane : lanes)
            lane.buildSegmentation(numSegs);
    }

    double Road::getWidth() {
        double width = 0;
        for (auto lane : getLanePointers()){
            width += lane->getWidth();
        }
        return width;
    }

    double Road::getLength() {
        double length = 0;
        for (auto lane : getLanePointers()){
            length += lane->getLength();
        }
        return length;
    }

    void Intersection::reset() {
        trafficLight.reset();
        for (auto &roadLink : roadLinks) roadLink.reset();
        for (auto &cross : crosses) cross.reset();
    }

    std::vector<Point> Intersection::getOutline() {
        // Calculate the convex hull as the outline of the intersection
        std::vector<Point> points;
        points.push_back(getPosition());
        for (auto road : getRoads()){
            Vector roadDirect = road->getEndIntersection().getPosition() - road->getStartIntersection().getPosition();
            roadDirect = roadDirect.unit();
            Vector pDirect = roadDirect.normal();
            if (&road->getStartIntersection() == this) {
                roadDirect = -roadDirect;
            }
            /*                          <deltaWidth>
             *                   [pointB *]------[pointB1 *]--------
             *                       |
             *                       v
             *                   [pDirect] <- roadDirect <- Road
             *                       |
             *                       v
             * [intersection]----[pointA *]------[pointA1 *]--------
             */
            double roadWidth = road->getWidth();
            double deltaWidth = 0.5 * min2double(width, roadWidth);
            deltaWidth = max2double(deltaWidth, 5);

            Point pointA = getPosition() -  roadDirect * width;
            Point pointB  = pointA - pDirect * roadWidth;
            points.push_back(pointA);
            points.push_back(pointB);

            if (deltaWidth < road->getLength()) {
                Point pointA1 = pointA - roadDirect * deltaWidth;
                Point pointB1 = pointB - roadDirect * deltaWidth;
                points.push_back(pointA1);
                points.push_back(pointB1);
            }
        }

        auto minIter = std::min_element(points.begin(), points.end(),
                [](const Point &a, const Point &b){ return a.y < b.y; });

        Point p0 = *minIter;
        std::vector<Point> stack;
        stack.push_back(p0);
        points.erase(minIter);

        std::sort(points.begin(), points.end(),
                [&p0](const Point &a, const Point &b)
                {return (a - p0).ang() < (b - p0).ang(); });

        stack.push_back(points[0]);
        for (int i = 1 ; i < points.size(); ++i) {
            Point &point = points[i];
            Point p1 = stack[stack.size() - 2];
            Point p2 = stack[stack.size() - 1];

            if (point.x == p2.x && point.y == p2.y) continue;

            while (stack.size() > 1 && crossMultiply(point - p2, p2 - p1) > 0) {
                p2 = p1;
                stack.pop_back();
                if (stack.size() > 1) p1 = stack[stack.size() - 2];
            }
            stack.push_back(point);
        }

        return stack;
    }

    bool Intersection::isImplicitIntersection() {
        return trafficLight.getPhases().size() <= 1;
    }

    void RoadLink::reset() {
        for (auto &laneLink : laneLinks) laneLink.reset();
    }

    void LaneLink::reset() {
        vehicles.clear();
    }

    void Lane::reset() {
        waitingBuffer.clear();
        vehicles.clear();
    }

    std::vector<Vehicle *> Lane::getVehiclesBeforeDistance(double dis, size_t segmentIndex, double deltaDis) {
        std::vector<Vehicle *> ret;
        for (int i = segmentIndex; i >=0 ;i--) {
            Segment * segment = getSegment(i);
            auto &vehicles = segment->getVehicles();
            for(auto it = vehicles.begin(); it != vehicles.end(); ++it) {
                Vehicle *vehicle = *(*it);
                if (vehicle->getDistance() < dis - deltaDis) return ret;
                if (vehicle->getDistance() < dis) ret.emplace_back(vehicle);
            }
        }
        return ret;
    }


    void Lane::buildSegmentation(size_t numSegs) {
        this->segments.resize((unsigned) numSegs);
        for (size_t i = 0; i < numSegs; i++) {
            segments[i].index = i;
            segments[i].vehicles.clear();
            segments[i].belongLane = this;
            segments[i].startPos = i * this->length / numSegs;
            segments[i].endPos = (i + 1) * this->length / numSegs;
        }
    }

    void Lane::initSegments() {
        auto iter = this->vehicles.begin();
        auto end = this->vehicles.end();
        for (int i = (int) segments.size() - 1; i >= 0; i--) {
            Segment &seg = segments[i];
            seg.vehicles.clear();
            while (iter != end && (*iter)->getDistance() >= seg.getStartPos()) {
                seg.vehicles.push_back(iter);
                (*iter)->setSegmentIndex(seg.index);
                ++iter;
            }
//            if (iter != end) std::cerr << (*iter)->getId() << std::endl;
            //if (seg.tryChange != nullptr && (seg.tryChange->getCurDrivable()->isLaneLink() || seg.tryChange->getSegmentIndex() != i))
            //    seg.tryChange = nullptr;
//            seg.prev_vehicle_iter = iter;
        }
    }

    Vehicle *Lane::getVehicleBeforeDistance(double dis, size_t segmentIndex) {
        for (int i = segmentIndex ; i >= 0 ; --i){
            auto vehs = getSegment(i)->getVehicles();
            for (auto itr = vehs.begin() ; itr != vehs.end(); ++itr){
                auto &vehicle = **itr;
                if (vehicle->getDistance() < dis) return **itr;
            }
        }

        return nullptr;
    }

    Vehicle *Lane::getVehicleAfterDistance(double dis, size_t segmentIndex) {
        for (int i = segmentIndex ; i < getSegmentNum() ; ++i){
            auto vehs = getSegment(i)->getVehicles();
            for (auto itr = vehs.rbegin() ; itr != vehs.rend(); ++itr){
                auto &vehicle = **itr;
                if (vehicle->getDistance() >= dis) return **itr;
            }
        }
        return nullptr;
    }

    void Cross::reset() { }

    std::list<Vehicle *>::iterator Segment::findVehicle(Vehicle *vehicle) {
        for (auto itr = vehicles.begin() ; itr != vehicles.end() ; ++itr)
            if (**itr == vehicle) {
                return *itr;
            }
        return belongLane->getVehicles().end();
    }

    void Segment::removeVehicle(Vehicle *vehicle) {
        for (auto itr = vehicles.begin() ; itr != vehicles.end() ; ++itr)
            if (**itr == vehicle) {
                vehicles.erase(itr);
                return;
            }
    }

    void Segment::insertVehicle(std::list<Vehicle *>::iterator &vehicle) {
        auto itr = vehicles.begin();
        for (; itr != vehicles.end() && (**itr)->getDistance() > (*vehicle)->getDistance() ; ++itr);
        vehicles.insert(itr, vehicle);
    }

}

