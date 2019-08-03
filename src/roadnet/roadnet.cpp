#include "roadnet/roadnet.h"
#include "roadnet/trafficlight.h"
#include "utility/config.h"
#include "vehicle/vehicle.h"

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"

#include <iostream>
#include <fstream>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>

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
        FILE *fp = fopen(jsonFileName.c_str(), "r");
        if (!fp) {
            std::cerr << "cannot open roadnet file" << std::endl;
            return false;
        }
        char readBuffer[65536];
        rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
        rapidjson::Document document;
        document.ParseStream(is);
        fclose(fp);
        //std::clog << root << std::endl;

        if (!document.IsObject())
            throw jsonTypeError("roadnet", "object");
        try {
            const rapidjson::Value &interValues = getJsonMemberArray("intersections", document);
            const rapidjson::Value &roadValues = getJsonMemberArray("roads", document);

            //  build mapping
            roads.resize(roadValues.Size());
            intersections.resize(interValues.Size());
            for (rapidjson::SizeType i = 0; i < roadValues.Size(); i++) {
                std::string id = getJsonMemberString("id", roadValues[i]);
                roadMap[id] = &roads[i];
                roads[i].id = id;
            }
            for (rapidjson::SizeType i = 0; i < interValues.Size(); i++) {
                std::string id = getJsonMemberString("id", interValues[i]);;
                interMap[id] = &intersections[i];
                intersections[i].id = id;
            }

            //  read roads
            for (rapidjson::SizeType i = 0; i < roadValues.Size(); i++) {
                //  read startIntersection, endIntersection
                const auto &curRoadValue = roadValues[i];
                if (!curRoadValue.IsObject()) {
                    throw jsonTypeError("road[" + std::to_string(i) + "]", "object");
                }
                roads[i].startIntersection = interMap[getJsonMemberString("startIntersection", curRoadValue)];
                roads[i].endIntersection = interMap[getJsonMemberString("endIntersection", curRoadValue)];

                //  read lanes
                const auto &lanesValue = getJsonMemberArray("lanes", curRoadValue);
                int laneIndex = 0;
                for (const auto &laneValue : lanesValue.GetArray()) {
                    if (!laneValue.IsObject())
                        throw jsonTypeError("lane", "object");
                    double width = getJsonMemberDouble("width", laneValue);
                    double maxSpeed = getJsonMemberDouble("maxSpeed", laneValue);
                    roads[i].lanes.emplace_back(width, maxSpeed, laneIndex, &roads[i]);
                    laneIndex++;
                }

                //  read points
                const auto &pointsValue = getJsonMemberArray("points", curRoadValue);
                for (const auto &pointValue : pointsValue.GetArray()) {
                    if (!pointValue.IsObject())
                        throw jsonTypeError("point of road", "object");
                    double x = getJsonMemberDouble("x", pointValue);
                    double y = getJsonMemberDouble("y", pointValue);
                    roads[i].points.emplace_back(x, y);
                }
            }
            for (rapidjson::SizeType i = 0; i < roadValues.Size(); i++) {
                roads[i].initLanesPoints();
            }

            //  read intersections
            std::map<std::string, RoadLinkType> typeMap = {{"turn_left",   turn_left},
                                                           {"turn_right",  turn_right},
                                                           {"go_straight", go_straight}};
            for (rapidjson::SizeType i = 0; i < interValues.Size(); i++) {
                const auto &curInterValue = interValues[i];
                if (!curInterValue.IsObject()) {
                    throw jsonTypeError("intersection", "object");
                    return false;
                }

                //  read point
                const auto &pointValue = getJsonMemberObject("point", curInterValue);
                intersections[i].isVirtual = getJsonMemberBool("virtual", curInterValue);
                double x = getJsonMemberDouble("x", pointValue);
                double y = getJsonMemberDouble("y", pointValue);
                intersections[i].point = Point(x, y);

                //  read roads
                const auto &roadsValue = getJsonMemberArray("roads", curInterValue);
                for (auto &roadNameValue : roadsValue.GetArray()) {
                    intersections[i].roads.push_back(roadMap[roadNameValue.GetString()]);
                }

                //  skip other information if intersection is virtual
                intersections[i].trafficLight.intersection = &intersections[i];
                if (intersections[i].isVirtual)
                    continue;

                //  read width
                intersections[i].width = getJsonMemberDouble("width", curInterValue);

                //  read laneLinks
                const auto &roadLinksValue = getJsonMemberArray("roadLinks", curInterValue);
                intersections[i].roadLinks.resize(roadLinksValue.Size());
                int roadLinkIndex = 0;
                for (const auto &roadLinkValue : roadLinksValue.GetArray()) {
                    if (!roadLinkValue.IsObject())
                        throw jsonTypeError("roadLink", "object");
                    RoadLink &roadLink = intersections[i].roadLinks[roadLinkIndex];
                    roadLink.index = roadLinkIndex++;
                    roadLink.type = typeMap[getJsonMemberString("type", roadLinkValue)];
                    roadLink.startRoad = roadMap[getJsonMemberString("startRoad", roadLinkValue)];
                    roadLink.endRoad = roadMap[getJsonMemberString("endRoad", roadLinkValue)];

                    const auto &laneLinksValue = getJsonMemberArray("laneLinks", roadLinkValue);
                    roadLink.laneLinks.resize(laneLinksValue.Size());
                    int laneLinkIndex = 0;
                    for (const auto &laneLinkValue : laneLinksValue.GetArray()) {
                        if (!laneLinkValue.IsObject())
                            throw jsonTypeError("laneLink", "object");
                        LaneLink &laneLink = roadLink.laneLinks[laneLinkIndex++];
                        int startLaneIndex = getJsonMemberInt("startLaneIndex", laneLinkValue);
                        int endLaneIndex = getJsonMemberInt("endLaneIndex", laneLinkValue);
                        if (startLaneIndex >= static_cast<int>(roadLink.startRoad->lanes.size()) || startLaneIndex < 0)
                            throw jsonFormatError("startLaneIndex out of range");
                        if (endLaneIndex >= static_cast<int>(roadLink.endRoad->lanes.size()) || endLaneIndex < 0)
                            throw jsonFormatError("startLaneIndex out of range");
                        Lane *startLane = &roadLink.startRoad->lanes[startLaneIndex];
                        Lane *endLane = &roadLink.endRoad->lanes[endLaneIndex];

                        auto iter = laneLinkValue.FindMember("points");
                        if (iter != laneLinkValue.MemberEnd() && !iter->value.IsArray())
                            throw jsonTypeError("points in laneLink", "array");
                        if (iter != laneLinkValue.MemberEnd() && iter->value.Empty())
                            for (const auto &pValue : iter->value.GetArray()) {
                                laneLink.points.emplace_back(getJsonMemberDouble("x", pValue),
                                                             getJsonMemberDouble("y", pValue));
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
                const auto &trafficLightValue = getJsonMemberObject("trafficLight", curInterValue);
                const auto &lightPhasesValue = getJsonMemberArray("lightphases", trafficLightValue);
                for (const auto &lightPhaseValue : lightPhasesValue.GetArray()) {
                    if (!lightPhaseValue.IsObject())
                        throw jsonTypeError("lightphase", "object");
                    LightPhase lightPhase;
//                    lightPhase.phase = static_cast<unsigned int>(getJsonMemberInt("phase", lightPhaseValue));
                    lightPhase.time = getJsonMemberDouble("time", lightPhaseValue);
                    lightPhase.roadLinkAvailable = std::vector<bool>(intersections[i].roadLinks.size(), false);
                    const auto& availableRoadLinksValue =
                            getJsonMemberArray("availableRoadLinks", lightPhaseValue);
                    for (rapidjson::SizeType index = 0; index < availableRoadLinksValue.Size(); index++) {
                        if (!availableRoadLinksValue[index].IsInt())
                            throw jsonTypeError("availableRoadLink", "int");
                        int indexInRoadLinks = availableRoadLinksValue[index].GetInt();
                        lightPhase.roadLinkAvailable[indexInRoadLinks] = true;
                    }
                    intersections[i].trafficLight.phases.push_back(lightPhase);
                }
                intersections[i].trafficLight.init(0);
            }
        }catch (const jsonFormatError &e) {
            std::cerr << e.what() << std::endl;
            return false;
        }

        for (auto &intersection : intersections)
            intersection.initCrosses();
        VehicleInfo vehicleTemplate;

        for (auto &road : roads)
            road.initLanesPoints();

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

    rapidjson::Value RoadNet::convertToJson(rapidjson::Document::AllocatorType &allocator) {
        rapidjson::Value jsonRoot(rapidjson::kObjectType);
        // write nodes
        rapidjson::Value jsonNodes(rapidjson::kArrayType);
        for (size_t i = 0; i < intersections.size(); ++i) {
            rapidjson::Value jsonNode(rapidjson::kObjectType), jsonPoint(rapidjson::kArrayType);
            rapidjson::Value idValue;
            idValue.SetString(rapidjson::StringRef(intersections[i].id.c_str()));
            jsonNode.AddMember("id", idValue, allocator);
            jsonPoint.PushBack(intersections[i].point.x, allocator);
            jsonPoint.PushBack(intersections[i].point.y, allocator);
            jsonNode.AddMember("point", jsonPoint, allocator);
            jsonNode.AddMember("virtual", intersections[i].isVirtual, allocator);
            if (!intersections[i].isVirtual) {
                jsonNode.AddMember("width", intersections[i].width, allocator);
            }

            rapidjson::Value jsonOutline(rapidjson::kArrayType);
            for (auto &point: intersections[i].getOutline()) {
                jsonOutline.PushBack(point.x, allocator);
                jsonOutline.PushBack(point.y, allocator);
            }
            jsonNode.AddMember("outline", jsonOutline, allocator);
            jsonNodes.PushBack(jsonNode, allocator);
        }
        jsonRoot.AddMember("nodes", jsonNodes, allocator);

        //write edges
        rapidjson::Value jsonEdges(rapidjson::kArrayType);
        for (size_t i = 0; i < roads.size(); ++i) {
            rapidjson::Value jsonEdge(rapidjson::kObjectType);
            rapidjson::Value jsonPoints(rapidjson::kArrayType);
            rapidjson::Value jsonLaneWidths(rapidjson::kArrayType);
            rapidjson::Value jsonDirs(rapidjson::kArrayType);

            rapidjson::Value idValue;
            idValue.SetString(rapidjson::StringRef(roads[i].id.c_str()));
            jsonEdge.AddMember("id", idValue, allocator);
            rapidjson::Value startValue;
            if (roads[i].startIntersection)
                startValue.SetString(rapidjson::StringRef(roads[i].startIntersection->id.c_str()));
            else
                startValue.SetString("null");
            jsonEdge.AddMember("from", startValue, allocator);

            rapidjson::Value endValue;
            if (roads[i].endIntersection)
                endValue.SetString(rapidjson::StringRef(roads[i].endIntersection->id.c_str()));
            else
                endValue.SetString("null");
            jsonEdge.AddMember("to", endValue, allocator);
            for (size_t j = 0; j < roads[i].points.size(); ++j) {
                rapidjson::Value jsonPoint(rapidjson::kArrayType);
                jsonPoint.PushBack(roads[i].points[j].x, allocator);
                jsonPoint.PushBack(roads[i].points[j].y, allocator);
                jsonPoints.PushBack(jsonPoint, allocator);
            }
            jsonEdge.AddMember("points", jsonPoints, allocator);
            jsonEdge.AddMember("nLane", static_cast<int>(roads[i].lanes.size()), allocator);
            for (size_t j = 0; j < roads[i].lanes.size(); ++j) {
                jsonLaneWidths.PushBack(roads[i].lanes[j].width, allocator);
            }
            jsonEdge.AddMember("laneWidths", jsonLaneWidths, allocator);
            jsonEdges.PushBack(jsonEdge, allocator);
        }
        jsonRoot.AddMember("edges", jsonEdges, allocator);
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

    std::vector<LaneLink *> Lane::getLaneLinksToRoad(const Road *road) const {
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

    double Road::getWidth() const{
        double width = 0;
        for (const auto &lane : getLanes()){
            width += lane.getWidth();
        }
        return width;
    }

    double Road::getLength() const{
        double length = 0;
        for (const auto &lane : getLanes()){
            length += lane.getLength();
        }
        return length;
    }

    double Road::averageLength() const{
        double sum = 0;
        size_t laneNum = getLanes().size();
        if (laneNum == 0) return 0;
        for (const auto &lane : getLanes()){
            sum += lane.getLength();
        }
        return sum / laneNum;
    }

    double Road::getAverageSpeed() const{
        int vehicleNum = 0;
        double speedSum = 0;
        for (const auto &lane : getLanes()) {
            vehicleNum += lane.getHistoryVehicleNum();
            speedSum += lane.getHistoryAverageSpeed();
        }
        return vehicleNum ? speedSum / vehicleNum : -1;
        // If no vehicles in history, return -1
    }

    double Road::getAverageDuration() const{
        double averageSpeed = getAverageSpeed();
        if (averageSpeed < 0) return -1;
        return averageLength() / averageSpeed;
    }

    bool Road::connectedToRoad(const Road *road) const{
        for (const auto &lane : getLanes()) {
            if (lane.getLaneLinksToRoad(road).size())
                return true;
        }
        return false;
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

            if (deltaWidth < road->averageLength()) {
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
        for (std::size_t i = 1 ; i < points.size(); ++i) {
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

    Vehicle *Lane::getVehicleBeforeDistance(double dis, size_t segmentIndex) const{
        for (int i = segmentIndex ; i >= 0 ; --i){
            auto vehs = getSegment(i)->getVehicles();
            for (auto itr = vehs.begin() ; itr != vehs.end(); ++itr){
                auto &vehicle = **itr;
                if (vehicle->getDistance() < dis) return **itr;
            }
        }

        return nullptr;
    }

    Vehicle *Lane::getVehicleAfterDistance(double dis, size_t segmentIndex) const{
        for (std::size_t i = segmentIndex ; i < getSegmentNum() ; ++i){
            auto vehs = getSegment(i)->getVehicles();
            for (auto itr = vehs.rbegin() ; itr != vehs.rend(); ++itr){
                auto &vehicle = **itr;
                if (vehicle->getDistance() >= dis) return **itr;
            }
        }
        return nullptr;
    }

    void Lane::updateHistory() {
        double speedSum = historyVehicleNum * historyAverageSpeed;
        while (history.size() > historyLen){
            historyVehicleNum -= history.front().vehicleNum;
            speedSum -= history.front().vehicleNum * history.front().averageSpeed;
            history.pop_front();
        }
        double curSpeedSum = 0;
        int vehicleNum = getVehicles().size();
        historyVehicleNum += vehicleNum;
        for (auto vehicle : getVehicles())
            curSpeedSum += vehicle->getSpeed();
        speedSum += curSpeedSum;
        history.emplace_back(vehicleNum, vehicleNum ? curSpeedSum / vehicleNum : 0);
        historyAverageSpeed = historyVehicleNum ? speedSum / historyVehicleNum : 0;
    }

    int Lane::getHistoryVehicleNum() const{
        return historyVehicleNum;
    }

    double Lane::getHistoryAverageSpeed() const{
        return historyAverageSpeed;
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

