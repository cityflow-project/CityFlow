from __future__ import division, print_function
import sys
import argparse
import json
import math
import numpy as np

parser = argparse.ArgumentParser(description='Generate roadnet JSON from a grid\
            network JSON file.')
parser.add_argument('--config', dest='config', help='grid config file', type=str)

dx = [1, 0, -1, 0]
dy = [0, 1, 0, -1]

def isHorizontal(road):
    return dx[road["direction"]] != 0

def isVertical(k):
    return dy[k] == 0

def pointToDict(x, y):
    return {"x": x, "y": y}

def pointToDict2(p):
    return {"x": float(p.x), "y": float(p.y)}

def pointToDict3(p):
    return {"x": p[0], "y": p[1]}

def getLaneShift(road, laneIndex):
    shift = 0.
    for i in range(laneIndex):
        shift += road["lanes"][i]["width"]
    shift += road["lanes"][laneIndex]["width"] * .5
    return shift

def getRoadUnitVector(road):
    startPoint = road["points"][0]
    endPoint = road["points"][-1]
    dx = endPoint["x"] - startPoint["x"]
    dy = endPoint["y"] - startPoint["y"]
    length = math.sqrt(dx * dx + dy * dy)
    # print(length, dx, dy)
    return dx / length, dy / length

def getOutPoint(road, width, laneIndex):
    dx, dy = getRoadUnitVector(road)
    laneShift = getLaneShift(road, laneIndex)
    point = road["points"][-1]
    x, y = point["x"], point["y"]
    x, y = x - dx * width, y - dy * width
    x, y = x + dy * laneShift, y - dx * laneShift
    return x, y

def getOutTurnPoints(road, lim, laneIndex, width):
    dx, dy = getRoadUnitVector(road)
    laneShift = getLaneShift(road, laneIndex)
    # distance = width * .5
    point = road["points"][-1]
    x, y = point["x"], point["y"]
    x, y = x - dx * width, y - dy * width
    x, y = x + dy * laneShift, y - dx * laneShift
    return [pointToDict(x, y),
            pointToDict(x + dx * lim, y + dy * lim)]

def getInPoint(road, width, laneIndex):
    dx, dy = getRoadUnitVector(road)
    laneShift = getLaneShift(road, laneIndex)
    point = road["points"][0]
    x, y = point["x"], point["y"]
    x, y = x + dx * width, y + dy * width
    x, y = x + dy * laneShift, y - dx * laneShift
    return x, y

def getInTurnPoints(road, lim, laneIndex, width):
    dx, dy = getRoadUnitVector(road)
    laneShift = getLaneShift(road, laneIndex)
    # distance = width * .5
    point = road["points"][0]
    x, y = point["x"], point["y"]
    x, y = x + dx * width, y + dy * width
    x, y = x + dy * laneShift, y - dx * laneShift
    return [pointToDict(x - dx * lim, y - dy * lim),
            pointToDict(x, y),]

# Hermite Spline
def findPath(roada, lanea, roadb, laneb, width, midPoint=10):
    # def scale(k, p):
    #     return p.scale(k, k)

    dxa, dya = getRoadUnitVector(roada)
    dxb, dyb = getRoadUnitVector(roadb)
    pxa, pya = getOutPoint(roada, width, lanea)
    pxb, pyb = getInPoint(roadb, width, laneb)

    # pa = Point(pxa, pya)
    # pb = Point(pxb, pyb)
    dxa = dxa * width
    dya = dya * width
    dxb = dxb * width
    dyb = dyb * width
    # da = scale(width, Point(dxa, dya))
    # db = scale(width, Point(dxb, dyb))

    path = []
    for i in range(midPoint + 1):
        t = i / midPoint
        t3 = t * t * t
        t2 = t * t

        k1 = 2 * t3 - 3 * t2 + 1
        x1 = k1 * pxa
        y1 = k1 * pya

        k2 = t3 - 2 * t2 + t
        x2 = k2 * dxa
        y2 = k2 * dya

        k3 = -2 * t3 + 3 * t2
        x3 = k3 * pxb
        y3 = k3 * pyb

        k4 = t3 - t2
        x4 = k4 * dxb
        y4 = k4 * dyb

        path.append([x1 + x2 + x3 + x4, y1 + y2 + y3 + y4])
        # path.append((scale(2 * t3 - 3 * t2 + 1, pa) +
        #             scale(t3 - 2 * t2 + t, da) +
        #             scale(-2 * t3 + 3 * t2, pb) +
        #             scale(t3 - t2, db)).evalf())
    # path.append(pb)

    # print(path)

    return list(map(pointToDict3, path))

def findPathSimple(roada, lanea, roadb, laneb, width):
    dxa, dya = getRoadUnitVector(roada)
    dxb, dyb = getRoadUnitVector(roadb)
    pxa, pya = getOutPoint(roada, width, lanea)
    pxb, pyb = getInPoint(roadb, width, laneb)

    return [pointToDict(pxa, pya), pointToDict(pxa + dxa * width / 2, pya + dya * width / 2),
            pointToDict(pxb - dxb * width / 2, pyb - dyb * width / 2), pointToDict(pxb, pyb)]

def decideType(roada, roadb):
    da = roada["direction"]
    db = roadb["direction"]
    if (da + 1) % 4 == db:
        return "turn_left"
    elif (db + 1) % 4 == da:
        return "turn_right"
    elif da == db:
        return "go_straight"
    else:
        raise ValueError

def checkIntersection(interseciontName, isTruelyInside):
    return interseciontName
    # _, i, j = interseciontName.split('_')
    # i, j = int(i), int(j)
    # if isTruelyInside(i, j):
    #     return interseciontName
    # else:
    #     return None

def gridToRoadnet(rowNumber, columnNumber, rowDistances, columnDistances, outRowDistance, outColumnDistance,
                  intersectionWidths, laneWidth=4, laneMaxSpeed=20,
                  numLeftLanes=1, numStraightLanes=1, numRightLanes=1, tlPlan=False, midPoints=10):

    rowNumber += 2
    columnNumber += 2
    numLanes = numLeftLanes + numStraightLanes + numRightLanes
    intersectionWidths = [[0] * columnNumber] + intersectionWidths + [[0] * columnNumber]
    for i in range(1, rowNumber - 1):
        intersectionWidths[i] = [0] + intersectionWidths[i] + [0]

    def isInside(i, j):
        return i >= 0 and j >= 0 and i < rowNumber and j < columnNumber

    def isTruelyInside(i, j):
        return i > 0 and j > 0 and i < rowNumber - 1 and j < columnNumber - 1

    def isCorner(i, j):
        return (i == 0 or i == rowNumber - 1) and (j == 0 or j == columnNumber - 1)

    def isEdge(i, j):
        return isInside(i, j) and not isTruelyInside(i, j)

    def shouldDraw(road):
        return isTruelyInside(road["fromi"], road["fromj"]) or isTruelyInside(road["toi"], road["toj"])

    def isLeftLane(index):
        return 0 <= index < numLeftLanes

    def isStraightLane(index):
        return numLeftLanes <= index < numLeftLanes + numStraightLanes

    def isRightLane(index):
        return numLeftLanes + numStraightLanes <= index < numLanes

    x = [[None for _ in range(columnNumber)] for _ in range(rowNumber)]
    y = [[None for _ in range(columnNumber)] for _ in range(rowNumber)]

    rowDistances = [outRowDistance] + rowDistances + [outRowDistance]
    columnDistances = [outColumnDistance] + columnDistances + [outColumnDistance]

    for i in range(rowNumber):
        for j in range(columnNumber):
            if j > 0:
                x[i][j] = x[i][j - 1] + rowDistances[i - 1]
                y[i][j] = y[i][j - 1]
            elif i > 0:
                x[i][j] = x[i - 1][j]
                y[i][j] = y[i - 1][j] + columnDistances[j - 1]
            else:
                x[i][j] = -outRowDistance
                y[i][j] = -outColumnDistance

    roads = [[[None, None, None, None] for _ in range(columnNumber)] for _ in range(rowNumber)]
    for i in range(rowNumber):
        for j in range(columnNumber):
            for k in range(4):
                ni, nj = i + dy[k], j + dx[k]
                if not isInside(ni, nj):
                    road = None
                else:
                    road = {
                        "id": "road_%d_%d_%d" % (j, i, k),
                        "direction": k,
                        "fromi": i,
                        "fromj": j,
                        "toi": ni,
                        "toj": nj,
                        "points": [
                            pointToDict(x[i][j], y[i][j]),
                            pointToDict(x[ni][nj], y[ni][nj])
                        ],
                        "lanes": [
                            {
                                "width": laneWidth,
                                "maxSpeed": laneMaxSpeed
                            }
                        ] * numLanes,
                        "startIntersection": "intersection_%d_%d" % (j, i),
                        "endIntersection": "intersection_%d_%d" % (nj, ni)
                    }
                roads[i][j][k] = road

    intersections = [[None for _ in range(columnNumber)] for _ in range(rowNumber)]
    for i in range(rowNumber):
        for j in range(columnNumber):
            width = intersectionWidths[i][j]
            intersection = {
                "id": "intersection_%d_%d" % (j, i),
                "point": pointToDict(x[i][j], y[i][j]),
                "width": width,
                "roads": [],
                "roadLinks": [],
                "trafficLight": {
                    "roadLinkIndices": [],
                    "lightphases": []
                },
                "virtual": not isTruelyInside(i, j)
            }
            
            roadLinks = intersection["roadLinks"]
            roadLinkIndices = intersection["trafficLight"]["roadLinkIndices"]

            outRoads = list(filter(lambda x: x is not None, roads[i][j]))
            inRoads = list(map(lambda k: roads[i - dy[k]][j - dx[k]][k],
                    filter(lambda k: isInside(i - dy[k], j - dx[k]), range(4))))
            outRoads = list(filter(shouldDraw, outRoads))
            inRoads = list(filter(shouldDraw, inRoads))

            for road in inRoads + outRoads:
                intersection["roads"].append(road["id"])

            for a in range(len(inRoads)):
                for b in range(len(outRoads)):
                    try:
                        roada = inRoads[a]
                        roadb = outRoads[b]
                        roadLink = {
                            "type": decideType(roada, roadb),
                            "startRoad": roada["id"],
                            "endRoad": roadb["id"],
                            "direction": roada["direction"],
                            "laneLinks": []
                        }
                        for c in range(len(roada["lanes"])):
                            if roadLink["type"] == "turn_left" and not isLeftLane(c):
                                continue
                            if roadLink["type"] == "go_straight" and not isStraightLane(c):
                                continue
                            if roadLink["type"] == "turn_right" and not isRightLane(c):
                                continue
                            for d in range(len(roadb["lanes"])):
                                path = {
                                    "startLaneIndex": c,
                                    "endLaneIndex": d,
                                    "points": findPath(roada, c, roadb, d, width, midPoints)
                                }
                                roadLink["laneLinks"].append(path)
                        if roadLink["laneLinks"]:
                            roadLinkIndices.append(len(roadLinks))
                            roadLinks.append(roadLink)
                    except ValueError:
                        pass
            
            leftLaneLinks = set(filter(lambda x: roadLinks[x]["type"] == "turn_left", roadLinkIndices))
            rightLaneLinks = set(filter(lambda x: roadLinks[x]["type"] == "turn_right", roadLinkIndices))
            straightLaneLinks = set(filter(lambda x: roadLinks[x]["type"] == "go_straight", roadLinkIndices))
            WELaneLinks = set(filter(lambda x: roadLinks[x]["direction"] == 0, roadLinkIndices))
            NSLaneLinks = set(filter(lambda x: roadLinks[x]["direction"] == 1, roadLinkIndices))
            EWLaneLinks = set(filter(lambda x: roadLinks[x]["direction"] == 2, roadLinkIndices))
            SNLaneLinks = set(filter(lambda x: roadLinks[x]["direction"] == 3, roadLinkIndices))
            
            tlPhases = intersection["trafficLight"]["lightphases"]
            if not tlPlan:
                tlPhases.append({
                    "time": 5,
                    "availableRoadLinks": rightLaneLinks
                })
                tlPhases.append({
                    "time": 30,
                    "availableRoadLinks": ((EWLaneLinks | WELaneLinks) & straightLaneLinks) | (rightLaneLinks)
                })
                tlPhases.append({
                    "time": 30,
                    "availableRoadLinks": ((NSLaneLinks | SNLaneLinks) & straightLaneLinks) | (rightLaneLinks)
                })
                tlPhases.append({
                    "time": 30,
                    "availableRoadLinks": ((EWLaneLinks | WELaneLinks) & leftLaneLinks) | (rightLaneLinks)
                })
                tlPhases.append({
                    "time": 30,
                    "availableRoadLinks": ((SNLaneLinks | NSLaneLinks) & leftLaneLinks) | (rightLaneLinks)
                })
                tlPhases.append({
                    "time": 30,
                    "availableRoadLinks": (WELaneLinks) | (rightLaneLinks)
                })
                tlPhases.append({
                    "time": 30,
                    "availableRoadLinks": (EWLaneLinks) | (rightLaneLinks)
                })
                tlPhases.append({
                    "time": 30,
                    "availableRoadLinks": (NSLaneLinks) | (rightLaneLinks)
                })
                tlPhases.append({
                    "time": 30,
                    "availableRoadLinks": (SNLaneLinks) | (rightLaneLinks)
                })
            else:
                tlPhases.append({
                    "time": 30,
                    "availableRoadLinks": ((EWLaneLinks | WELaneLinks) & straightLaneLinks) | (rightLaneLinks)
                })
                tlPhases.append({
                    "time": 5,
                    "availableRoadLinks": rightLaneLinks
                })
                if numLeftLanes:
                    tlPhases.append({
                        "time": 30,
                        "availableRoadLinks": ((EWLaneLinks | WELaneLinks) & leftLaneLinks) | (rightLaneLinks)
                    })
                    tlPhases.append({
                        "time": 5,
                        "availableRoadLinks": rightLaneLinks
                    })
                tlPhases.append({
                    "time": 30,
                    "availableRoadLinks": ((NSLaneLinks | SNLaneLinks) & straightLaneLinks) | (rightLaneLinks)
                })
                tlPhases.append({
                    "time": 5,
                    "availableRoadLinks": rightLaneLinks
                })
                if numLeftLanes:
                    tlPhases.append({
                        "time": 30,
                        "availableRoadLinks": ((SNLaneLinks | NSLaneLinks) & leftLaneLinks) | (rightLaneLinks)
                    })
                    tlPhases.append({
                        "time": 5,
                        "availableRoadLinks": rightLaneLinks
                    })
            for tlPhase in tlPhases:
                tlPhase["availableRoadLinks"] = list(tlPhase["availableRoadLinks"])
            intersections[i][j] = intersection

    final_intersecions = []
    for i in range(rowNumber):
        for j in range(columnNumber):
            if not isCorner(i, j):
                # print(i, j)
                final_intersecions.append(intersections[i][j])

    final_roads = []
    for i in range(rowNumber):
        for j in range(columnNumber):
            for k in range(4):
                if roads[i][j][k] is not None and shouldDraw(roads[i][j][k]):
                    ni, nj = i + dy[k], j + dx[k]
                    roads[i][j][k].pop("direction")
                    roads[i][j][k].pop("fromi")
                    roads[i][j][k].pop("fromj")
                    roads[i][j][k].pop("toi")
                    roads[i][j][k].pop("toj")
                    roads[i][j][k]["startIntersection"] = checkIntersection(roads[i][j][k]["startIntersection"], isTruelyInside)
                    roads[i][j][k]["endIntersection"] = checkIntersection(roads[i][j][k]["endIntersection"], isTruelyInside)
                    final_roads.append(roads[i][j][k])

    return {
        "intersections": final_intersecions,
        "roads": final_roads
    }

if __name__ == "__main__":
    args = parser.parse_args()
    path = '../data/roadnet/'
    grid_config = json.load(open(path + args.config if args.config is not None else path + 'grid66.json'))
    json.dump(gridToRoadnet(**grid_config), open(path + "roadnet66.json", "w"), indent=2)
