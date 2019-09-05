import argparse
import json
import os
from generate_json_from_grid import gridToRoadnet

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("rowNum", type=int)
    parser.add_argument("colNum", type=int)
    parser.add_argument("--rowDistance", type=int, default=300)
    parser.add_argument("--columnDistance", type=int, default=300)
    parser.add_argument("--intersectionWidth", type=int, default=30)
    parser.add_argument("--numLeftLanes", type=int, default=1)
    parser.add_argument("--numStraightLanes", type=int, default=1)
    parser.add_argument("--numRightLanes", type=int, default=1)
    parser.add_argument("--laneMaxSpeed", type=float, default=16.67)
    parser.add_argument("--vehLen", type=float, default=5.0)
    parser.add_argument("--vehWidth", type=float, default=2.0)
    parser.add_argument("--vehMaxPosAcc", type=float, default=2.0)
    parser.add_argument("--vehMaxNegAcc", type=float, default=4.5)
    parser.add_argument("--vehUsualPosAcc", type=float, default=2.0)
    parser.add_argument("--vehUsualNegAcc", type=float, default=4.5)
    parser.add_argument("--vehMinGap", type=float, default=2.5)
    parser.add_argument("--vehMaxSpeed", type=float, default=16.67)
    parser.add_argument("--vehHeadwayTime", type=float, default=1.5)
    parser.add_argument("--dir", type=str, default="./")
    parser.add_argument("--roadnetFile", type=str)
    parser.add_argument("--turn", action="store_true")
    parser.add_argument("--tlPlan", action="store_true")
    parser.add_argument("--interval", type=float, default=2.0)
    parser.add_argument("--flowFile", type=str)
    return parser.parse_args()

def generate_route(rowNum, colNum, turn=False):
    routes = []
    move = [(1, 0), (0, 1), (-1, 0), (0, -1)]

    def get_straight_route(start, direction, step):
        x, y = start
        route = []
        for _ in range(step):
            route.append("road_%d_%d_%d" % (x, y, direction))
            x += move[direction][0]
            y += move[direction][1]
        return route

    for i in range(1, rowNum+1):
        routes.append(get_straight_route((0, i), 0, colNum+1))
        routes.append(get_straight_route((colNum+1, i), 2, colNum+1))
    for i in range(1, colNum+1):
        routes.append(get_straight_route((i, 0), 1, rowNum+1))
        routes.append(get_straight_route((i, rowNum+1), 3, rowNum+1))
    
    if turn:
        def get_turn_route(start, direction):
            if direction[0] % 2 == 0:
                step = min(rowNum*2, colNum*2+1)
            else:
                step = min(colNum*2, rowNum*2+1)
            x, y = start
            route = []
            cur = 0
            for _ in range(step):
                route.append("road_%d_%d_%d" % (x, y, direction[cur]))
                x += move[direction[cur]][0]
                y += move[direction[cur]][1]
                cur = 1 - cur
            return route

        routes.append(get_turn_route((1, 0), (1, 0)))
        routes.append(get_turn_route((0, 1), (0, 1)))
        routes.append(get_turn_route((colNum+1, rowNum), (2, 3)))
        routes.append(get_turn_route((colNum, rowNum+1), (3, 2)))
        routes.append(get_turn_route((0, rowNum), (0, 3)))
        routes.append(get_turn_route((1, rowNum+1), (3, 0)))
        routes.append(get_turn_route((colNum+1, 1), (2, 1)))
        routes.append(get_turn_route((colNum, 0), (1, 2)))
    
    return routes

if __name__ == '__main__':
    args = parse_args()
    if args.roadnetFile is None:
        args.roadnetFile = "roadnet_%d_%d%s.json" % (args.rowNum, args.colNum, "_turn" if args.turn else "")
    if args.flowFile is None:
        args.flowFile = "flow_%d_%d%s.json" % (args.rowNum, args.colNum, "_turn" if args.turn else "")

    grid = {
        "rowNumber": args.rowNum,
        "columnNumber": args.colNum,
        "rowDistances": [args.rowDistance] * (args.colNum-1),
        "columnDistances": [args.columnDistance] * (args.rowNum-1),
        "outRowDistance": args.rowDistance,
        "outColumnDistance": args.columnDistance,
        "intersectionWidths": [[args.intersectionWidth] * args.colNum] * args.rowNum,
        "numLeftLanes": args.numLeftLanes,
        "numStraightLanes": args.numStraightLanes,
        "numRightLanes": args.numRightLanes,
        "laneMaxSpeed": args.laneMaxSpeed,
        "tlPlan": args.tlPlan
    }

    json.dump(gridToRoadnet(**grid), open(os.path.join(args.dir, args.roadnetFile), "w"), indent=2)
    
    vehicle_template = {
        "length": args.vehLen,
        "width": args.vehWidth,
        "maxPosAcc": args.vehMaxPosAcc,
        "maxNegAcc": args.vehMaxNegAcc,
        "usualPosAcc": args.vehUsualPosAcc,
        "usualNegAcc": args.vehUsualNegAcc,
        "minGap": args.vehMinGap,
        "maxSpeed": args.vehMaxSpeed,
        "headwayTime": args.vehHeadwayTime
    }
    routes = generate_route(args.rowNum, args.colNum, args.turn)
    flow = []
    for route in routes:
        flow.append({
            "vehicle": vehicle_template,
            "route": route,
            "interval": args.interval,
            "startTime": 0,
            "endTime": -1
        })
    json.dump(flow, open(os.path.join(args.dir, args.flowFile), "w"), indent=2)

