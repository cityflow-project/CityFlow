import argparse
import json
import xml.dom.minidom
import copy
import pyproj as proj
import folium

intersections = []
roads = []
node_dic = {}
way_dic = {}
road_dic = {}
lanes_dic = {}
way_direction = {}
cross = {}
min_lat = 1000.0
min_lon = 1000.0


def geo_converter(lon, lat):
    crs_wgs = proj.Proj(init='epsg:4326')
    crs_bng = proj.Proj(init='epsg:8826')

    x1, y1 = proj.transform(crs_wgs, crs_bng, lon, lat)
    x2, y2 = proj.transform(crs_wgs, crs_bng, min_lon, min_lat)
    return (x1 - x2), (y1 - y2)


def build_virtual_intersection(nodeID, id):
    node = node_dic[nodeID]
    x, y = geo_converter(float(node.getAttribute('lon')), float(node.getAttribute('lat')))
    intersection = {
        "id": nodeID,
        "point": {"x": x, "y": y},
        "width": 0,
        "roads": [id],
        "roadLinks": [],
        "trafficLight": {
            "roadLinkIndices": [],
            "lightphases": []
        },
        "virtual": True
    }
    intersections.append(intersection)


def build_road(wayID, maxSpeed, lanes, points, id, startId, endID):
    road = {
        "id": id,
        "points": points,
        "lanes": [
        ],
        "startIntersection": startId,
        "endIntersection": endID
    }
    lane_template = {
        "width": 3.7,
        "maxSpeed": maxSpeed
    }
    for _ in range(lanes):
        road["lanes"].append(lane_template)
    roads.append(road)
    road_dic[id] = road
    lanes_dic[id] = lanes
    way = way_dic[wayID]
    ndlist = way.getElementsByTagName('nd')
    if endID == ndlist[0].getAttribute('ref') or endID == ndlist[len(ndlist) - 1].getAttribute('ref'):
        if endID not in cross or len(cross[endID]) <= 1:
            build_virtual_intersection(endID, id)
    if startId == ndlist[0].getAttribute('ref') or startId == ndlist[len(ndlist) - 1].getAttribute('ref'):
        if startId not in cross or len(cross[startId]) <= 1:
            build_virtual_intersection(startId, id)


def get_tag(taglist):
    numLanes = -1
    oneDirectionLanes = -1
    otherDirectionLanes = -1
    oneway = False
    maxSpeed = 0
    for tag in taglist:
        key = tag.getAttribute('k')
        value = tag.getAttribute('v')
        if key == 'lanes':
            numLanes = int(value)
        if key == 'lanes:forward':
            oneDirectionLanes = int(value)
        if key == 'lanes:backward':
            otherDirectionLanes = int(value)
        if key == 'oneway' and value == 'yes':
            oneway = True
        if key == 'maxspeed':
            maxSpeed = float(value[0:-3]) * 1.609344
    return numLanes, oneDirectionLanes, otherDirectionLanes, oneway, maxSpeed


def build_not_one_way_roads(crossID, in_roadIDs, out_roadIDs, nodeID, endID, points, otherPoints, pos, endPos, wayID, numLanes,
                            oneDirectionLanes, otherDirectionLanes, oneway, maxSpeed):
    if oneDirectionLanes < 0 and otherDirectionLanes < 0:
        oneDirectionLanes = int(numLanes / 2)
        otherDirectionLanes = int(numLanes / 2)
    elif oneDirectionLanes < 0:
        oneDirectionLanes = numLanes - otherDirectionLanes
    elif otherDirectionLanes < 0:
        otherDirectionLanes = numLanes - oneDirectionLanes
    if oneDirectionLanes > 0:
        name = str(wayID + '_' + str(pos) + '_' + str(endPos))
        if name not in road_dic:
            build_road(wayID, maxSpeed, oneDirectionLanes, points,
                       name, nodeID, endID)
        if road_dic[name]["endIntersection"] == crossID:
            in_roadIDs.append(name)
        else:
            out_roadIDs.append(name)
    if otherDirectionLanes > 0:
        name = str(wayID + '_' + str(endPos) + '_' + str(pos))
        if name not in road_dic:
            build_road(wayID, maxSpeed, otherDirectionLanes, otherPoints,
                       name, endID, nodeID)
        if road_dic[name]["endIntersection"] == crossID:
            in_roadIDs.append(name)
        else:
            out_roadIDs.append(name)
    return in_roadIDs, out_roadIDs

def build_one_way_roads(crossID, in_roadIDs, out_roadIDs, nodeID, endID, points, otherPoints, pos, endPos, wayID, numLanes,
                            oneDirectionLanes, otherDirectionLanes, oneway, maxSpeed):
    if numLanes > 0:
        name = str(wayID + '_' + str(pos) + '_' + str(endPos))
        if name not in road_dic:
            build_road(wayID, maxSpeed, numLanes, points,
                       name, nodeID, endID)
        if road_dic[name]["endIntersection"] == crossID:
            in_roadIDs.append(name)
        else:
            out_roadIDs.append(name)
    return in_roadIDs, out_roadIDs


def cross_to_roads(nodeID, wayID):
    in_roadIDs = []
    out_roadIDs = []
    way = way_dic[wayID]
    points = []
    ndlist = way.getElementsByTagName('nd')
    pos = 0
    length = 0
    for nd in ndlist:
        nd_id = nd.getAttribute('ref')
        if nd_id == nodeID:
            pos = length
            break
        length = length + 1
    # three cases: cross is in the start, end and middle of a road
    if pos == 0:  # start
        endPos = len(ndlist) - 1
        endID = ndlist[endPos].getAttribute('id')
        for i in range(len(ndlist)):
            nd = ndlist[i]
            nd_id = nd.getAttribute('ref')
            node = node_dic[nd_id]
            x, y = geo_converter(float(node.getAttribute('lon')), float(node.getAttribute('lat')))
            points.append({"x": x,
                           "y": y})
            if i == pos:
                continue
            endID = nd_id
            endPos = i
            if nd_id in cross and len(cross[nd_id]) > 1:
                break
        otherPoints = []
        for i in range(len(points)):
            otherPoints.append(points[len(points) - i - 1])
        taglist = way.getElementsByTagName('tag')
        numLanes, oneDirectionLanes, otherDirectionLanes, oneway, maxSpeed = get_tag(taglist)
        if not oneway:
            in_roadIDs, out_roadIDs = build_not_one_way_roads(nodeID, in_roadIDs, out_roadIDs, nodeID, endID, points, otherPoints, pos, endPos,
                                                          wayID, numLanes, oneDirectionLanes, otherDirectionLanes, oneway, maxSpeed)
        else:
            in_roadIDs, out_roadIDs = build_one_way_roads(nodeID, in_roadIDs, out_roadIDs, nodeID, endID, points,
                                                              otherPoints, pos, endPos,
                                                              wayID, numLanes, oneDirectionLanes, otherDirectionLanes,
                                                              oneway, maxSpeed)
    elif pos == len(ndlist) - 1:  # end
        endPos = 0
        endID = ndlist[endPos].getAttribute('id')
        for i in range(len(ndlist)):
            nd = ndlist[len(ndlist) - i - 1]
            nd_id = nd.getAttribute('ref')
            node = node_dic[nd_id]
            x, y = geo_converter(float(node.getAttribute('lon')), float(node.getAttribute('lat')))
            points.append({"x": x,
                           "y": y})
            if pos == len(ndlist) - i - 1:
                continue
            endID = nd_id
            endPos = len(ndlist) - i - 1
            if nd_id in cross and len(cross[nd_id]) > 1:
                break
        otherPoints = []
        for i in range(len(points)):
            otherPoints.append(points[len(points) - i - 1])
        taglist = way.getElementsByTagName('tag')
        numLanes, oneDirectionLanes, otherDirectionLanes, oneway, maxSpeed = get_tag(taglist)
        if not oneway:
            in_roadIDs, out_roadIDs = build_not_one_way_roads(nodeID, in_roadIDs, out_roadIDs, endID, nodeID, otherPoints, points, pos, endPos,
                                                          wayID, numLanes, otherDirectionLanes, oneDirectionLanes, oneway, maxSpeed)
        else:
            in_roadIDs, out_roadIDs = build_one_way_roads(nodeID, in_roadIDs, out_roadIDs, endID, nodeID, otherPoints,
                                                              points, endPos, pos,
                                                              wayID, numLanes, otherDirectionLanes, oneDirectionLanes,
                                                              oneway, maxSpeed)
    # middle, the road will be divide into two sub-roads, cross in the start of one road and the end in the other road.
    else:
        endPos = len(ndlist) - 1
        endID = ndlist[endPos].getAttribute('id')
        for i in range(len(ndlist) - pos):
            nd = ndlist[pos + i]
            nd_id = nd.getAttribute('ref')
            node = node_dic[nd_id]
            x, y = geo_converter(float(node.getAttribute('lon')), float(node.getAttribute('lat')))
            points.append({"x": x,
                           "y": y})
            if pos + i == pos:
                continue
            endID = nd_id
            endPos = pos + i
            if nd_id in cross and len(cross[nd_id]) > 1:
                break
        otherPoints = []
        for i in range(len(points)):
            otherPoints.append(points[len(points) - i - 1])
        taglist = way.getElementsByTagName('tag')
        numLanes, oneDirectionLanes, otherDirectionLanes, oneway, maxSpeed = get_tag(taglist)
        if not oneway:
            in_roadIDs, out_roadIDs = build_not_one_way_roads(nodeID, in_roadIDs, out_roadIDs, nodeID, endID, points, otherPoints, pos, endPos,
                                                          wayID, numLanes, oneDirectionLanes, otherDirectionLanes, oneway, maxSpeed)
        else:
            in_roadIDs, out_roadIDs = build_one_way_roads(nodeID, in_roadIDs, out_roadIDs, nodeID, endID, points,
                                                              otherPoints, pos, endPos,
                                                              wayID, numLanes, oneDirectionLanes, otherDirectionLanes,
                                                              oneway, maxSpeed)
        points = []
        endPos = 0
        endID = ndlist[endPos].getAttribute('id')

        for i in range(pos + 1):
            nd = ndlist[pos - i]
            nd_id = nd.getAttribute('ref')
            node = node_dic[nd_id]
            x, y = geo_converter(float(node.getAttribute('lon')), float(node.getAttribute('lat')))
            points.append({"x": x,
                           "y": y})
            if pos - i == pos:
                continue
            endID = nd_id
            endPos = pos - i
            if nd_id in cross and len(cross[nd_id]) > 1:
                break
        otherPoints = []
        for i in range(len(points)):
            otherPoints.append(points[len(points) - i - 1])
        taglist = way.getElementsByTagName('tag')
        numLanes, oneDirectionLanes, otherDirectionLanes, oneway, maxSpeed = get_tag(taglist)
        if not oneway:
            in_roadIDs, out_roadIDs = build_not_one_way_roads(nodeID, in_roadIDs, out_roadIDs, endID, nodeID, otherPoints, points, pos, endPos,
                                                          wayID, numLanes, otherDirectionLanes, oneDirectionLanes, oneway, maxSpeed)
        else:
            in_roadIDs, out_roadIDs = build_one_way_roads(nodeID, in_roadIDs, out_roadIDs, endID, nodeID, otherPoints,
                                                              points, endPos, pos,
                                                              wayID, numLanes, otherDirectionLanes, oneDirectionLanes,
                                                              oneway, maxSpeed)
    return in_roadIDs, out_roadIDs


def node_to_intersection(nodeID, waylist):
    cross_node = node_dic[nodeID]
    x, y = geo_converter(float(cross_node.getAttribute('lon')), float(cross_node.getAttribute('lat')))
    intersection = {
        "id": nodeID,
        "point": {"x": x, "y": y},
        "width": 5,
        "roads": [],
        "roadLinks": [],
        "trafficLight": {
            "roadLinkIndices": [],
            "lightphases": []
        },
        "virtual": False
    }
    roadLink_base = {
        "type": "go_straight",
        "startRoad": "road_0_1_0",
        "endRoad": "road_1_1_0",
        "direction": 0,
        "laneLinks": [
        ]
    }
    laneLink_base = {
        "startLaneIndex": 0,
        "endLaneIndex": 0,
        "points": [
        ]
    }
    in_roads = []
    out_roads = []
    roads = []
    for wayID in waylist:
        in_roadIDs, out_roadIDs = cross_to_roads(nodeID, wayID)
        for roadId in in_roadIDs:
            intersection['roads'].append(roadId)
            in_roads.append(roadId)
            roads.append(roadId)
        for roadId in out_roadIDs:
            intersection['roads'].append(roadId)
            out_roads.append(roadId)
            roads.append(roadId)
    different_road = {}
    for road in roads:
        road_name = ""
        for c in road:
            if c == '_':
                break
            road_name = road_name + c
        if road_name not in different_road:
            different_road[road_name] = 1
    all_green = False
    if len(different_road.keys()) <= 2:
        intersection['width'] = 0
        all_green = True
    count = 0
    for roadID1 in in_roads:
        for roadID2 in out_roads:
            if roadID1 == roadID2:
                continue
            roadLink = copy.deepcopy(roadLink_base)
            roadLink['startRoad'] = roadID1
            roadLink['endRoad'] = roadID2
            intersection["trafficLight"]["roadLinkIndices"].append(count)
            count += 1
            for i in range(lanes_dic[roadID1]):
                for j in range(lanes_dic[roadID2]):
                    laneLink = copy.deepcopy(laneLink_base)
                    laneLink['startLaneIndex'] = i
                    laneLink['endLaneIndex'] = j
                    roadLink['laneLinks'].append(laneLink)
            intersection['roadLinks'].append(roadLink)
    return process_intersection_simple_phase(intersection, all_green)


def process_intersection_simple_phase(intersection, all_green):
    if intersection['virtual']:
        return intersection
    if all_green:
        all_green = {
            "time": 30,
            "availableRoadLinks": intersection['trafficLight']['roadLinkIndices']
        }
        lightphases = [all_green]
        intersection['trafficLight']['lightphases'] = lightphases
        return intersection
    else:
        all_green = {
            "time": 30,
            "availableRoadLinks": intersection['trafficLight']['roadLinkIndices']
        }
        all_red = {
            "time": 30,
            "availableRoadLinks": []
        }
        lightphases = [all_green, all_red]
        intersection['trafficLight']['lightphases'] = lightphases
        return intersection


def extract(osmFile):
    dom = xml.dom.minidom.parse(osmFile)
    root = dom.documentElement
    totalnodelist = root.getElementsByTagName('node')
    totalwaylist = root.getElementsByTagName('way')
    for node in totalnodelist:
        node_id = node.getAttribute('id')
        node_dic[node_id] = node
        global min_lat
        global min_lon
        if float(node.getAttribute('lat')) < min_lat:
            min_lat = float(node.getAttribute('lat'))
        if float(node.getAttribute('lon')) < min_lon:
            min_lon = float(node.getAttribute('lon'))

    node_dic2 = {}

    for way in totalwaylist:
        way_inside = {}
        taglist = way.getElementsByTagName('tag')
        road_flag = False
        unused_flag = True
        belong = way.getAttribute('id')
        way_dic[belong] = way
        for tag in taglist:
            if tag.getAttribute('k') == 'highway' and tag.getAttribute('v') == 'primary':
                unused_flag = False
                break
        if unused_flag:
            continue
        for tag in taglist:
            if tag.getAttribute('k') == 'highway':
                road_flag = True
                break
        if road_flag:
            ndlist = way.getElementsByTagName('nd')
            for nd in ndlist:
                nd_id = nd.getAttribute('ref')
                node_lat = node_dic[nd_id].getAttribute('lat')
                node_lon = node_dic[nd_id].getAttribute('lon')
                way_inside[nd_id] = (node_lat, node_lon, belong)
            node_dic2[belong] = way_inside

    # print(len(node_dic2))

    return node_dic2


def draw(nodes, CityFlowNet, html):
    road_map = folium.Map(zoom_start=10)
    incidents = folium.map.FeatureGroup()
    num = 0
    for wayID, way in nodes.items():
        color_list = ['red', 'blue', 'green', 'purple', 'orange', 'darkred',
                      'lightred', 'beige', 'darkblue', 'darkgreen', 'cadetblue',
                      'darkpurple', 'white', 'pink', 'lightblue', 'lightgreen',
                      'gray', 'black', 'lightgray']
        color = color_list[int(wayID) % (len(color_list) - 1) + 1]
        way_num = 0
        name = ''
        taglist = way_dic[wayID].getElementsByTagName('tag')
        for tag in taglist:
            if tag.getAttribute('k') == 'name':
                name = tag.getAttribute('v')

        for nodeID, node in way.items():
            if nodeID not in cross:
                cross[nodeID] = [node[2]]
            else:
                cross[nodeID].append(node[2])
            incidents.add_child(
                folium.CircleMarker(
                    (node[0], node[1]),
                    radius=7,
                    color=color,
                    fill=True,
                    fill_color=color,
                    fill_opacity=1,
                    popup=name + '__' + str(way_num) + '\n' + way_dic[wayID].getAttribute('id')
                )
            )
            way_num = way_num + 1
        num = num + way_num
    print('total roads: ', num, ' total crosses: ', len(cross))
    road_map.add_child(incidents)
    if html:
        road_map.save("road.html")
    incidents = folium.map.FeatureGroup()
    cross_map = folium.Map(zoom_start=15)
    count = 0
    for nodeID, way_list in cross.items():
        count += 1
        if count % 100 == 0:
            print(count)
        if len(way_list) > 1:
            intersection = node_to_intersection(nodeID, way_list)
            intersections.append(intersection)
            label = ''
            for wayID in way_list:
                label = label + '\n' + wayID
            incidents.add_child(
                folium.CircleMarker(
                    (node_dic[nodeID].getAttribute(
                        'lat'), node_dic[nodeID].getAttribute('lon')),
                    radius=7,  # define how big you want the circle markers to be
                    color='red',
                    fill=True,
                    fill_color='red',
                    fill_opacity=1,
                    popup=label
                )
            )
    cross_map.add_child(incidents)
    if html:
        cross_map.save('cross.html')

    result = {
        "intersections": intersections,
        "roads": roads
    }

    f = open(CityFlowNet, 'w')
    json.dump(result, f, indent=2)
    f.close()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--osmFile", type=str, default='map.osm')
    parser.add_argument("--CityFlowNet", type=str, default='roadnet.json')
    parser.add_argument("--html", type=bool, default=True)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    nodes = extract(args.osmFile)
    draw(nodes, args.CityFlowNet, args.html)
