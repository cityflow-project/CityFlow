import os
import sys
from sys import platform
import argparse
from collections import defaultdict
import sympy
from mpmath import degrees, radians
import copy
import math
import json

if platform == "linux" or platform == "linux2":
    # this is linux
    try:
        import traci
        import traci.constants as tc
        import sumolib
        from sumolib.net import Connection
    except ImportError:
        if "SUMO_HOME" in os.environ:
            print(os.path.join(os.environ["SUMO_HOME"], "tools"))
            sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))
            import traci
            import traci.constants as tc
            import sumolib
            from sumolib.net import Connection
        else:
            raise EnvironmentError("Please set SUMO_HOME environment variable or install traci as python module!")
elif platform == "win32":
    os.environ['SUMO_HOME'] = 'D:\\software\\sumo-0.32.0'
    try:
        import traci
        import traci.constants as tc
        import sumolib
        from sumolib.net import Connection
    except ImportError:
        if "SUMO_HOME" in os.environ:
            print(os.path.join(os.environ["SUMO_HOME"], "tools"))
            sys.path.append(
                os.path.join(os.environ["SUMO_HOME"], "tools")
            )
            import traci
            import traci.constants as tc
            import sumolib
            from sumolib.net import Connection
        else:
            raise EnvironmentError("Please set SUMO_HOME environment variable or install traci as python module!")

elif platform =='darwin':
    os.environ['SUMO_HOME'] = "/Users/{0}/sumo/".format(os.environ.get('USER'))
    print(os.environ['SUMO_HOME'])
    try:
        import traci
        import traci.constants as tc
        import sumolib
        from sumolib.net import Connection
    except ImportError:
        if "SUMO_HOME" in os.environ:
            print(os.path.join(os.environ["SUMO_HOME"], "tools"))
            sys.path.append(
                os.path.join(os.environ["SUMO_HOME"], "tools")
            )
            import traci
            import traci.constants as tc
            import sumolib
            from sumolib.net import Connection
        else:
            raise EnvironmentError("Please set SUMO_HOME environment variable or install traci as python module!")
else:
    sys.exit("platform error")

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sumonet", type=str,default='atlanta_sumo.net.xml')
    parser.add_argument("--cityflownet", type=str,default='atlanta_cityflow.json')
    return parser.parse_args()

U_TURN_AS = "turn_left"
DEBUG = False
TRUE_CORRECTION_lane = True
SUMO_PROGRAM = True


def get_direction_fron_connection(connection):
    _map = {
        Connection.LINKDIR_STRAIGHT: "go_straight",
        Connection.LINKDIR_TURN: "turn_u",
        Connection.LINKDIR_LEFT: "turn_left",
        Connection.LINKDIR_RIGHT: "turn_right",
        Connection.LINKDIR_PARTLEFT: "turn_left",
        Connection.LINKDIR_PARTRIGHT: "turn_right",
    }
    return _map[connection.getDirection()]

def process_edge(edge):
    lanes = []
    if TRUE_CORRECTION_lane:
        for inx,lane in enumerate(reversed(edge.getLanes())):
            outgoing_list = lane.getOutgoing()
            for outgoing in outgoing_list:
                new_lane = copy.copy(lane)
                direction = get_direction_fron_connection(outgoing)
                to_lane = outgoing.getToLane()
                # marky,add to_lane
                new_lane._cityflow_lane_id = f'{lane.getID()}|{to_lane.getID()}|{direction}'
                new_lane._cityflow_lane_inx = inx
                new_lane._direction = direction
                lanes.append(new_lane)
            if len(outgoing_list) == 0:
                new_lane = copy.copy(lane)
                new_lane._cityflow_lane_id = f'{lane.getID()}'
                new_lane._cityflow_lane_inx = inx
                new_lane._direction = 'go_end'
                lanes.append(new_lane)
    else:
        for lane in edge.getLanes():
            outgoing_list = lane.getOutgoing()
            for outgoing in outgoing_list:
                new_lane = copy.copy(lane)
                direction = get_direction_fron_connection(outgoing)
                to_lane = outgoing.getToLane()
                new_lane._cityflow_lane_id = f'{lane.getID()}|{to_lane.getID()}|{direction}'
                new_lane._direction = direction
                lanes.append(new_lane)
            if len(outgoing_list) == 0:
                new_lane = copy.copy(lane)
                new_lane._cityflow_lane_id = f'{lane.getID()}'
                new_lane._direction = 'go_end'
                lanes.append(new_lane)
    edge._cityflow_lanes = lanes[::-1]
    return edge




def _cityflow_get_lane_index_in_edge(lane, edge):
    for i, _lane in enumerate(edge._cityflow_lanes):
        if _lane._cityflow_lane_id == lane._cityflow_lane_id:
            return i
    raise Exception('lane in edge not found')

def _cityflow_get_lane_index_in_edge_cor(lane, edge):
    ## i = lane._cityflow_lane_id.split('|')[0]
    for i, _lane in enumerate(edge._cityflow_lanes):
        if _lane._cityflow_lane_id == lane._cityflow_lane_id:
            return _lane._cityflow_lane_inx
    raise Exception('lane in edge not found')


def point_tuple_to_dict(point_tuple):
    return {"x": point_tuple[0], "y": point_tuple[1]}


def _is_node_virtual(node):
    n = node
    edges = [edge for edge in n.getIncoming() + n.getOutgoing()]
    ids = list(set([e.getFromNode().getID() for e in edges] + [e.getToNode().getID() for e in edges]))
    if len(ids)<=2:
        return True
    else:
        return False


def group_connections_by_start_end(connections):
    connection_group_result = defaultdict(list)
    for connection in connections:
        start_road = connection.getFrom()
        end_road = connection.getTo()
        direction = get_direction_fron_connection(connection)
        key = "{}|{}|{}".format(start_road.getID(), end_road.getID(), direction)
        connection_group_result[key].append(connection)
    return connection_group_result


def calc_edge_compass_angle(edge):
    north_ray = sympy.Ray((0, 0), (0, 1))
    # 反向算进入edge，直观。
    edge_ray = sympy.Ray(*edge.getShape()[:2][::-1])
    angle = north_ray.closing_angle(edge_ray)
    angle = (angle + 2 * sympy.pi) % (2 * sympy.pi)
    angle_degrees = float(degrees(angle))
    angle_radians = float(radians(degrees(angle)))
    edge._angle_degrees = round(angle_degrees,4)
    edge._angle_radians = round(angle_radians,4)
    return angle_degrees, angle_radians


def calc_edge_compass_angle_no_modify(edge):
    north_ray = sympy.Ray((0, 0), (0, 1))
    # 要算所有edge，所以不要反向。
    edge_ray = sympy.Ray(*edge.getShape()[:2])
    angle = north_ray.closing_angle(edge_ray)
    angle = (angle + 2 * sympy.pi) % (2 * sympy.pi)
    angle_degrees = float(degrees(angle))
    # angle_radians = float(radians(degrees(angle)))
    # edge._angle_degrees = round(angle_degrees,4)
    # edge._angle_radians = round(angle_radians,4)
    return angle_degrees


def process_intersection_simple_phase(intersection):
    if intersection['virtual']:
        return intersection

    all_green = {
        "time": 30,
        "availableRoadLinks": intersection['trafficLight']['roadLinkIndices']
    }
    all_red = {
        "time": 30,
        "availableRoadLinks": []
    }
    lightphases = [all_green]
    intersection['trafficLight']['lightphases'] = lightphases
    return intersection


def _cal_angle_pair(cluster):
    centroids = cluster['centroids']
    centroids = [x[0] for x in centroids]
    if len(centroids) == 4:
        pairs = [(centroids[0], centroids[2]), (centroids[1], centroids[3])]
    elif len(centroids) == 3:
        r1 = centroids[1] - centroids[0]
        r2 = centroids[2] - centroids[0]
        r3 = centroids[2] - centroids[1]
        near180_1 = abs(180 - r1)
        near180_2 = abs(180 - r2)
        near180_3 = abs(180 - r3)
        lista = [
            ([(centroids[0], centroids[1]), (centroids[2],)],near180_1),
            ([(centroids[0], centroids[2]), (centroids[1],)],near180_2),
            ([(centroids[0],),(centroids[1], centroids[2]),],near180_3),
        ]
        pairs = min(lista,key=lambda item:item[1])[0]
    elif len(centroids) == 2:
        pairs = [(centroids[0], centroids[1]), ]
    elif len(centroids) == 1:
        pairs = [(centroids[0],),]
    return pairs


def find_edges_by_angle(all_edges,angle):
    edges = []
    for edge in all_edges:
        if math.isclose(edge._angle_degrees , angle, abs_tol=0.0001):
        # if edge._angle_degrees == angle:
            edges.append(edge)
    if not edges:
        raise Exception('!!!no edge._angle_degrees = angle')
    return edges

def find_edges_by_cluster_centroid(all_edges,angle):
    edges = []
    for edge in all_edges:
        if math.isclose(edge._cluster_centroid[0] , angle, abs_tol=0.0001):
        # if edge._angle_degrees == angle:
            edges.append(edge)
    if not edges:
        raise Exception('!!!no edge._cluster_centroid[0] = angle')
    return edges



def get_all_turn_right_link_index(roadLinks):
    allow = []
    for index,roadlink in enumerate(roadLinks):
        if roadlink['type'] == 'turn_right':
            allow.append(index)
    return allow


def filter_roadlinks_by_startedge_and_turn_type(roadLinks,edge,turntype):
    result = []
    for index,roadlink in enumerate(roadLinks):
        if roadlink['startRoad'] == edge.getID() and roadlink['type']==turntype:
            result.append((index,roadlink))
    return result

def filter_roadlinks_by_startedge(roadLinks,lane_id):
    result = []
    edge_id,lane_index  = lane_id.split('_')
    for index,roadlink in enumerate(roadLinks):
        lane_index_list = []
        for laneLink in roadlink['laneLinks']:
            lane_index_list.append(laneLink['startLaneIndex'])
        lane_index_list = list(set(lane_index_list))

        if roadlink['startRoad'] == edge_id and int(lane_index) in lane_index_list:
            result.append((index,roadlink))
    return result

def fill_empty_phase(current_phase,count):
    need_fill_count = count - len(current_phase)
    for x in range(need_fill_count):
        empty_phase_dict = {
            'availableRoadLinks': [],
            'time': 0,
        }
        current_phase.append(empty_phase_dict)
    return current_phase

all_phase_dict = {}
node_outgoing_dict = {}

def node_to_intersection(node,tls_dict,edge_dict):
    node_type = node.getType()
    node_coord = node.getCoord()
    intersection = {
        "id": node.getID(),
        "point": {"x": node_coord[0], "y": node_coord[1]},
        "width": 0,  # warning.路口宽度对于任意路口是未定义的.取15
        "roads": [edge.getID() for edge in node.getIncoming() + node.getOutgoing()],

        # "_roads":[{'id':}]
        "roadLinks": [],
        "trafficLight": {
            "roadLinkIndices": [],
            "lightphases": []
        },
        "virtual": _is_node_virtual(node)  # dead_end判断为virtual
    }


    connections_group = group_connections_by_start_end(node.getConnections())
    roadLinks = intersection['roadLinks']
    for k, v in connections_group.items():
        connection_template = v[0]
        start_road = connection_template.getFrom()
        end_road = connection_template.getTo()
        # 加上驶入方向的正北夹角
        raw_roadlink_type = get_direction_fron_connection(connection_template)
        roadLink = {
            "type": raw_roadlink_type,
            "startRoad": start_road.getID(),
            "endRoad": end_road.getID(),
            "direction": 0,  # WARNING: direction is falsely defined but it doesn't affect usage
            "laneLinks": []
        }
        if roadLink["type"] == "turn_u":
            roadLink["type"] = U_TURN_AS


        for start_lane in reversed(start_road._cityflow_lanes):
            if start_lane._direction != raw_roadlink_type:
                continue
            ## TODO lane enumerate
            if TRUE_CORRECTION_lane:
                for end_inx,end_lane in enumerate(reversed(end_road._lanes)):
                    start_point = start_lane.getShape()[-1]
                    start_point = point_tuple_to_dict(start_point)
                    end_point = end_lane.getShape()[0]
                    end_point = point_tuple_to_dict(end_point)
                    path = {
                        "startLaneIndex": _cityflow_get_lane_index_in_edge_cor(start_lane, start_road),
                        "endLaneIndex": end_inx,
                        # ytodo: 或许改为起始lane结束点，路口点，结束lane起始点。
                        "points": [start_point, end_point]  # warning 飞行模式
                    }
                    roadLink["laneLinks"].append(path)
            else:
                for end_lane in end_road._cityflow_lanes:
                    start_point = start_lane.getShape()[-1]
                    start_point = point_tuple_to_dict(start_point)
                    end_point = end_lane.getShape()[0]
                    end_point = point_tuple_to_dict(end_point)
                    path = {
                        "startLaneIndex": _cityflow_get_lane_index_in_edge(start_lane, start_road),
                        "endLaneIndex": _cityflow_get_lane_index_in_edge(end_lane, end_road),
                        "points": [start_point, end_point]  # warning 飞行模式
                    }
                    roadLink["laneLinks"].append(path)
        roadLinks.append(roadLink)


    for i, _ in enumerate(intersection["roadLinks"]):
        intersection["trafficLight"]["roadLinkIndices"].append(i)

    if node_type in ['dead_end']:
        pass
    if node_type in ['priority']:
        pass
    if node_type in ['right_before_left']:
        pass
    if node_type in ['dead_end','priority','right_before_left']:
        intersection = process_intersection_simple_phase(intersection)


    if node_type in ['traffic_light']:
        print(node.getID())
        if SUMO_PROGRAM:
            all_phase = []
            nodeid = node.getID()
            all_phase_dict[nodeid] = []
            G_to_lane_dict = {}
            for connec in tls_dict[nodeid]._connections:
                G_to_lane_dict[connec[-1]] = connec[0].getID()

            for phase,duration in tls_dict[nodeid]._programs['0']._phases:
                lane_list = []
                for i,alpha in enumerate(phase):
                    if (alpha == 'G' or alpha == 'g') and i in G_to_lane_dict.keys():
                        lane_list.append(G_to_lane_dict[i])

                lane_list_ = []
                for lane in lane_list:
                    edge_id,lane_id=lane.split('_')
                    lane_id = int(lane_id)
                    lane_ = edge_id + '_' + str(len(edge_dict[edge_id])-lane_id-1)
                    lane_list_.append(lane_)

                all_phase_dict[nodeid].append(list(set(lane_list_)))
                index_list = []

                for _lane in lane_list_:
                    index_roadlink_list = filter_roadlinks_by_startedge(roadLinks, _lane)
                    index_list += [item[0] for item in index_roadlink_list]
                phase_dict = {
                    'availableRoadLinks': list(set(index_list)),
                    'time': duration
                }
                all_phase.append(phase_dict)
            intersection["trafficLight"]["lightphases"] = all_phase

            outgoing_lane_list = []
            edge_list_ = [edge_.getID() for edge_ in node.getOutgoing()]
            for edge in edge_list_:
                for i in range(len(edge_dict[edge])):
                    outgoing_lane_list.append(edge+'_'+str(i))
            node_outgoing_dict[nodeid] = outgoing_lane_list

        exiting_lane_list = []
        for edge in node.getOutgoing():
            exiting_lane_list.extend([lane.getID() for lane in edge.getLanes()])

    return intersection

def get_final_intersections(net,tls_dict,edge_dict):

    final_intersections = []
    net_nodes = net.getNodes()
    net_nodes_sorted = sorted(net_nodes,key=lambda n:n.getID())
    nodes = [(index,node) for index,node in enumerate(net_nodes_sorted)]
    nodes = nodes[:]
    for obj in nodes:

        index = obj[0]
        node = obj[1]

        intersection = node_to_intersection(node,tls_dict,edge_dict)
        if intersection["roads"] != []:
            final_intersections.append(intersection)

    return final_intersections

def get_final_roads(net):
    edges = net.getEdges()
    final_roads = []
    for edge in edges:
        start_intersection = edge.getFromNode()
        start_coord = start_intersection.getCoord()
        end_intersection = edge.getToNode()
        end_coord = end_intersection.getCoord()
        road = {
            "id": edge.getID(),
            "points": [
                {
                    "x": start_coord[0],
                    "y": start_coord[1],
                },
                {
                    "x": end_coord[0],
                    "y": end_coord[1],
                }
            ],
            "lanes": [
            ],
            "startIntersection": start_intersection.getID(),
            "endIntersection": end_intersection.getID(),
        }
        if DEBUG:
            road['_compass_angle'] = calc_edge_compass_angle_no_modify(edge)
        lane_template = {
            "width": 4,
            "maxSpeed": 11.111  # warning 放弃速度
        }
        if TRUE_CORRECTION_lane:
            for _v in edge._lanes:
                road["lanes"].append(lane_template)
        else:
            for _v in edge._cityflow_lanes:
                road["lanes"].append(lane_template)
        final_roads.append(road)
    return final_roads



def main(args):
    print("Converting sumo net file",args.sumonet)
    net = sumolib.net.readNet(os.path.join(os.getcwd(),args.sumonet), withPrograms=True)

    for edge in net.getEdges():
        process_edge(edge)

    tls_dict = {}
    for tls in net.getTrafficLights():
        tls_dict[tls.getID()] = tls

    print('Start processing '+str(len(tls_dict))+" traffic lights")
    edge_dict = {}
    for edge_ in net.getEdges():
        edge_dict[edge_.getID()] = edge_._lanes

    final_intersections = get_final_intersections(net,tls_dict,edge_dict)

    for intersection in final_intersections:
        if intersection['virtual']:
            intersection['roadLinks'] = []

    final_roads = get_final_roads(net)

    result = {
        "intersections": final_intersections,
        "roads": final_roads
    }

    f = open(args.cityflownet, 'w')
    json.dump(result, f, indent=2)
    f.close()


if __name__ == '__main__':
    args = parse_args()

    main(args)
    print("Cityflow net file generated successfully!")


'''

Direction is meaningless
u turn type exists
'''