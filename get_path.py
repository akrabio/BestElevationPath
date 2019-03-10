from geopy.distance import vincenty
from path_algorithm import AStar
from colorama import Fore


Infinite = float('inf')
range_width = 1000
thresholds = [0]#[0, 10, 20, 30, 40]


class PathFinder:
    def __init__(self, graph):
        self.graph = graph
        self.sorted_nodes = sorted(graph.nodes(), key=lambda node: graph.node[node]["elevation"])

    def generate_path(self, start, distance):
        closest = find_closest_node(self.graph, start)
        return get_path(self.graph, self.sorted_nodes, closest, distance)


def get_path(graph, sorted_nodes, start, desired_distance):
    best_lat = []
    best_long = []
    best_lat_back = []
    best_long_back = []
    best_elevation = Infinite
    best_distance = Infinite
    i = 1
    found_nodes = 1
    closest_node = find_closest(graph, sorted_nodes, start, 'elevation')
    range_start = 0 if closest_node - range_width < 0 else closest_node - range_width
    range_end = len(sorted_nodes) - 1 if closest_node + range_width >= len(sorted_nodes) else closest_node + range_width
    print(Fore.GREEN + 'closest node {}, start range {}, end range {}'
          '\nstart node elevation {}, found node elevation {}'
          '\nstarting search at elevation {}, ending search at elevation {}'.format(closest_node, range_start, range_end,
                                                                                    graph.node[start]['elevation'],
                                                                                    graph.node[sorted_nodes[closest_node]]['elevation'],
                                                                                    graph.node[sorted_nodes[range_start]]['elevation'],
                                                                                    graph.node[sorted_nodes[range_end]]['elevation']))
    for node in sorted_nodes[range_start:range_end]:
        if i % 100 == 0:
            print(Fore.WHITE + '{} out of {}'.format(i, range_end - range_start))
        i += 1
        if desired_distance / 3.5 < vincenty(graph.node[start]["coords"], graph.node[node]["coords"]).km < desired_distance / 2.5:
            for threshold in thresholds:
                found_path = AStar(graph, desired_distance, threshold).astar(graph.node[start], graph.node[node])
                lat, long, tot_elevation, tot_distance, edges = get_path_information(graph, found_path, start)
                if tot_elevation < best_elevation / 2 and edges is not None and len(edges) > 0:
                    tmp_graph = graph.copy()
                    # tmp_graph.remove_edges_from(edges)
                    # tmp_graph.remove_edge(edges[-1][0], edges[-1][1])
                    # tmp_graph.remove_edge(edges[0][0], edges[0][1])
                    way_back = AStar(tmp_graph, desired_distance - tot_distance, threshold).astar(tmp_graph.node[node], tmp_graph.node[start])
                    if way_back is not None:
                        lat_back, long_back, tot_elevation_way_back, tot_distance_way_back, _ = get_path_information(tmp_graph, way_back, node)
                        if tot_elevation + tot_elevation_way_back < best_elevation and abs((tot_distance + tot_distance_way_back) - desired_distance) < 0.5:
                            print(Fore.BLUE + 'found {} routes so far - best elevation {}'.format(found_nodes, tot_elevation + tot_elevation_way_back))
                            found_nodes += 1
                            best_lat = lat
                            best_lat_back = lat_back
                            best_long_back = long_back
                            best_long = long
                            best_elevation = tot_elevation + tot_elevation_way_back
                            best_distance = tot_distance + tot_distance_way_back

    # import matplotlib.pyplot as plt
    # import mplleaflet
    #
    # plt.plot(best_long, best_lat, 'b')
    # plt.plot(best_long_back, best_lat_back, 'r')
    # mapfile = 'test_plot.html'
    # mplleaflet.show(path=mapfile)
    return best_lat + best_lat_back, best_long + best_long_back, best_distance, best_elevation


def get_path_information(graph, found_path, start):
    if not found_path:
        return None, None, 0, 0, None
    lat = []
    long = []
    edges = []
    last_node = None
    last_elevation = graph.node[start]['elevation']
    last_distance = 0
    tot_distance = 0
    tot_elevation = 0
    for node in found_path:
        if last_node is not None:
            edges.append((node["key"], last_node))
            # edges.append((last_node, node["key"]))
        last_node = node["key"]
        tot_elevation += abs(last_elevation - node['elevation'])
        last_elevation = node['elevation']
        tot_distance = node['distance']
        lat.append(node["coords"][0])
        long.append(node["coords"][1])
    return lat, long, tot_elevation, tot_distance, edges


def find_closest(graph, sorted_nodes, target, key):
    target_value = target if target not in graph.node else graph.node[target][key]
    # target_elevation = graph.node[target][key]
    if target_value <= graph.node[sorted_nodes[0]][key]:
        return 0
    if target_value >= graph.node[sorted_nodes[-1]][key]:
        return len(sorted_nodes) - 1

    mid = 0
    left = 0
    right = len(sorted_nodes)

    while left < right:
        mid = int((left + right) / 2)
        current = graph.node[sorted_nodes[mid]][key]
        if current == target_value:
            return mid
        if target_value < current:
            if mid > 0 and target_value > graph.node[sorted_nodes[mid -1]][key]:
                return get_closest(graph, sorted_nodes, mid, mid-1, target)
            right = mid
        else:
            if mid < len(sorted_nodes) - 1 and target_value < graph.node[sorted_nodes[mid+1]][key]:
                return get_closest(graph, sorted_nodes, mid, mid+1, target)
            left = mid + 1

    return mid


def get_closest(graph, sorted_nodes, node1, node2, target):
    node1_elevation = graph.node[sorted_nodes[sorted_nodes[node1]]]['elevation']
    node2_elevation = graph.node[sorted_nodes[sorted_nodes[node2]]]['elevation']
    target_elevation = graph.node[target]['elevation']
    if abs(target_elevation - node1_elevation) < abs(target_elevation - node2_elevation):
        return node1
    else:
        return node2


def find_closest_node(graph, start):
    if start['osm_id'] in graph.nodes():
        return start['osm_id']
    closest_node = start['osm_id']
    closest_distance = Infinite
    for node in graph.nodes():
        distance = vincenty(graph.node[node]["coords"], (start['lat'], start['lon'])).km
        if distance < closest_distance:
            closest_distance = distance
            closest_node = node
    return closest_node
