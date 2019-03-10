from heapq import heappush, heappop
from geopy.distance import vincenty

Infinite = float('inf')


class AStar:
    def __init__(self, graph, target_distance, elevation_threshold):
        self.graph = graph
        self.elevation_threshold = elevation_threshold
        self.target_distance = target_distance

    class SearchNode:
        def __init__(self, data, gscore=Infinite, fscore=Infinite):
            self.data = data
            self.gscore = gscore
            self.fscore = fscore
            self.closed = False
            self.out_openset = True
            self.came_from = None

        def __lt__(self, b):
            return self.fscore < b.fscore

    class SearchNodeDict(dict):
        def __init__(self, g):
            self.g = g

        def __missing__(self, k):
            v = AStar.SearchNode(self.g.node[k])
            self.__setitem__(k, v)
            return v

    def heuristic_cost_estimate(self, current, goal):
        if 'distance' not in current:
            current['distance'] = 0
        elevation_dif = abs(goal["elevation"] - current["elevation"])
        difference_from_target = self.target_distance - current['distance']
        return difference_from_target if elevation_dif <= self.elevation_threshold else elevation_dif

    def distance_between(self, n1, n2):
        elevation_dif = abs(n1["elevation"] - n2["elevation"])
        distance = vincenty((n1["coords"][0], n1["coords"][1]), (n2["coords"][0], n2["coords"][1])).km

        return distance / 10 if elevation_dif <= self.elevation_threshold else elevation_dif

    def neighbors(self, node):
        return list(self.graph.neighbors(node))

    def is_goal_reached(self, current, goal):
        return current == goal

    def reconstruct_path(self, last, reverse_path=False):
        def _gen():
            current = last
            while current:
                yield current.data
                current = current.came_from
        if reverse_path:
            return _gen()
        else:
            return reversed(list(_gen()))

    def astar(self, start, goal, reverse_path=False):
        start["distance"] = 0
        if self.is_goal_reached(start, goal):
            return [start]
        search_nodes = AStar.SearchNodeDict(self.graph)
        start_node = search_nodes[start['key']] = AStar.SearchNode(
            start, gscore=.0, fscore=self.heuristic_cost_estimate(start, goal))
        open_set = []
        heappush(open_set, start_node)
        while open_set:
            current = heappop(open_set)
            if self.is_goal_reached(current.data, goal):
                return self.reconstruct_path(current, reverse_path)
            current.out_openset = True
            current.closed = True
            for neighbor in [search_nodes[n] for n in self.neighbors(current.data["key"])]:
                if neighbor.closed:
                    continue
                tentative_gscore = current.gscore + \
                    self.distance_between(current.data, neighbor.data)
                if tentative_gscore >= neighbor.gscore:
                    continue
                neighbor.came_from = current
                neighbor.gscore = tentative_gscore
                neighbor.fscore = tentative_gscore + \
                    self.heuristic_cost_estimate(neighbor.data, goal)
                if 'distance' not in neighbor.data:
                    neighbor.data['distance'] = 0
                neighbor.data['distance'] = current.data["distance"] + vincenty((current.data["coords"][0], current.data["coords"][1]),
                                              (neighbor.data["coords"][0], neighbor.data["coords"][1])).kilometers
                if neighbor.out_openset:
                    neighbor.out_openset = False
                    heappush(open_set, neighbor)
        return None


if __name__ == '__main__':
    import networkx as nx
    start = '3725304404'#'1136268079'
    finish = '1173992753'#'1137184296'
    graph = nx.read_gpickle('graph.txt')
    found_path = AStar(graph, 3, 0).astar(graph.node[start], graph.node[finish])
    lat = []
    long = []
    last_elevation = graph.node[start]['elevation']
    tot_elevation = 0
    tot_distance = 0
    for node in found_path:
        tot_elevation += abs(last_elevation - node['elevation'])
        last_elevation = node['elevation']
        tot_distance = node['distance']
        lat.append(node["coords"][0])
        long.append(node["coords"][1])

    print(lat)
    print(long)
    print('elevation start: {}\nelevation end: {}\ntotal elevation change: {}\ntotal distance: {}'.format(graph.node[start]['elevation'], graph.node[finish]['elevation'],tot_elevation, tot_distance))
    import matplotlib.pyplot as plt
    import mplleaflet
    plt.plot(long, lat, 'b')
    mapfile = 'test_plot.html'
    mplleaflet.show(path=mapfile)


