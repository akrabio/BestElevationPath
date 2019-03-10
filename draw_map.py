import matplotlib.pyplot as plt
import mplleaflet
from get_path import PathFinder


def draw_map(longitude, latitude):
    # plt.hold(True)
    plt.plot(longitude, latitude, 'b')
    mapfile = 'test_plot.html'
    mplleaflet.show(path=mapfile)


if __name__ == "__main__":
    import networkx as nx
    import json
    import requests
    import osm_parser
    search_string = 'אינטרנציונל 52'
    start = json.loads(requests.get('https://nominatim.openstreetmap.org/?format=json&addressdetails=1&q={}&format=json&limit=1'.format(search_string)).text)[0]
    # address = 'https://overpass-api.de/api/map?bbox={},{},{},{}'.format(str(float(start['boundingbox'][2])-0.002),str(float(start['boundingbox'][0])-0.002),str(float(start['boundingbox'][3])+0.002),str(float(start['boundingbox'][1])+0.002))
    # address = 'https://overpass-api.de/api/map?bbox=34.9857,32.7638,35.0324,32.7877'
    # map = requests.get(address).text
    # start = '1173992753'#'1144258541'
    # finish = '1148726786'#'1672043507'
    graph = nx.read_gpickle('graph2.txt')
    # graph = osm_parser.osm_xml_parser(map)
    desired_distance = 4
    path_finder = PathFinder(graph)
    lat, long, distance, elevation = path_finder.generate_path(start, desired_distance)
    print('total elevation change: {}\ntotal distance: {}'.format(elevation, distance))
    draw_map(long, lat)
