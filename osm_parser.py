
import networkx as nx
import xml.etree.ElementTree as xml
import requests
import json


class UndirectedGraph:
    def __init__(self, path):
        self.graph = osm_xml_parser(path)


REQUESTS_PER_CALL = 1500


def osm_xml_parser(file, from_file=False):
    """Function to parse an osm file and create a network out of it.

    Parameters:
        filename - The filename of the file to import.
    Returns:
        graph - The created graph.
    """

    # Parse the xml structure and initialize variables.
    if from_file:
        e = xml.parse(file).getroot()
    else:
        e = xml.fromstring(file)
    node_dict_tmp = {}
    graph = nx.Graph()

    # Allow these types of streets to be represented in the network by an edge.
    way_types = ["footway", "residential", "living_street", "track", "road", "bridleway", "steps", "path", "cycleway"]

    # Create nodes and edges.
    for i in e:
        # Nodes.
        if i.tag == "node":
            node_dict_tmp[i.attrib["id"]] = [i.attrib["lat"], i.attrib["lon"]]

        # Edges.
        if i.tag == "way":
            insert = False
            directed = False
            max_speed_v = None
            way_tmp = []
            for j in i:
                if j.tag == "nd":
                    way_tmp.append(j.attrib["ref"])
                if j.tag == "tag":
                    if j.attrib["k"] == "highway":# and j.attrib["v"] in way_types:
                        insert = True
            if insert:
                if max_speed_v is None:
                    graph.add_path(way_tmp)
                    # if not directed:
                    #     graph.add_path(list(reversed(way_tmp)))
                else:
                    graph.add_path(way_tmp, max_speed=max_speed_v)
                    if not directed:
                        graph.add_path(list(reversed(way_tmp)), max_speed=max_speed_v)

    # Extend the nodes by their geographical coordinates.
    headers = {'Accept': 'application/json', 'Content-Type': 'application/json'}
    elevations = {}
    prev_limit = 0
    print("Total nodes {}".format(len(node_dict_tmp.keys())))
    for index in range(1, int(len(node_dict_tmp.keys()) / REQUESTS_PER_CALL) + 2):
        limit = index * REQUESTS_PER_CALL
        if limit > len(node_dict_tmp.keys()):
            limit = prev_limit + len(node_dict_tmp.keys())
        keys = list(node_dict_tmp.keys())
        partial_keys = keys[prev_limit: limit]
        prev_limit = limit
        data = json.dumps({"locations": [{"longitude": float(node_dict_tmp[i][1]), "latitude": float(node_dict_tmp[i][0])} for i in partial_keys]})
        res = requests.post("https://api.open-elevation.com/api/v1/lookup", data=data, headers=headers)
        if not res.ok:
            print("Error getting elevations: {}".format(res))
            exit()
        else:
            print("Successfully obtained elevations")
        elevations_json = json.loads(res.content.decode("utf-8"))["results"]
        for i, key in enumerate(partial_keys):
            elevations[key] = elevations_json[i]
    network_nodes = graph.nodes()
    for i in network_nodes:
        graph.node[i]["coords"] = [elevations[i]["latitude"], elevations[i]["longitude"]]
        graph.node[i]["elevation"] = elevations[i]["elevation"]
        graph.node[i]["key"] = i

    # Return the generated graph.
    return graph


if __name__ == "__main__":
    # Specify the path to the OSM-XML file and call the parser.
    path_to_file = "nufar.osm"
    graph = osm_xml_parser(path_to_file, from_file=True)
    nx.write_gpickle(graph, 'graph-nofar.txt')
