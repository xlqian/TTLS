import networkx as nx
import osmnx

# Download the OSM in which you're interesting

# location_point = (48.866667, 2.333333)
# G = osmnx.core.graph_from_point(location_point, network_type='walk', distance=3500, simplify=True)
# osmnx.save_graphml(G, filename='network.graphml')

G = osmnx.load_graphml(filename='network.graphml')
orig_node = osmnx.get_nearest_node(G, (48.84561, 2.37207))
dest_node = osmnx.get_nearest_node(G, (48.867006, 2.335881))
route = nx.shortest_path(G, orig_node, dest_node, weight='length')
fig, ax = osmnx.plot_graph_route(G, route, node_size=0)
