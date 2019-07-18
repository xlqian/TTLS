import osmnx
import time

from algorithms.multimodal_astar import MultiModalAStart
from algorithms.utils import project_nodes
from bss_locations_example import bss_locations

G = osmnx.load_graphml(filename='network.graphml')


orig_node = osmnx.get_nearest_node(G, (48.84571, 2.37261))
dest_node = osmnx.get_nearest_node(G, (48.867006, 2.335881))

# Example of multimodal_astar

bss_nodes = project_nodes(G, bss_locations)

print('start double A*!')
start = time.time()
multimodal_star = MultiModalAStart()
(f, bss, b), (f_secs, bss_secs, b_secs) = multimodal_star.get_best_path(G, orig_node, dest_node, bss_nodes)
end = time.time()
print(end - start)
print(f_secs, bss_secs, b_secs)

osmnx.plot_graph_routes(G, [f, bss, b], node_size=0)
