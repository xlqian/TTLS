from algorithms.isochrone import Isocrhone
from bss_locations_example import bss_locations
from algorithms.utils import project_nodes
import osmnx
import time

G = osmnx.load_graphml(filename='network.graphml')
orig_node = osmnx.get_nearest_node(G, (48.867006, 2.335881))

bss_nodes = project_nodes(G, bss_locations)

isocrhone = Isocrhone()

start = time.time()
res = isocrhone.get_isochrone(G, orig_node, bss_nodes)
end = time.time()
print(end - start)

nc = []
for node in G.nodes():
    if node in res:
        nc.append('r')
    else:
        nc.append('#f4fbff')

ns = []
for node in G.nodes():
    if node in res:
        ns.append(20)
    else:
        ns.append(0)

osmnx.plot_graph(G, dpi=300, node_size=ns, node_color=nc, node_zorder=2)
