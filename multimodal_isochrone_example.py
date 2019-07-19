import osmnx
import time

from algorithms.multimodal_isochrone import MultiModalIsochrone
from algorithms.utils import project_nodes
from bss_locations_example import bss_locations

G = osmnx.load_graphml(filename='network.graphml')

bss_nodes = project_nodes(G, bss_locations)
orig_node = osmnx.get_nearest_node(G, (48.867006, 2.335881))

print('start multimodal isochrone!')

start = time.time()

multimodal_isochrone = MultiModalIsochrone(bss_nodes)

dest_nodes = {306092060, 368142}

res = multimodal_isochrone.get_isochrone(G, orig_node, dest_nodes)

print(res)
