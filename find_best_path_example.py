from algorithms.astar import AStar
from algorithms.double_astar import DoubleAstar
import osmnx
import time

G = osmnx.load_graphml(filename='network.graphml')


orig_node = osmnx.get_nearest_node(G, (48.84561, 2.37207))
dest_node = osmnx.get_nearest_node(G, (48.867006, 2.335881))

print('start A*!')
start = time.time()
a_star = AStar()
route, secs = a_star.get_best_path(G, orig_node, dest_node)
end = time.time()
print(end - start)
print(route)
print(secs)

osmnx.plot_graph_route(G, route, node_size=0)

# Example of using call_back
from call_backs import double_astar_call_back
print('start double A*!')
start = time.time()
double_star = DoubleAstar()
route, secs = double_star.get_best_path(G, orig_node, dest_node)
end = time.time()
print(end - start)
print(route)
print(secs)

osmnx.plot_graph_route(G, route, node_size=0)
