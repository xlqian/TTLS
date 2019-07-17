import osmnx


def astar_call_back(g, origin, dest, edge_status, edge_labels, i):
    perm_nodes = set()
    temp_nodes = set()
    for _, status in edge_status.items():
        if status.is_permanent():
            perm_nodes.add(edge_labels[status.edge_label_index].end_node)
        elif status.is_temporary():
            temp_nodes.add(edge_labels[status.edge_label_index].end_node)

    nc = []
    for node in g.nodes():
        if node in (origin, dest):
            nc.append('#c942ff')
        elif node in perm_nodes:
            nc.append('#5bff33')
        elif node in temp_nodes:
            nc.append('r')
        else:
            nc.append('#f4fbff')

    ns = []
    for node in g.nodes():
        if node in (origin, dest):
            ns.append(20)
        elif node in perm_nodes or node in temp_nodes:
            ns.append(5)
        else:
            ns.append(0)

    osmnx.plot_graph(g, dpi=100, node_size=ns, node_color=nc, node_zorder=2, show=False, save=True,
                     filename='a_astar_{}'.format(i))


def double_astar_call_back(g, origin, dest, edge_status_f, edge_labels_f, edge_status_b, edge_labels_b, i):

    perm_nodes = set()
    temp_nodes = set()

    for _, status in edge_status_f.items():
        if status.is_permanent():
            perm_nodes.add(edge_labels_f[status.edge_label_index].end_node)
        elif status.is_temporary():
            temp_nodes.add(edge_labels_f[status.edge_label_index].end_node)

    for _, status in edge_status_b.items():
        if status.is_permanent():
            perm_nodes.add(edge_labels_b[status.edge_label_index].end_node)
        elif status.is_temporary():
            temp_nodes.add(edge_labels_b[status.edge_label_index].end_node)

    nc = []
    for node in g.nodes():
        if node in (origin, dest):
            nc.append('#c942ff')
        elif node in perm_nodes:
            nc.append('#5bff33')
        elif node in temp_nodes:
            nc.append('r')
        else:
            nc.append('#f4fbff')

    ns = []
    for node in g.nodes():
        if node in (origin, dest):
            ns.append(20)
        elif node in perm_nodes or node in temp_nodes:
            ns.append(5)
        else:
            ns.append(0)

    osmnx.plot_graph(g, dpi=100, node_size=ns, node_color=nc, node_zorder=2, show=False, save=True,
                     filename='double_a_astar_{}'.format(i))
