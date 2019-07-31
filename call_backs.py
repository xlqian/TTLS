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
            nc.append('#a2fe91')
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


def double_astar_call_back(g, origin, dest, edge_status_f, edge_labels_f, edge_status_b, edge_labels_b, i, prefix='double_astar'):

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

    # bbpx = [north, south, east, west]
    osmnx.plot_graph(g, margin=0, bbox=(48.87, 48.838963, 2.382, 2.32), dpi=100, node_size=ns, node_color=nc, node_zorder=2, show=False, save=True,
                     filename='{}_{}'.format(prefix, i))

def double_expansion_call_back(g, origin, dest,bss,
                               edge_status_w_f,
                               edge_labels_w_f,
                               edge_status_w_b,
                               edge_labels_w_b,
                               edge_status_b_f,
                               edge_labels_b_f,
                               edge_status_b_b,
                               edge_labels_b_b,
                               i,
                               prefix='double_expansion'):
    # bbpx = [north, south, east, west]
    bbox = (48.88, 48.838963, 2.382, 2.32)
    dpi = 200
    fig, ax = osmnx. plot_graph(g, bbox=bbox, fig_height=6,
                     margin=0.02, bgcolor='w', axis_off=True, show=False,
                                save=False, close=False, file_format='png', filename='temp',
                     dpi=dpi, annotate=False, node_color='#999999',
                     node_size=0, node_alpha=0, node_edgecolor='none',
                     node_zorder=1, edge_color='#999999', edge_linewidth=1,
                     edge_alpha=1, use_geom=True)

    origin_destination_lats = (g.nodes[origin]['y'], g.nodes[dest]['y'])
    origin_destination_lons = (g.nodes[origin]['x'], g.nodes[dest]['x'])

    ax.scatter(origin_destination_lons, origin_destination_lats, s=50,
               c='#c942ff', alpha=1, edgecolor='none', zorder=6)

    w_lats = []
    w_lons = []

    for _, status in edge_status_w_f.items():
        node = edge_labels_w_f[status.edge_label_index].end_node
        w_lats.append(g.nodes[node]['y'])
        w_lons.append(g.nodes[node]['x'])

    for _, status in edge_status_w_b.items():
        node = edge_labels_w_b[status.edge_label_index].end_node
        w_lats.append(g.nodes[node]['y'])
        w_lons.append(g.nodes[node]['x'])

    ax.scatter(w_lons, w_lats, s=5, c='#42ff00', alpha=0.7, edgecolor='none', zorder=3)

    b_lats = []
    b_lons = []

    for _, status in edge_status_b_f.items():
        node = edge_labels_b_f[status.edge_label_index].end_node
        b_lats.append(g.nodes[node]['y'])
        b_lons.append(g.nodes[node]['x'])

    for _, status in edge_status_b_b.items():
        node = edge_labels_b_b[status.edge_label_index].end_node
        b_lats.append(g.nodes[node]['y'])
        b_lons.append(g.nodes[node]['x'])

    ax.scatter(b_lons, b_lats, s=5, c='#ff1b00', alpha=0.5, edgecolor='none', zorder=4)

    bss_lats = []
    bss_lons = []

    for node in bss:
        bss_lats.append(g.nodes[node]['y'])
        bss_lons.append(g.nodes[node]['x'])

    ax.scatter(bss_lons, bss_lats, s=10, c='#0046ff', alpha=1, edgecolor='none', zorder=5)

    filename = '{}_{}'.format(prefix, i)
    show = False
    save = True
    axis_off = True
    file_format = 'png'
    close = True

    osmnx.save_and_show(fig, ax, save, show, close, filename, file_format, dpi, axis_off)

