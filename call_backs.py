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

def double_expansion_call_back(g, origin, dest, bss,
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
    # bbox = [north, south, east, west]
    o_lat, o_lon = g.nodes[origin]['y'], g.nodes[origin]['x']
    d_lat, d_lon = g.nodes[dest]['y'], g.nodes[dest]['x']
    lat_diff = abs(o_lat - d_lat)
    lon_diff = abs(o_lon - d_lon)

    coeff = lat_diff / lon_diff

    # bbox = [north, south, east, west]
    bbox = (max(o_lat, d_lat) + 0.2 * lat_diff / coeff,
            min(o_lat, d_lat) - 0.2 * lat_diff / coeff,
            max(o_lon, d_lon) + 0.2 * lon_diff,
            min(o_lon, d_lon) - 0.2 * lon_diff)

    dpi = 200
    fig, ax = osmnx.plot_graph(g, bbox=bbox, fig_height=6,
                               margin=0.02, bgcolor='w', axis_off=True, show=False,
                               save=False, close=False, file_format='png', filename='temp',
                               dpi=dpi, annotate=False, node_color='#999999',
                               node_size=0, node_alpha=0, node_edgecolor='none',
                               node_zorder=1, edge_color='#999999', edge_linewidth=1,
                               edge_alpha=1, use_geom=True)

    origin_destination_lats = (o_lat, d_lat)
    origin_destination_lons = (o_lon, d_lon)

    ax.scatter(origin_destination_lons, origin_destination_lats, s=50,
               c='#c942ff', alpha=1, edgecolor='none', zorder=6)

    w_nodes = set()
    for _, status in edge_status_w_f.items():
        node = edge_labels_w_f[status.edge_label_index].end_node
        w_nodes.add(node)
    for _, status in edge_status_w_b.items():
        node = edge_labels_w_b[status.edge_label_index].end_node
        w_nodes.add(node)

    w_lats = []
    w_lons = []

    for node in w_nodes:
        w_lats.append(g.nodes[node]['y'])
        w_lons.append(g.nodes[node]['x'])

    ax.scatter(w_lons, w_lats, s=7, c='#ff1b00', alpha=0.5, edgecolor='none', zorder=3)

    b_nodes = set()
    for _, status in edge_status_b_f.items():
        node = edge_labels_b_f[status.edge_label_index].end_node
        b_nodes.add(node)
    for _, status in edge_status_b_b.items():
        node = edge_labels_b_b[status.edge_label_index].end_node
        b_nodes.add(node)

    b_lats = []
    b_lons = []

    for node in b_nodes:
        b_lats.append(g.nodes[node]['y'])
        b_lons.append(g.nodes[node]['x'])

    ax.scatter(b_lons, b_lats, s=5, c='#42ff00', alpha=0.5, edgecolor='none', zorder=4)

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


def double_expansion_isochrone_callback(g, origin, dest_nodes, bss,
                                        edges_status_walking, edge_labels_walking,
                                        edges_status_bike, edge_labels_bike,
                                        i, prefix="double_expansion_isochrone"):

    o_lat, o_lon = g.nodes[origin]['y'], g.nodes[origin]['x']

    # bbox = [north, south, east, west]
    bbox = (o_lat + 0.042, o_lat - 0.042, o_lon + 0.06, o_lon - 0.06)

    dpi = 200
    fig, ax = osmnx.plot_graph(g, bbox=bbox, fig_height=6,
                               margin=0.02, bgcolor='w', axis_off=True, show=False,
                               save=False, close=False, file_format='png', filename='temp',
                               dpi=dpi, annotate=False, node_color='#999999',
                               node_size=0, node_alpha=0, node_edgecolor='none',
                               node_zorder=1, edge_color='#999999', edge_linewidth=1,
                               edge_alpha=1, use_geom=True)

    ax.scatter((o_lon), (o_lat), s=50, c='#c942ff', alpha=1, edgecolor='none', zorder=6)

    walking_first_nodes = set([edge_labels_walking[status.edge_label_index].end_node
                               for _, status in edges_status_walking.items()
                               if edge_labels_walking[status.edge_label_index].can_change_mode])

    walking_first_lats = []
    walking_first_lons = []

    for node in walking_first_nodes:
        walking_first_lats.append(g.nodes[node]['y'])
        walking_first_lons.append(g.nodes[node]['x'])

    ax.scatter(walking_first_lons, walking_first_lats, s=7, c='#ff1b00', alpha=0.3, edgecolor='none', zorder=3)

    walking_second_nodes = set([edge_labels_walking[status.edge_label_index].end_node
                               for _, status in edges_status_walking.items()
                               if not edge_labels_walking[status.edge_label_index].can_change_mode])

    walking_second_lats = []
    walking_second_lons = []

    for node in walking_second_nodes:
        walking_second_lats.append(g.nodes[node]['y'])
        walking_second_lons.append(g.nodes[node]['x'])

    ax.scatter(walking_second_lons, walking_second_lats, s=5, c='#22ff36', alpha=0.7, edgecolor='none', zorder=5)

    bike_nodes = set([edge_labels_bike[status.edge_label_index].end_node
                      for _, status in edges_status_bike.items()])

    bike_lats = []
    bike_lons = []

    for node in bike_nodes:
        bike_lats.append(g.nodes[node]['y'])
        bike_lons.append(g.nodes[node]['x'])

    ax.scatter(bike_lons, bike_lats, s=6, c='#cb6aff', alpha=0.2, edgecolor='none', zorder=4)

    bss_lats = []
    bss_lons = []

    for node in bss:
        bss_lats.append(g.nodes[node]['y'])
        bss_lons.append(g.nodes[node]['x'])

    ax.scatter(bss_lons, bss_lats, s=8, c='#0046ff', alpha=1, edgecolor='none', zorder=6)

    filename = '{}_{}'.format(prefix, i)
    show = False
    save = True
    axis_off = True
    file_format = 'png'
    close = True
    osmnx.save_and_show(fig, ax, save, show, close, filename, file_format, dpi, axis_off)


def double_expansion_one_queue_callback(g,
                                        origin, dest, bss,
                                        edge_status, edge_labels,
                                        i, prefix="double_expansion_one_queue"):
    # bbox = [north, south, east, west]
    o_lat, o_lon = g.nodes[origin]['y'], g.nodes[origin]['x']
    d_lat, d_lon = g.nodes[dest]['y'], g.nodes[dest]['x']
    lat_diff = abs(o_lat - d_lat)
    lon_diff = abs(o_lon - d_lon)

    coeff = lat_diff / lon_diff

    # bbox = [north, south, east, west]
    bbox = (max(o_lat, d_lat) + 0.8 * lat_diff / coeff,
            min(o_lat, d_lat) - 0.8 * lat_diff / coeff,
            max(o_lon, d_lon) + 0.8 * lon_diff,
            min(o_lon, d_lon) - 0.8 * lon_diff)

    dpi = 200
    fig, ax = osmnx.plot_graph(g, bbox=bbox, fig_height=6,
                               margin=0.02, bgcolor='w', axis_off=True, show=False,
                               save=False, close=False, file_format='png', filename='temp',
                               dpi=dpi, annotate=False, node_color='#999999',
                               node_size=0, node_alpha=0, node_edgecolor='none',
                               node_zorder=1, edge_color='#999999', edge_linewidth=1,
                               edge_alpha=1, use_geom=True)

    origin_destination_lats = (o_lat, d_lat)
    origin_destination_lons = (o_lon, d_lon)

    ax.scatter(origin_destination_lons, origin_destination_lats, s=50, c='#c942ff', alpha=1, edgecolor='none', zorder=6)

    bss_lats = []
    bss_lons = []

    for node in bss:
        bss_lats.append(g.nodes[node]['y'])
        bss_lons.append(g.nodes[node]['x'])

    ax.scatter(bss_lons, bss_lats, s=8, c='#0046ff', alpha=1, edgecolor='none', zorder=6)

    walking_nodes = set([edge_labels[status.edge_label_index].end_node
                         for _, status in edge_status.items()
                         if edge_labels[status.edge_label_index].edge_id.mode.value == 0])

    walking_lats = []
    walking_lons = []

    for node in walking_nodes:
        walking_lats.append(g.nodes[node]['y'])
        walking_lons.append(g.nodes[node]['x'])

    ax.scatter(walking_lons, walking_lats, s=7, c='#ff1b00', alpha=0.4, edgecolor='none', zorder=3)

    bike_nodes = set([edge_labels[status.edge_label_index].end_node
                      for _, status in edge_status.items()
                      if edge_labels[status.edge_label_index].edge_id.mode.value == 1])

    bike_lats = []
    bike_lons = []

    for node in bike_nodes:
        bike_lats.append(g.nodes[node]['y'])
        bike_lons.append(g.nodes[node]['x'])

    ax.scatter(bike_lons, bike_lats, s=6, c='#22ff36', alpha=0.4, edgecolor='none', zorder=4)

    bss_lats = []
    bss_lons = []

    for node in bss:
        bss_lats.append(g.nodes[node]['y'])
        bss_lons.append(g.nodes[node]['x'])

    ax.scatter(bss_lons, bss_lats, s=8, c='#0046ff', alpha=1, edgecolor='none', zorder=6)

    filename = '{}_{}'.format(prefix, i)
    show = False
    save = True
    axis_off = True
    file_format = 'png'
    close = True
    osmnx.save_and_show(fig, ax, save, show, close, filename, file_format, dpi, axis_off)
