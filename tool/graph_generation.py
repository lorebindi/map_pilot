import os
import networkx as nx
import osmnx as ox
import pandas as pd

data_path = os.path.join("..", "data")

def check_reachability(G, node_id_to_index, ordered_nodes, src, dest):
    if len(ordered_nodes) >= src and len(ordered_nodes) >= dest:
        node_A = ordered_nodes[1]
        node_B = ordered_nodes[3]
        print(f"Nodo A: {node_id_to_index[node_A]}, Nodo B: {node_id_to_index[node_B]}")

        reachable = nx.has_path(G, node_A, node_B)
        print(f"Raggiungibilità da A a B: {reachable}")
        if reachable:
            path = nx.shortest_path(G, node_A, node_B, weight='length')
            path_indices = [node_id_to_index[n] for n in path]
            print(f"Percorso da A a B: {path_indices}")

        reachable_inv = nx.has_path(G, node_B, node_A)
        print(f"Raggiungibilità da B a A: {reachable_inv}")
        if reachable_inv:
            path = nx.shortest_path(G, node_B, node_A, weight='length')
            path_indices = [node_id_to_index[n] for n in path]
            print(f"Percorso da B a A: {path_indices}")

def graph_generation(directory, places):
    """
    Scarica la rete stradale da OpenStreetMap per una lista di comuni.
    Genera nodi e archi con indici dei nodi che partono da 0.
    """

    # Graph download
    G = ox.graph_from_place(places, network_type="drive", simplify=True)

    # Ordinamento nodi e mapping ID originale -> indice consecutivo
    ordered_nodes = list(G.nodes)
    node_id_to_index = {node_id: idx for idx, node_id in enumerate(ordered_nodes)}

    # ======================
    # Costruzione nodes.csv
    # ======================
    nodes_list = []
    for node_id in ordered_nodes:
        lat = G.nodes[node_id]["y"]
        lon = G.nodes[node_id]["x"]
        idx = node_id_to_index[node_id]  # indice da 0
        nodes_list.append((lat, lon))

    full_path = os.path.join(data_path, directory)
    os.makedirs(full_path, exist_ok=True)
    df_nodes = pd.DataFrame(nodes_list, columns=["lat", "lon"])
    df_nodes.to_csv(os.path.join(full_path, "nodes.csv"), index=False, header=False)

    # ======================
    # Costruzione edges.csv
    # ======================
    edges_list = []
    for u, v, k, data in G.edges(keys=True, data=True):
        x = node_id_to_index[u]
        y = node_id_to_index[v]

        weight = data.get("length", 1.0)  # metri
        name = data.get("name")
        if isinstance(name, list):
            name = name[0]
        elif not isinstance(name, str):
            name = "Unnamed"

        highway = data.get("highway")
        if isinstance(highway, list):
            highway = highway[0]
        elif not isinstance(highway, str):
            highway = "unknown"

        edges_list.append((x, y, weight, name, highway))

    df_edges = pd.DataFrame(edges_list, columns=["id_1", "id_2", "weight", "name", "highway"])
    df_edges.to_csv(os.path.join(full_path, "edges.csv"), index=False, header=False)

    # ======================
    # Info diagnostica
    # ======================
    print(f"Numero nodi: {len(G.nodes)}, numero archi: {len(G.edges)}")


if __name__ == "__main__":
    comuni = ["Provincia di Pisa, Italia", "Provincia di Livorno, Italia"]
    graph_generation("Provincia_Pisa+Livorno", comuni)