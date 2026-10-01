from __future__ import annotations

import argparse
from pathlib import Path

from src.logistics import (
    astar,
    benchmark,
    build_distance_matrix,
    clarke_wright_vrp,
    dijkstra,
    format_path,
    load_graph,
    nearest_neighbor_tsp,
    print_matrix,
    route_distance,
    two_opt,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Optimisation simple de tournees logistiques")
    parser.add_argument("--deliveries", type=int, default=8, help="Nombre de points de livraison")
    parser.add_argument("--capacity", type=int, default=3, help="Capacite maximale de chaque vehicule")
    parser.add_argument("--benchmark", action="store_true", help="Lancer les benchmarks")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    graph = load_graph(Path("data/paris_nodes.csv"), Path("data/paris_edges.csv"))

    depot = "depot"
    all_deliveries = [node_id for node_id in graph.nodes if node_id != depot]
    deliveries = all_deliveries[: max(1, min(args.deliveries, len(all_deliveries)))]

    print("=== Graphe charge ===")
    print(f"Noeuds: {len(graph.nodes)} | Aretes: {graph.edge_count}")
    print(f"Depot: {graph.nodes[depot].name}")
    print(f"Livraisons utilisees: {len(deliveries)}\n")

    start = depot
    goal = deliveries[-1]

    dijkstra_result = dijkstra(graph, start, goal)
    astar_result = astar(graph, start, goal)

    print("=== Plus court chemin ===")
    print(f"Dijkstra vers {graph.nodes[goal].name}: {dijkstra_result.distance:.2f} km")
    print(format_path(graph, dijkstra_result.path))
    print(f"A*: {astar_result.distance:.2f} km")
    print(format_path(graph, astar_result.path))
    print()

    points = [depot] + deliveries
    matrix = build_distance_matrix(graph, points, astar)

    print("=== Matrice de distances (km) ===")
    print_matrix(graph, points, matrix)
    print()

    print("=== TSP: plus proche voisin + 2-opt ===")
    initial_tour = nearest_neighbor_tsp(points, matrix, depot)
    improved_tour = two_opt(initial_tour, matrix)
    print(f"Tour initial: {format_path(graph, initial_tour)}")
    print(f"Distance initiale: {route_distance(initial_tour, matrix):.2f} km")
    print(f"Tour ameliore: {format_path(graph, improved_tour)}")
    print(f"Distance amelioree: {route_distance(improved_tour, matrix):.2f} km")
    print()

    print("=== VRP: Clarke & Wright simplifie ===")
    demands = {delivery: 1 for delivery in deliveries}
    routes = clarke_wright_vrp(deliveries, depot, matrix, demands, args.capacity)
    total = 0.0
    for index, route in enumerate(routes, start=1):
        distance = route_distance(route, matrix)
        total += distance
        load = sum(demands.get(node, 0) for node in route)
        print(f"Vehicule {index}: charge {load}/{args.capacity} | {distance:.2f} km")
        print(f"  {format_path(graph, route)}")
    print(f"Distance totale VRP: {total:.2f} km\n")

    if args.benchmark:
        print("=== Benchmark ===")
        benchmark(graph, depot, all_deliveries, sizes=[4, 6, 8, 10, 12], capacity=args.capacity)


if __name__ == "__main__":
    main()
