from __future__ import annotations

import csv
import heapq
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable


@dataclass(frozen=True)
class Node:
    id: str
    name: str
    lat: float
    lon: float


@dataclass(frozen=True)
class PathResult:
    distance: float
    path: list[str]
    visited: int
    runtime_ms: float


class Graph:
    def __init__(self) -> None:
        self.nodes: dict[str, Node] = {}
        self.adjacency: dict[str, list[tuple[str, float]]] = {}
        self.edge_count = 0

    def add_node(self, node: Node) -> None:
        self.nodes[node.id] = node
        self.adjacency.setdefault(node.id, [])

    def add_undirected_edge(self, a: str, b: str) -> None:
        distance = haversine_km(self.nodes[a], self.nodes[b])
        self.adjacency[a].append((b, distance))
        self.adjacency[b].append((a, distance))
        self.edge_count += 1


def haversine_km(a: Node, b: Node) -> float:
    radius = 6371.0
    lat1 = math.radians(a.lat)
    lat2 = math.radians(b.lat)
    dlat = math.radians(b.lat - a.lat)
    dlon = math.radians(b.lon - a.lon)
    value = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * radius * math.asin(math.sqrt(value))


def load_graph(nodes_path: Path, edges_path: Path) -> Graph:
    graph = Graph()

    with nodes_path.open(newline="", encoding="utf-8") as file:
        for row in csv.DictReader(file):
            graph.add_node(
                Node(
                    id=row["id"],
                    name=row["name"],
                    lat=float(row["lat"]),
                    lon=float(row["lon"]),
                )
            )

    with edges_path.open(newline="", encoding="utf-8") as file:
        for row in csv.DictReader(file):
            graph.add_undirected_edge(row["from"], row["to"])

    return graph


def reconstruct_path(previous: dict[str, str | None], goal: str) -> list[str]:
    path = [goal]
    current = goal
    while previous[current] is not None:
        current = previous[current] or ""
        path.append(current)
    path.reverse()
    return path


def dijkstra(graph: Graph, start: str, goal: str) -> PathResult:
    begin = time.perf_counter()
    distances = {node: math.inf for node in graph.nodes}
    previous: dict[str, str | None] = {node: None for node in graph.nodes}
    distances[start] = 0.0
    queue: list[tuple[float, str]] = [(0.0, start)]
    visited = 0

    while queue:
        current_distance, current = heapq.heappop(queue)
        if current_distance > distances[current]:
            continue

        visited += 1
        if current == goal:
            break

        for neighbor, weight in graph.adjacency[current]:
            new_distance = current_distance + weight
            if new_distance < distances[neighbor]:
                distances[neighbor] = new_distance
                previous[neighbor] = current
                heapq.heappush(queue, (new_distance, neighbor))

    runtime_ms = (time.perf_counter() - begin) * 1000
    return PathResult(distances[goal], reconstruct_path(previous, goal), visited, runtime_ms)


def astar(graph: Graph, start: str, goal: str) -> PathResult:
    begin = time.perf_counter()
    distances = {node: math.inf for node in graph.nodes}
    previous: dict[str, str | None] = {node: None for node in graph.nodes}
    distances[start] = 0.0
    queue: list[tuple[float, str]] = [(0.0, start)]
    closed: set[str] = set()
    visited = 0

    while queue:
        _, current = heapq.heappop(queue)
        if current in closed:
            continue
        closed.add(current)
        visited += 1

        if current == goal:
            break

        for neighbor, weight in graph.adjacency[current]:
            tentative = distances[current] + weight
            if tentative < distances[neighbor]:
                distances[neighbor] = tentative
                previous[neighbor] = current
                heuristic = haversine_km(graph.nodes[neighbor], graph.nodes[goal])
                heapq.heappush(queue, (tentative + heuristic, neighbor))

    runtime_ms = (time.perf_counter() - begin) * 1000
    return PathResult(distances[goal], reconstruct_path(previous, goal), visited, runtime_ms)


ShortestPathAlgo = Callable[[Graph, str, str], PathResult]
DistanceMatrix = dict[tuple[str, str], float]


def build_distance_matrix(graph: Graph, points: list[str], algo: ShortestPathAlgo) -> DistanceMatrix:
    matrix: DistanceMatrix = {}
    for origin in points:
        for destination in points:
            if origin == destination:
                matrix[(origin, destination)] = 0.0
            elif (destination, origin) in matrix:
                matrix[(origin, destination)] = matrix[(destination, origin)]
            else:
                matrix[(origin, destination)] = algo(graph, origin, destination).distance
    return matrix


def nearest_neighbor_tsp(points: list[str], matrix: DistanceMatrix, depot: str) -> list[str]:
    unvisited = set(points)
    unvisited.remove(depot)
    route = [depot]
    current = depot

    while unvisited:
        next_node = min(unvisited, key=lambda node: matrix[(current, node)])
        route.append(next_node)
        unvisited.remove(next_node)
        current = next_node

    route.append(depot)
    return route


def route_distance(route: list[str], matrix: DistanceMatrix) -> float:
    return sum(matrix[(route[i], route[i + 1])] for i in range(len(route) - 1))


def two_opt(route: list[str], matrix: DistanceMatrix) -> list[str]:
    best = route[:]
    improved = True

    while improved:
        improved = False
        best_distance = route_distance(best, matrix)
        for i in range(1, len(best) - 2):
            for j in range(i + 1, len(best) - 1):
                candidate = best[:i] + list(reversed(best[i : j + 1])) + best[j + 1 :]
                candidate_distance = route_distance(candidate, matrix)
                if candidate_distance + 1e-9 < best_distance:
                    best = candidate
                    improved = True
                    best_distance = candidate_distance
    return best


def clarke_wright_vrp(
    deliveries: list[str],
    depot: str,
    matrix: DistanceMatrix,
    demands: dict[str, int],
    capacity: int,
) -> list[list[str]]:
    routes = {delivery: [depot, delivery, depot] for delivery in deliveries}
    route_of = {delivery: delivery for delivery in deliveries}
    loads = {delivery: demands[delivery] for delivery in deliveries}

    savings: list[tuple[float, str, str]] = []
    for i, first in enumerate(deliveries):
        for second in deliveries[i + 1 :]:
            saving = matrix[(depot, first)] + matrix[(depot, second)] - matrix[(first, second)]
            savings.append((saving, first, second))
    savings.sort(reverse=True)

    for _, first, second in savings:
        first_route_key = route_of[first]
        second_route_key = route_of[second]
        if first_route_key == second_route_key:
            continue
        if loads[first_route_key] + loads[second_route_key] > capacity:
            continue

        first_route = routes[first_route_key]
        second_route = routes[second_route_key]

        oriented_first = orient_route(first_route, first, at_end=True, depot=depot)
        oriented_second = orient_route(second_route, second, at_end=False, depot=depot)
        if oriented_first is None or oriented_second is None:
            continue

        merged = oriented_first[:-1] + oriented_second[1:]
        new_key = first_route_key
        routes[new_key] = merged
        loads[new_key] += loads[second_route_key]
        del routes[second_route_key]
        del loads[second_route_key]
        for node in merged:
            if node != depot:
                route_of[node] = new_key

    return list(routes.values())


def orient_route(route: list[str], endpoint: str, at_end: bool, depot: str) -> list[str] | None:
    if at_end and route[-2] == endpoint:
        return route
    if at_end and route[1] == endpoint:
        return [depot] + list(reversed(route[1:-1])) + [depot]
    if not at_end and route[1] == endpoint:
        return route
    if not at_end and route[-2] == endpoint:
        return [depot] + list(reversed(route[1:-1])) + [depot]
    return None


def format_path(graph: Graph, path: list[str]) -> str:
    return " -> ".join(graph.nodes[node].name for node in path)


def print_matrix(graph: Graph, points: list[str], matrix: DistanceMatrix) -> None:
    labels = [graph.nodes[node].name[:8] for node in points]
    print(" " * 10 + "".join(f"{label:>10}" for label in labels))
    for origin in points:
        row = [f"{matrix[(origin, destination)]:10.2f}" for destination in points]
        print(f"{graph.nodes[origin].name[:8]:>10}" + "".join(row))


def benchmark(graph: Graph, depot: str, deliveries: list[str], sizes: list[int], capacity: int) -> None:
    print(f"{'n':>3} | {'Dijkstra ms':>11} | {'A* ms':>8} | {'TSP km':>8} | {'2-opt km':>8} | {'VRP km':>8}")
    print("-" * 65)
    for size in sizes:
        selected = deliveries[: min(size, len(deliveries))]
        points = [depot] + selected
        goal = selected[-1]

        dijkstra_result = dijkstra(graph, depot, goal)
        astar_result = astar(graph, depot, goal)
        matrix = build_distance_matrix(graph, points, astar)
        tsp_route = nearest_neighbor_tsp(points, matrix, depot)
        opt_route = two_opt(tsp_route, matrix)
        demands = {delivery: 1 for delivery in selected}
        vrp_routes = clarke_wright_vrp(selected, depot, matrix, demands, capacity)
        vrp_distance = sum(route_distance(route, matrix) for route in vrp_routes)

        print(
            f"{len(selected):3d} | "
            f"{dijkstra_result.runtime_ms:11.3f} | "
            f"{astar_result.runtime_ms:8.3f} | "
            f"{route_distance(tsp_route, matrix):8.2f} | "
            f"{route_distance(opt_route, matrix):8.2f} | "
            f"{vrp_distance:8.2f}"
        )
