from pathlib import Path
import unittest

from src.logistics import (
    astar,
    build_distance_matrix,
    clarke_wright_vrp,
    dijkstra,
    load_graph,
    nearest_neighbor_tsp,
    route_distance,
    two_opt,
)


class LogisticsAlgorithmsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.graph = load_graph(Path("data/paris_nodes.csv"), Path("data/paris_edges.csv"))

    def test_astar_and_dijkstra_find_same_distance(self) -> None:
        dijkstra_result = dijkstra(self.graph, "depot", "n08")
        astar_result = astar(self.graph, "depot", "n08")

        self.assertAlmostEqual(dijkstra_result.distance, astar_result.distance, places=6)
        self.assertEqual(dijkstra_result.path[0], "depot")
        self.assertEqual(dijkstra_result.path[-1], "n08")

    def test_two_opt_does_not_degrade_tour(self) -> None:
        points = ["depot", "n01", "n02", "n03", "n04", "n05", "n06", "n07", "n08"]
        matrix = build_distance_matrix(self.graph, points, astar)
        route = nearest_neighbor_tsp(points, matrix, "depot")
        improved = two_opt(route, matrix)

        self.assertLessEqual(route_distance(improved, matrix), route_distance(route, matrix))
        self.assertEqual(improved[0], "depot")
        self.assertEqual(improved[-1], "depot")

    def test_vrp_respects_capacity(self) -> None:
        deliveries = ["n01", "n02", "n03", "n04", "n05", "n06", "n07", "n08"]
        points = ["depot"] + deliveries
        matrix = build_distance_matrix(self.graph, points, astar)
        demands = {delivery: 1 for delivery in deliveries}
        routes = clarke_wright_vrp(deliveries, "depot", matrix, demands, capacity=3)

        visited = []
        for route in routes:
            self.assertEqual(route[0], "depot")
            self.assertEqual(route[-1], "depot")
            self.assertLessEqual(sum(demands.get(node, 0) for node in route), 3)
            visited.extend(node for node in route if node != "depot")

        self.assertCountEqual(visited, deliveries)


if __name__ == "__main__":
    unittest.main()
