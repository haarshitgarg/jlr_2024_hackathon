import sys
import math


NO_PARENT = -1

class Dijkstra():
  def __init__(self, node_positions, avg_distance, angle_weights, angles, node_to_edge) -> None:
    self.NO_PARENT = -1
    self.node_positions = node_positions
    self.avg_distance = avg_distance
    self.angle_weights = angle_weights
    self.angles = angles
    self.node_to_edge = node_to_edge
    self.optimal_routes = {}

    print(self.angles)
    print(self.angle_weights)

  def turnCost(self, current_edge, next_edge):
    curr_start = self.node_positions[current_edge[0]]
    curr_end = self.node_positions[current_edge[1]]
    curr_vec = (curr_end[0] - curr_start[0], curr_end[1] - curr_start[1])

    next_start = self.node_positions[next_edge[0]]
    next_end = self.node_positions[next_edge[1]]
    next_vec = (next_end[0] - next_start[0], next_end[1] - next_start[1])

    curr_dot_next = curr_vec[0]*next_vec[0] + curr_vec[1]*next_vec[1]
    curr_mod = math.sqrt(curr_vec[0]**2 + curr_vec[1]**2)
    next_mod = math.sqrt(next_vec[0]**2 + next_vec[1]**2)
    cos_theta = curr_dot_next /(curr_mod * next_mod)

    if cos_theta <= -1:
      cos_theta = -1
    if cos_theta >= 1:
      cos_theta = 1

    theta_deg = 180.0 * (math.acos(cos_theta) / math.pi)
    print(f"{theta_deg}")


    factor = self.angle_weights[0]
    for i in range(len(self.angles)):
      factor = self.angle_weights[i]
      if theta_deg < self.angles[i]:
        break
    print(factor)

    # return 0

    return factor * self.avg_distance


  def dijkstra(self, adjacency_matrix, start_vertex):
    n_vertices = len(adjacency_matrix[0])

    # shortest_distances[i] will hold the
    # shortest distance from start_vertex to i
    shortest_distances = [sys.maxsize] * n_vertices

    # added[i] will true if vertex i is
    # included in shortest path tree
    # or shortest distance from start_vertex to
    # i is finalized
    added = [False] * n_vertices

    # Initialize all distances as
    # INFINITE and added[] as false
    for vertex_index in range(n_vertices):
      shortest_distances[vertex_index] = sys.maxsize
      added[vertex_index] = False
    # Distance of source vertex from
    # itself is always 0
    shortest_distances[start_vertex] = 0

    # Parent array to store shortest
    # path tree
    parents = [-1] * n_vertices

    # The starting vertex does not
    # have a parent
    parents[start_vertex] = NO_PARENT

    # Find shortest path for all
    # vertices
    for i in range(1, n_vertices):
      # Pick the minimum distance vertex
      # from the set of vertices not yet
      # processed. nearest_vertex is
      # always equal to start_vertex in
      # first iteration.
      nearest_vertex = -1
      shortest_distance = sys.maxsize
      for vertex_index in range(n_vertices):
        if not added[vertex_index] and shortest_distances[vertex_index] < shortest_distance:
          nearest_vertex = vertex_index
          shortest_distance = shortest_distances[vertex_index]

      # Mark the picked vertex as
      # processed
      added[nearest_vertex] = True

      # Update dist value of the
      # adjacent vertices of the
      # picked vertex.
      for vertex_index in range(n_vertices):
        edge_distance = adjacency_matrix[nearest_vertex][vertex_index]
        if edge_distance > 0 and added[vertex_index] == False:
          turn_cost = 0
          if parents[nearest_vertex] >= 0:
            print(f"{parents[nearest_vertex]}, {nearest_vertex}, {vertex_index}")
            turn_cost += self.turnCost((parents[nearest_vertex], nearest_vertex), (nearest_vertex, vertex_index))
          if (shortest_distance + edge_distance + turn_cost) < shortest_distances[vertex_index]:
            parents[vertex_index] = nearest_vertex
            shortest_distances[vertex_index] = shortest_distance + edge_distance + turn_cost

    return shortest_distances, parents


  # A utility function to print
  # the constructed distances
  # array and shortest paths
  def print_solution(self, start_vertex, distances, parents, node_map):
    n_vertices = len(distances)
    print("Vertex\t Distance\tPath")
    for vertex_index in range(n_vertices):
      if vertex_index != start_vertex:
        print("\n", node_map[start_vertex], "->", node_map[vertex_index], "\t\t", distances[vertex_index], "\t\t", end="")
        self.optimal_routes[vertex_index] = []
        self.print_path(vertex_index, vertex_index, parents, node_map)
    for index in self.optimal_routes:
      print(f"{node_map[index]} : {self.optimal_routes[index]}")
    # print(self.optimal_routes)


  # Function to print shortest path
  # from source to current_vertex
  # using parents array
  def print_path(self, start_index, current_vertex, parents, node_map):
    # Base case : Source node has
    # been processed
    if current_vertex == NO_PARENT:
      return
    self.print_path(start_index, parents[current_vertex], parents, node_map)
    if parents[current_vertex] != NO_PARENT:
      x = self.node_to_edge[parents[current_vertex]]
      y = current_vertex
      z = x[y]
      self.optimal_routes[start_index].append(z)

    print(node_map[current_vertex], end=" ")


  # Driver code
  if __name__ == '__main__':
      adjacency_matrix = [[0, 4, 0, 0, 0, 0, 0, 8, 0],
                [4, 0, 8, 0, 0, 0, 0, 11, 0],
                [0, 8, 0, 7, 0, 4, 0, 0, 2],
                [0, 0, 7, 0, 9, 14, 0, 0, 0],
                [0, 0, 0, 9, 0, 10, 0, 0, 0],
                [0, 0, 4, 14, 10, 0, 2, 0, 0],
                [0, 0, 0, 0, 0, 2, 0, 1, 6],
                [8, 11, 0, 0, 0, 0, 1, 0, 7],
                [0, 0, 2, 0, 0, 0, 6, 7, 0]]
      dijkstra(adjacency_matrix, 0)