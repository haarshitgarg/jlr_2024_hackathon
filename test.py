import traci
import sumolib
import math
import numpy as np
import sys
from dijkstra_angle import Dijkstra
import matplotlib.pyplot as plt
import numpy as np

avg_distance = 0.0
angle_weights = np.array([0, 0.3, 3, 5])
angles = np.array([10, 50, 150, 180])

sumoBinary = sumolib.checkBinary('sumo-gui')

sumo_cfg = "random_sumo.sumocfg"
traci.start([sumoBinary, "-c", sumo_cfg])

def getWeight(start, end):
  start_x, start_y = traci.junction.getPosition(start)
  end_x, end_y = traci.junction.getPosition(end)
  return math.sqrt((start_x - end_x)**2 + (start_y - end_y)**2)

def isEdge(edge):
  if traci.edge.getFromJunction(edgeID=edge) == traci.edge.getToJunction(edgeID=edge):
    return False
  return True

junction_map = {}
node_map = {}
node_positions = {}
edges = []
node_to_edge = {}
i = 0
for edge in traci.edge.getIDList():
  if isEdge(edge):
    edges += [edge]
    start = traci.edge.getFromJunction(edgeID=edge)
    end = traci.edge.getToJunction(edgeID=edge)

    if start not in junction_map:
      junction_map[start] = i
      node_map[i] = start
      node_positions[i] = traci.junction.getPosition(start)
      i += 1

    if end not in junction_map:
      junction_map[end] = i
      node_map[i] = end
      node_positions[i] = traci.junction.getPosition(end)
      i += 1

print(junction_map)

V = len(junction_map)
adj = np.zeros((V, V))

for edge in edges:
    start = traci.edge.getFromJunction(edgeID=edge)
    end = traci.edge.getToJunction(edgeID=edge)
    if junction_map[start] not in node_to_edge:
      node_to_edge[junction_map[start]] = {junction_map[end]: edge}
    else:
      node_to_edge[junction_map[start]].update({junction_map[end]: edge})

    # if junction_map[end] not in node_to_edge:
    #   node_to_edge[junction_map[end]] = {junction_map[start]: edge}
    # else:
    #   node_to_edge[junction_map[end]].update({junction_map[start]: edge})

    weight = getWeight(start, end)

    print(f"edge: {edge}, {weight}")
    avg_distance += weight

    i = junction_map[start]
    j = junction_map[end]
    adj[i][j] = weight
    adj[j][i] = weight
print(adj)

avg_distance /= len(edges)

print(node_to_edge)
dijkstra = Dijkstra(node_positions=node_positions, avg_distance=avg_distance, angle_weights=angle_weights, angles=angles, node_to_edge=node_to_edge)

shortest_distances, parents = dijkstra.dijkstra(adjacency_matrix=adj, start_vertex=junction_map['1403'])
dijkstra.print_solution(junction_map['1403'], shortest_distances, parents, node_map)

path = dijkstra.optimal_routes[junction_map['1770']]
# path = path[::-1]
print("My path: ", path)

traci.route.add("kaustabh", path)
traci.vehicle.add(vehID="patel", routeID="kaustabh")
steps = []
speeds = []
i = 0
while i < 1000:
  traci.simulationStep()

  if "patel" in traci.vehicle.getIDList():
    speed = traci.vehicle.getSpeed("patel")
    speeds += [speed]
    steps += [i]

#   for vehicle_id in traci.vehicle.getIDList():
#     speed = traci.vehicle.getSpeed(vehicle_id)
#     print(f"Step: {step}, VehicleID: {vehicle_id}, Speed: {speed}")

#   for traffic_light_id in traci.trafficlight.getIDList():
#     if step < 20:
#       traci.trafficlight.setRedYellowGreenState(traffic_light_id, 'r')
#     else:
#       traci.trafficlight.setRedYellowGreenState(traffic_light_id, 'G')
#     print(f"Step: {step}, VehicleID: {vehicle_id}")

#   print("==========================")

  i += 1

traci.close()

plt.plot(steps, speeds)
plt.show()