import traci
import sumolib
import math
import numpy as np
import sys
from dijkstra_angle import Dijkstra
import matplotlib.pyplot as plt
import numpy as np
import random

##############################################################################
###################################################################

sumoBinary = sumolib.checkBinary('sumo-gui')

traci.start([sumoBinary, "-c", "random_sumo.sumocfg"])
list_of_vehicles = traci.vehicle.getIDList()

list_of_edges = {}
rev_edge = {}
list_of_junctions = {}

i = 0
j = 0

for edge_id in traci.edge.getIDList():
  start = traci.edge.getFromJunction(edge_id)
  end = traci.edge.getToJunction(edge_id)

  if start != end:
    if edge_id not in list_of_edges:
      list_of_edges[edge_id] = i
      rev_edge[i] = edge_id
      i += 1
      if start not in list_of_junctions:
        list_of_junctions[start] = j
        j += 1
      if end not in list_of_junctions:
        list_of_junctions[end] = j
        j += 1

matrix_l = len(list_of_junctions)
adj = []
for i in range(0,matrix_l):
  arr = []
  for j in range(0,matrix_l):
    arr.append("")
  adj.append(arr)


edges = list_of_edges.keys()
print(edges)

for edge_id in edges:
  start = traci.edge.getFromJunction(edge_id)
  end = traci.edge.getToJunction(edge_id)

  if start!=end:
      adj[list_of_junctions[start]][list_of_junctions[end]] = edge_id
      # adj[list_of_junctions[end]][list_of_junctions[start]] = edge_id


def random_route(index,parent,adj,route):
  print(route)
  if(len(route)>=10):
    return None
  # index = list_of_junctions[junction]
  list = adj[index]
  next_edges =[]
  for i in list:
    if i!="" and i!=list[parent]:
      next_edges.append(i)
  if len(next_edges)==0:
    return None
  if len(next_edges)!=0:
    choice = random.choice(next_edges)
    ind = list.index(choice)
    route.append(choice)
    print(index)
    parent = index
    route = random_route(ind,parent,adj,route)

  return None

rout = []
for i in range(200):
  temp = []
  random_route(0,-1,adj,temp)
  rout.append(temp)

new_rout = []
for i in range(200):
  temp = []
  random_route(5,-1,adj,temp)
  new_rout.append(temp)

new2_rout = []
for i in range(200):
  temp = []
  random_route(9,-1,adj,temp)
  new2_rout.append(temp)

##############################################################################
############KAUSTABH KA COMMAND###############################################

avg_distance = 0.0
angle_weights = np.array([0, 0.3, 3, 5])
angles = np.array([10, 50, 150, 180])

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
traci.vehicle.setColor("patel", color=(255, 0, 0, 255))
steps = []
speeds = []
step = 0
for edge in edges:
  traci.edge.setMaxSpeed(edge, 14)

while step < 1000:
  traci.simulationStep()

  if "patel" not in traci.vehicle.getIDList():
    break

  edge_id = traci.vehicle.getRoadID(vehID="patel")
  light_id = traci.edge.getToJunction(edgeID=edge_id)
  if light_id in traci.trafficlight.getIDList() and edge_id in edges :
    next_edge_id = path[path.index(edge_id) + 1]
    phase_index = 0

    for p in range(len(traci.trafficlight.getControlledLinks(light_id))):
      c = traci.trafficlight.getControlledLinks(light_id)[p]
      x, y, _ = c[0]
      x = x.split("_")[0]
      y = y.split("_")[0]

      if(x == edge_id and y == next_edge_id):
        phase_index = p
        print("Eurekaaaa. ", x, " Foff ", y, "Index: ", p)
        break

    start_x, start_y = traci.vehicle.getPosition("patel")
    end_x, end_y = traci.junction.getPosition(light_id)
    distance_to_reach = math.sqrt((start_x - end_x)**2 + (start_y - end_y)**2)
    speed = 14
    time_to_reach = distance_to_reach/speed

    logic = traci.trafficlight.getAllProgramLogics(light_id)[0]
    logic_phases = logic.phases

    curr_time_to_reach = time_to_reach
    prevPhase = traci.trafficlight.getPhase(light_id)
    nextPhase = (prevPhase + 1) % len(logic_phases)
    st = traci.trafficlight.getNextSwitch(light_id) - traci.simulation.getTime()
    curr_time_to_reach = curr_time_to_reach - st
    while(curr_time_to_reach > 0):
      print("Current Time to Reach: ", curr_time_to_reach)
      print("prevPhase: ", prevPhase, "nextPhase: ", nextPhase)
      prevPhase = nextPhase
      nextPhase = (prevPhase + 1) % len(logic_phases)
      curr_time_to_reach = curr_time_to_reach - logic_phases[nextPhase].duration

    myState = logic_phases[prevPhase].state[phase_index]
    if(myState == 'G'):
      traci.vehicle.setSpeed(vehID="patel", speed=14)
      print("Balle balee green light")
    else :
      timeToAdd = traci.trafficlight.getNextSwitch(light_id) - traci.simulation.getTime()
      myPhase_ = nextPhase
      z = 0
      while(logic_phases[myPhase_].state[phase_index] != 'G'):
        timeToAdd += logic_phases[myPhase_].duration
        myPhase_  = (myPhase_ + 1)%len(logic_phases)
        z += 1
        if(z > 14):
          print("Haryeya me dil hareyaa")
          break

      new_time = time_to_reach + timeToAdd + 3
      new_speed = distance_to_reach/new_time
      print("New speed: ", new_speed)
      traci.vehicle.setSpeed(vehID="patel", speed=new_speed)










    current_state = traci.trafficlight.getRedYellowGreenState(light_id)
    current_definition = traci.trafficlight.getProgram(light_id)
    current_program = traci.trafficlight.getCompleteRedYellowGreenDefinition(light_id)
    print("Current state at junction: ", light_id, " state: ", current_state)
    print("Current definitio at junction: ", light_id, " state: ", current_definition)
    print("Lanes: ", traci.trafficlight.getControlledLanes(light_id))
    print("Program: ", traci.trafficlight.getCompleteRedYellowGreenDefinition(light_id))
    print("Phase: ", traci.trafficlight.getPhase(light_id))
    print("Current Program at junction: ", light_id, " state: ", current_program)
    print("Current Program logic: ", traci.trafficlight.getAllProgramLogics(light_id))
    logic = traci.trafficlight.getAllProgramLogics(light_id)[0]


    print("Type: ", type(logic))
    print("Type: ", len(logic.phases))

  if "patel" in traci.vehicle.getIDList():
    speed = traci.vehicle.getSpeed("patel")
    speeds += [speed]
    steps += [step]

  step += 1

traci.close()

plt.plot(steps, speeds)
plt.show()
