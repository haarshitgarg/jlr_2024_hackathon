import traci
import sumolib
import math
import djikstra
import numpy as np
import random

sumoBinary = sumolib.checkBinary('sumo-gui')

traci.start([sumoBinary, "-c", "testMatrix.sumocfg"])
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






step = 0
i = 0
while step < 1000:
  traci.simulationStep()



  if step% 2 == 0 and i < 199:
    id = "rout" + str(step)
    traci.route.add(id,rout[i])
    traci.vehicle.add(vehID=id, routeID=id)

    id = "kau" + str(step)
    traci.route.add(id,new_rout[i])
    traci.vehicle.add(vehID=id, routeID=id)

    id = "harshit" + str(step)
    traci.route.add(id,new2_rout[i])
    traci.vehicle.add(vehID=id, routeID=id)

    i += 1




  if step % 100 == 0:
    list_of_vehicles = traci.vehicle.getIDList()
    print(list_of_vehicles)
    print("Step: ", step)

  step += 1

  #print("==========================")

traci.close()