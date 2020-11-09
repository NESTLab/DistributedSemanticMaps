# Import packages.
import glob
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import os

import cvxpy as cp
import numpy as np

##################################################################################
#           Data import
##################################################################################

folder = 'runD'
n_robots = '*'
min_votes = '*'
seed = '1'
storage = '10'
routing = '5'
hashing_bucket = '5'
name = '/vcsbppfile_' + min_votes + '_' + n_robots + '_' + seed + '_' + storage + '_' + routing + '_' + hashing_bucket + '.dat'

path = os.path.abspath(os.path.join(os.getcwd(), '..', 'argos-application', 'data', folder))
file_names = glob.glob(path + name)
file_names = sorted(file_names)
print(file_names)

def get_data(file_name):
 
    results = []
    items = []

    ################ Reading file ##################################
    with open(file_name, 'r') as file:
        while True:
            line = file.readline()
            if not line:
                break
            timestep, num_robots = map(int, line.split(' '))
            total_tuples = 0
            for _ in range(num_robots):
                line = file.readline().strip('\n').split(' ')
                rid = line[0]
                node_id = int(line[1])
                num_tuples = int(line[2])
                neighbors = int(line[3])
                total_tuples += num_tuples
                results.append((timestep, rid, node_id, num_tuples, neighbors))
            items.append(total_tuples)
    return results, items

results_dict = {}
load_dict = {}

for name in file_names:
    n = name.split('/')
    n = n[-1].split('.')
    n = n[0].split('_')
    min_votes = n[1]
    num_robots = int(n[2])

    results, load = get_data(name)
    results_dict[min_votes] = results
    load_dict[min_votes] = load

##################################################################################
#           Simulation
##################################################################################

########### Plotting bin packing cost vs time ###########################
plt.figure()
M = int(storage) + int(routing)
for key in results_dict.keys():
    results = results_dict[key]
    x = []
    y = []
    cost = 0
    for i, result in enumerate(results):
        free_memory = float(M - result[3])
        if (result[3] != 0):
            cost += 1 / (max(result[4], 1) * max(free_memory,1))
        if((i+1)%num_robots == 0):
            x.append(result[0] // 10)
            y.append(cost)
            cost = 0
    plt.plot(x, y , label = key, alpha = 0.7)
plt.xlabel('Time (sec)')
plt.ylabel('Storage Cost')
plt.legend()
plt.show()

##################################################################################
#           Optimization
##################################################################################



# # Generate data.
# bins = 10
# items = 40
# np.random.seed(1)
# neighbors = np.random.randint(1, bins, bins)
# M = 20

# # Define and solve the CVXPY problem.
# assignment = cp.Variable((items, bins), boolean=True)
# selection = cp.Variable(bins, boolean=True)

# # cost = 1/cp.multiply(neighbors, M - cp.sum(assignment, axis=0))
# cost = 1/neighbors
# # objective = cp.sum(cp.multiply(cost, selection) + cp.multiply(cost/M, cp.max(cp.sum(assignment, axis=0))) )  
# objective = cp.sum(cp.multiply(cost, selection) + cp.multiply(1/M, cp.max(cp.sum(assignment, axis=0))) )  
# constraints = [
#     cp.sum(assignment, axis=1) == 1, 
#     cp.sum(assignment, axis=0) <= M * selection
# ]
# prob = cp.Problem(cp.Minimize(objective), constraints)
# prob.solve()

# # Print result.
# print("Neigbors", neighbors)
# print("\nThe optimal value is", prob.value)
# # print("The optimal assignment is")
# # print(assignment.value)
# # print("The optimal selection is")
# # print(selection.value)

# # for tau in range(len(assignment.value))
# print("The optimal assignment per bin is")
# print(np.sum(assignment.value, axis=0))

