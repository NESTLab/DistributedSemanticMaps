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
                rid = int(line[0][1:])
                print(rid)
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
#           Optimization
##################################################################################

# Generate fake data
# bins = 10
# items = 40
# np.random.seed(1)
# neighbors = np.random.randint(1, bins, bins)
# M = 15

def partitions(n, I=1):
    yield (n,)
    for i in range(I, n//2 + 1):
        for p in partitions(n-i, i):
            yield (i,) + p

def solve_vscbpp(total_tuples, num_robots, neighbors, memory_capacity):
    min_cost = num_robots
    optimal_partition = []
    # Sort in ascending order
    a_neighbors = sorted(neighbors)
    idx_neighbors = np.argsort(neighbors)
    assignment_partitions = partitions(total_tuples)
    for partition in assignment_partitions:
        num_parts = len(partition)
        # Ignore partitions with too many parts
        if(num_parts >  num_robots): 
            continue
        # Sort in descending order
        d_partition = sorted(partition, reverse=True)
        # Impose volume constraint
        if(d_partition[0] > memory_capacity):
            continue
        # Match largest num neighbors with lowest part size
        prod = np.multiply(a_neighbors[-num_parts:], M - np.array(d_partition))
        cur_cost = sum(np.divide(1, prod))
        if (cur_cost < min_cost):
            # print (min_cost, cur_cost)
            min_cost = cur_cost
            optimal_partition = list([0] * (num_robots - len(partition)) + list(partition))
    return min_cost, optimal_partition, idx_neighbors

# cost, optimal_partition, idx = solve_vscbpp(items, bins, neighbors, M) 
# print(cost)
# # print(np.arange(1, bins+1))
# print(np.array(optimal_partition)[idx])

##################################################################################
#           Simulation
##################################################################################

########### Plotting bin packing cost vs time ###########################
plt.figure()
M = int(storage) + int(routing)
for key in results_dict.keys():
    results = results_dict[key]
    tuples = load_dict[key]
    x = []
    y = []
    y_opt = []
    neighbors = np.zeros(num_robots)
    cost = 0
    for i, result in enumerate(results):
        neighbors[result[1] - 1] = result[4]
        free_memory = float(M - result[3])
        if (result[3] != 0):
            cost += 1 / (max(result[4], 1) * max(free_memory,1))
        if((i+1)%num_robots == 0):
            x.append(result[0] // 10)
            y.append(cost)
            print("t", result[0])
            opt_cost, pa, idx = solve_vscbpp(tuples[i], num_robots, neighbors, M)
            y_opt.append(opt_cost)
            neighbors = np.zeros(num_robots)
            cost = 0
    plt.plot(x, y , label = key, alpha = 0.7)
    plt.plot(x, y_opt, label = key + '*', alpha = 0.7)
plt.xlabel('Time (sec)')
plt.ylabel('Storage Cost')
plt.legend()
plt.show()


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

