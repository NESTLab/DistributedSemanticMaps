# Import packages.
import glob
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import os

import numpy as np

import time
import timeout_decorator

# import cvxpy as cp

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
bins = 30
items = 80
np.random.seed(1)
neighbors = np.random.randint(1, bins, bins)
M = 15

# https://stackoverflow.com/questions/10035752/elegant-python-code-for-integer-partitioning

def partitions(n, I=1):
    yield (n,)
    for i in range(I, n//2 + 1):
        for p in partitions(n-i, i):
            yield (i,) + p

def accel_asc(n):
    a = [0 for i in range(n + 1)]
    k = 1
    y = n - 1
    while k != 0:
        x = a[k - 1] + 1
        k -= 1
        while 2 * x <= y:
            a[k] = x
            y -= x
            k += 1
        l = k + 1
        while x <= y:
            a[k] = x
            a[l] = y
            yield a[:k + 2]
            x += 1
            y -= 1
        a[k] = x + y
        y = x + y - 1
        yield a[:k + 1]

def sized_partitions(sum, max_val=100000, max_len=100000):
    """ generator of partitions of sum with limits on values and length """
    # Yields lists in decreasing lexicographical order. 
    # To get any length, omit 3rd arg.
    # To get all partitions, omit 2nd and 3rd args. 
    if sum <= max_val:       # Can start with a singleton.
        yield [sum]

    # Must have first*max_len >= sum; i.e. first >= sum/max_len.
    for first in range(min(sum-1, max_val), max(0, (sum-1)//max_len), -1):
        for p in sized_partitions(sum-first, first, max_len-1):
            yield [first]+p

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
        # Sort in ascending order
        a_partition = sorted(partition)
        # Impose volume constraint
        if(a_partition[-1] > memory_capacity):
            continue
        # Match largest num neighbors with largest part size
        prod = np.multiply(a_neighbors[-num_parts:], memory_capacity - np.array(a_partition))
        cur_cost = sum(np.divide(1, prod))
        if (cur_cost < min_cost):
            # print (min_cost, cur_cost)
            min_cost = cur_cost
            optimal_partition = list([0] * (num_robots - len(partition)) + list(a_partition))
    idx_unsort = idx_neighbors.argsort()
    opt_partition = np.array(optimal_partition)[idx_unsort]
    return min_cost, opt_partition

def solve_vscbpp_sized(total_tuples, num_robots, neighbors, memory_capacity):
    min_cost = num_robots
    optimal_partition = []
    # Sort in ascending order
    a_neighbors = sorted(neighbors)
    idx_neighbors = np.argsort(neighbors)
    assignment_partitions = sized_partitions(total_tuples, memory_capacity, num_robots)
    for partition in assignment_partitions:
        num_parts = len(partition)
        # Sort in ascending order
        a_partition = sorted(partition)
        # Match largest num neighbors with largest part size 
        prod = np.multiply(a_neighbors[-num_parts:], memory_capacity - np.array(a_partition))
        cur_cost = sum(np.divide(1, prod))
        if (cur_cost < min_cost):
            # print (min_cost, cur_cost)
            min_cost = cur_cost
            optimal_partition = list([0] * (num_robots - len(partition)) + list(a_partition))
    idx_unsort = idx_neighbors.argsort()
    opt_partition = np.array(optimal_partition)[idx_unsort]
    return min_cost, opt_partition 


def solve_vscbpp_accel(total_tuples, num_robots, neighbors, memory_capacity):
    time.sleep(5)
    min_cost = num_robots
    optimal_partition = []
    # Sort in ascending order
    a_neighbors = sorted(neighbors)
    idx_neighbors = np.argsort(neighbors)
    assignment_partitions = accel_asc(total_tuples)
    for partition in assignment_partitions:
        num_parts = len(partition)
        # Ignore partitions with too many parts
        if(num_parts >  num_robots): 
            continue
        # Sort in ascending order
        a_partition = sorted(partition)
        # Impose volume constraint
        if(a_partition[-1] > memory_capacity):
            continue
        # Match largest num neighbors with largest part size
        prod = np.multiply(a_neighbors[-num_parts:], memory_capacity - np.array(a_partition))
        cur_cost = sum(np.divide(1, prod))
        if (cur_cost < min_cost):
            # print (min_cost, cur_cost)
            min_cost = cur_cost
            optimal_partition = list([0] * (num_robots - len(partition)) + list(a_partition))
    idx_unsort = idx_neighbors.argsort()
    opt_partition = np.array(optimal_partition)[idx_unsort]
    return min_cost, opt_partition

start = time.time()
cost, optimal_partition = solve_vscbpp(items, bins, neighbors, M) 
stop = time.time()
print(stop - start)
print(cost)
print(neighbors)
print(optimal_partition)

start = time.time()
cost, optimal_partition = solve_vscbpp_sized(items, bins, neighbors, M) 
stop = time.time()
print(stop - start)
print(cost)
print(neighbors)
print(optimal_partition)

start = time.time()
cost, optimal_partition = solve_vscbpp_accel(items, bins, neighbors, M)
stop = time.time()
print(stop - start)
print(cost)
print(neighbors)
print(optimal_partition)

##################################################################################
#           Simulation
##################################################################################

########### Plotting bin packing cost vs time ###########################
# plt.figure()
# M = int(storage) + int(routing)
# for key in results_dict.keys():
#     results = results_dict[key]
#     tuples = load_dict[key]
#     x = []
#     y = []
#     y_opt = []
#     neighbors = np.zeros(num_robots)
#     cost = 0
#     for i, result in enumerate(results):
#         neighbors[result[1] - 1] = result[4]
#         free_memory = float(M - result[3])
#         if (result[3] != 0):
#             cost += 1 / (max(result[4], 1) * max(free_memory,1))
#         if((i+1)%num_robots == 0):
#             x.append(result[0] / 10)
#             y.append(cost)
#             print("t", result[0])
#             opt_cost, pa, idx = solve_vscbpp(tuples[i], num_robots, neighbors, M)
#             y_opt.append(opt_cost)
#             neighbors = np.zeros(num_robots)
#             cost = 0
#     plt.plot(x, y , label = key, alpha = 0.7)
#     plt.plot(x, y_opt, label = key + '*', alpha = 0.7)
# plt.xlabel('Time (sec)')
# plt.ylabel('Storage Cost')
# plt.legend()
# plt.show()


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

