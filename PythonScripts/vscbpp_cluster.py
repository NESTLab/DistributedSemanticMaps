# Import packages.
import glob
import numpy as np
import os

# import cvxpy as cp

##################################################################################
#           Data import
##################################################################################

folder = 'runD'
n_robots = '30'
min_votes = '3'
seed = '1'
storage = '10'
routing = '10'
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

# https://stackoverflow.com/questions/10035752/elegant-python-code-for-integer-partitioning
# See vscbpp_testing for more detail
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

def solve_vscbpp_accel(total_tuples, num_robots, neighbors, memory_capacity):
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
    # Unsort back to initial neighbor order 
    idx_unsort = idx_neighbors.argsort()
    opt_partition = np.array(optimal_partition)[idx_unsort]
    return min_cost, opt_partition

##################################################################################
#           Bin Packing
##################################################################################

########### Saving bin packing cost over time ###########################

#### In simulation ####

M = int(storage) + int(routing)
for key in results_dict.keys():
    results = results_dict[key]
    tuples = load_dict[key]
    x = []
    y = []
    # assignments = []
    cost = 0
    # assignment = np.zeros(num_robots)
    for i, result in enumerate(results):
        # assignment[result[1] - 1] = result[3]
        free_memory = float(M - result[3])
        if (result[3] != 0):
            cost += 1 / max(result[4] * free_memory, 1)
        if((i+1)%num_robots == 0):
            print(result[0], tuples[result[0]-1])
            x.append(result[0] / 10)
            y.append(cost)
            cost = 0
            # assignments.append(assignment)
            # assignment = np.zeros(num_robots)
    # Write to file (made to match optimal, want to have a partial file if takes too long)
    with open("heuristic_" + folder + '_' + key + '_' + n_robots + ".txt", "w") as f:
        for i,j in zip(x,y):
            f.write(str(i) +"\n")
            f.write(str(j) +"\n")

#### Optimal solution ####

# M = int(storage) + int(routing)
# for key in results_dict.keys():
#     results = results_dict[key]
#     tuples = load_dict[key]
#     x_opt = []
#     y_opt = []
#     neighbors = np.zeros(num_robots)
#     for i, result in enumerate(results):
#         neighbors[result[1] - 1] = result[4]
#         if((i+1)%num_robots == 0):
#             print("t", result[0])
#             # Skip time steps 
#             if (result[0]%10 != 0):
#                 continue
#             opt_cost, pa = solve_vscbpp_accel(tuples[result[0]-1], num_robots, neighbors, M)
#             x_opt = result[0] / 10
#             y_opt = opt_cost
#             neighbors = np.zeros(num_robots)
#             with open("optimal_" + folder + '_' + key + '_' + n_robots + ".txt", "a") as f:
#                 f.write(str(x_opt) +"\n")
#                 f.write(str(y_opt) +"\n")

#### Worst cost ####

M = int(storage) + int(routing)
for key in results_dict.keys():
    results = results_dict[key]
    tuples = load_dict[key]
    x_worst = []
    y_worst = []
    worst_cost = 0
    neighbors = np.zeros(num_robots)
    for i, result in enumerate(results):
        neighbors[result[1] - 1] = result[4]
        if((i+1)%num_robots == 0):
            items = tuples[result[0]-1]
            # Put one item in all bins with 0 neighbors (assuming low enough load factor)
            zero_neighbors = len(neighbors) - np.count_nonzero(neighbors)
            if(items > zero_neighbors):
                worst_cost += zero_neighbors
                items -= zero_neighbors
            if items > 0 and items < M: 
                # Put all in one bin 
                worst_cost += 1
            else:
                # Sort number of neighbors in ascending order
                a_neighbors = sorted(neighbors)
                # Fill out memory of bins with lowest degree
                num_bins_to_fill = items // M
                worst_cost += num_bins_to_fill
                items -= num_bins_to_fill * M # same as modulo
                # Put remaining items in next lowest 
                if(items > 0):
                    n_low = a_neighbors[zero_neighbors + num_bins_to_fill - 1]
                    free_memory = M - items
                    worst_cost += 1 / (max(n_low * free_memory, 1))
            x_worst = result[0] / 10
            y_worst = worst_cost
            neighbors = np.zeros(num_robots)
            worst_cost = 0
            with open("worst_" + folder + '_' + key + '_' + n_robots + ".txt", "a") as f:
                f.write(str(x_worst) +"\n")
                f.write(str(y_worst) +"\n")

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

