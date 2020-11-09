# Import packages.
import cvxpy as cp
import numpy as np

# Generate data.
bins = 10
items = 30
np.random.seed(1)
neighbors = np.random.randint(1, bins, bins)
M = 10

# Define and solve the CVXPY problem.
assignment = cp.Variable((items, bins), boolean=True)
selection = cp.Variable(bins, boolean=True)

# cost = 1/cp.multiply(neighbors, M - cp.sum(assignment, axis=0))
cost = 1/neighbors
# objective = cp.sum(cp.multiply(cost, selection) + cp.multiply(cost/M, cp.max(cp.sum(assignment, axis=0))) )  
objective = cp.sum(cp.multiply(cost, selection) + cp.multiply(1/M, cp.max(cp.sum(assignment, axis=0))) )  
constraints = [
    cp.sum(assignment, axis=1) == 1, 
    cp.sum(assignment, axis=0) <= M * selection
]
prob = cp.Problem(cp.Minimize(objective), constraints)
prob.solve()

# Print result.
print("Neigbors", neighbors)
print("\nThe optimal value is", prob.value)
# print("The optimal assignment is")
# print(assignment.value)
# print("The optimal selection is")
# print(selection.value)

# for tau in range(len(assignment.value))
print("The optimal assignment per bin is")
print(np.sum(assignment.value, axis=0))

