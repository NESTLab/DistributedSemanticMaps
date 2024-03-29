import numpy as np

import matplotlib.pyplot as plt
import os

file_name = os.path.abspath(os.path.join(os.getcwd(), '..', 'argos-application', 'histogramfile.dat'))

node_ids = []
hashes = []

################ Reading file ##################################
with open(file_name, 'r') as file:
    n_robots = int(file.readline())
    while True:
        line = file.readline()
        if not line:
            break    
        timestep = int(line)
        
        for _ in range(n_robots):
            node_id, num_tuples = map(int, file.readline().split(' '))
            for _ in range(num_tuples):
                identifier, hash = map(int, file.readline().split(' '))
                hashes.append(hash)
            node_ids.append(node_id)

plt.hist(node_ids, bins = n_robots, alpha = 0.5, label = 'Node IDs')
plt.hist(hashes, bins = n_robots, alpha = 0.5, label = 'Hashes')
plt.xlabel('ID')
plt.ylabel('Count')
plt.legend()
plt.show()