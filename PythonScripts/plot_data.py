import numpy as np
import matplotlib.pyplot as plt
import os

file_name = os.path.abspath(os.path.join(os.getcwd(), '..', 'argos-application', 'outputfile.dat'))

results = []
n_timesteps = 1000


################ Reading file ##################################
with open(file_name, 'r') as file:
    n_point_clouds = int(file.readline())
    for _ in range(n_timesteps):
        timestep, n_robots = map(int, file.readline().split(' '))
        for _ in range(n_robots):
            line = file.readline().split(' ')
            n_results = int(line[1])
            for _ in range(n_results):
                line = file.readline().split(' ')
                voted_category = line[0]
                actual_category = line[1]
                num_observations = int(line[2])
                total_time = int(line[3])
                X = float(line[4])
                Y = float(line[5])
                Z = float(line[6])
                results.append((timestep, voted_category, actual_category, num_observations, total_time, (X, Y, Z)))
        num_tuples = int(file.readline())
########### Plotting accuracy vs time ###########################
x = [0]
y = [0.0]
correct = 0
for i, result in enumerate(results):
    if result[1] == result[2]:
        correct += 1
    accuracy = correct / (i + 1)
    x.append(result[0])
    y.append(accuracy)

plt.plot(x, y)
plt.xlabel('Time')
plt.ylabel('Accuracy')
plt.show()

######### Plotting coverage vs time #############################
x = [0]
y = [0.0]
seen = set()
for result in results:
    seen.add(result[5])
    x.append(result[0])
    coverage = len(seen) / n_point_clouds
    y.append(coverage)
plt.plot(x, y)
plt.xlabel('Time')
plt.ylabel('Coverage')
plt.show()