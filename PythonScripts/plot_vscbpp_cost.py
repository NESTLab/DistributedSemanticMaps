import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import os
import glob

folder = 'runD'

name = '/heuristic_' + folder + '*'
path = os.path.abspath(os.path.join(os.getcwd(), '..', 'argos-application', 'data', folder))
file_name = glob.glob(path + name)
print(file_name)

x = []
y = []
################ Heuristic file ##################################
with open(file_name[0], 'r') as file:
    while True:
        line = file.readline().strip()
        if not line:
            break
        x.append(float(line))
        line = file.readline().strip()
        if not line:
            break
        y.append(float(line))

name = '/optimal_' + folder + '*'
path = os.path.abspath(os.path.join(os.getcwd(), '..', 'argos-application', 'data', folder))
file_name = glob.glob(path + name)
print(file_name)
x_opt = []
y_opt = []
################ Optimal file ##################################
with open(file_name[0], 'r') as file:
    while True:
        line = file.readline().strip()
        if not line:
            break
        x_opt.append(float(line))
        line = file.readline().strip()
        if not line:
            break
        y_opt.append(float(line))



########## Plotting bin packing cost vs time ###########################
plt.figure()
plt.plot(x, y , label = 'SwarmMesh', alpha = 0.7)
plt.plot(x_opt, y_opt, label = 'Optimal', alpha = 0.7)
plt.xlabel('Time (sec)')
plt.ylabel('Storage Cost')
plt.legend()
plt.show()