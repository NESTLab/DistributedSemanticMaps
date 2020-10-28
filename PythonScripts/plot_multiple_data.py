import glob

import numpy as np
import matplotlib.pyplot as plt
import os


folder = 'runC'
n_robots = '60'
min_votes = '*'
seed = '1'
storage = '10'
routing = '5'
hashing_bucket = '5'
name = '/outputfile_' + min_votes + '_' + n_robots + '_' + seed + '_' + storage + '_' + routing + '_' + hashing_bucket + '.dat'



path = os.path.abspath(os.path.join(os.getcwd(), '..', 'argos-application', 'data', folder))
file_names = glob.glob(path + name)
file_names = sorted(file_names)
print(file_names)


def get_data(file_name):
 
    results = []
    load = []
    bytes_sent = {}
    n_observations = []

    ################ Reading file ##################################
    with open(file_name, 'r') as file:
        n_point_clouds = int(file.readline())
        while True:
            line = file.readline()
            if not line:
                break
            timestep, n_robots = map(int, line.split(' '))
            for _ in range(n_robots):
                line = file.readline().split(' ')
                n_results = int(line[2])
                
                message_size = int(line[1])
                if line[0] not in bytes_sent:
                    bytes_sent[line[0]] = [message_size]
                else:
                    bytes_sent[line[0]].append(message_size)

                for _ in range(n_results):
                    line = file.readline().split(' ')
                    voted_category = line[0]
                    actual_category = line[1]
                    num_observations = int(line[2])
                    n_observations.append(num_observations)
                    total_time = int(line[3])
                    X = float(line[4])
                    Y = float(line[5])
                    Z = float(line[6])
                    results.append((timestep, voted_category, actual_category, num_observations, total_time, (X, Y, Z)))
            load.append(float(file.readline()))
    return results, load, bytes_sent, n_observations, n_point_clouds

results_dict = {}
load_dict = {}
bytes_sent_dict = {}
n_observations_dict = {}
n_point_clouds_dict = {}

for name in file_names:
    n = name.split('/')
    n = n[-1].split('.')
    n = n[0].split('_')
    min_votes = n[1]

    results, load, bytes_sent, n_observations, n_point_clouds = get_data(name)
    results_dict[min_votes] = results
    load_dict[min_votes] = load
    bytes_sent_dict[min_votes] = bytes_sent
    n_observations_dict[min_votes] = n_observations
    n_point_clouds_dict[min_votes] = n_point_clouds





########### Plotting accuracy vs time ###########################
plt.figure()
for key in results_dict.keys():
    results = results_dict[key]
    x = [0]
    y = [0.0]
    correct = 0
    for i, result in enumerate(results):
        if result[1] == result[2]:
            correct += 1
        accuracy = correct / (i + 1)
        x.append(result[0])
        y.append(accuracy)

    plt.plot(x, y, label = key, alpha = 0.7)
plt.xlabel('Time')
plt.ylabel('Accuracy')
# plt.title(n_robots + ' robots for differing min_votes')
plt.legend()

# ######### Plotting coverage vs time #############################
plt.figure()
for key in results_dict.keys():
    results = results_dict[key]
    n_point_clouds = n_point_clouds_dict[key]
    x = [0]
    y = [0.0]
    seen = set()
    for result in results:
        seen.add(result[5])
        x.append(result[0])
        coverage = len(seen) / n_point_clouds
        y.append(coverage)

    plt.plot(x, y, label = key, alpha = 0.7)
plt.xlabel('Time')
plt.ylabel('Coverage')
# plt.title(n_robots + ' robots for differing min_votes')
plt.legend()



# ############ Plotting number of tuples stored vs time ###########
plt.figure()
for key in load_dict.keys():
    load =  load_dict[key]
    timesteps = np.arange(len(load))
    plt.plot(timesteps, load, label = key, alpha = 0.7)
plt.xlabel('Time')
plt.ylabel('Storage Load')
# plt.title(n_robots + ' robots for differing min_votes')
plt.legend()

# ############ Plotting bytes sent vs time ###########
plt.figure()
for key in bytes_sent_dict.keys():
    bytes_sent = bytes_sent_dict[key] 
    data = []
    for key, value in bytes_sent.items():
        data.append(value)
    plt.boxplot(data, patch_artist = True, 
                notch = True, whis=(0, 100))
plt.xlabel('Time')
plt.ylabel('Bytes sent')
# plt.title(n_robots + ' robots for differing min_votes')
plt.legend()


# # ######### Plotting histogram of number of observations #######
# plt.figure()
# for key in n_observations_dict.keys():
#     n_observations = n_observations_dict[key]
#     plt.hist(n_observations, bins = 10, label = key, alpha = 0.3)
# plt.xlabel('Number of observations')
# plt.ylabel('Count')
# plt.legend()
plt.show()
