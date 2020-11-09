import glob

import numpy as np
import matplotlib.pyplot as plt
import os
import seaborn
import pandas as pd

folder = 'runD'
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


def get_data(file_name):
 
    results = []
    load = []
    bytes_sent = {}
    n_observations = []
    n_observed_categories = []

    ################ Reading file ##################################
    with open(file_name, 'r') as file:
        n_point_clouds = int(file.readline())
        while True:
            line = file.readline()
            if not line:
                break
            timestep, n_robots = map(int, line.split())
            for _ in range(n_robots):
                line = file.readline().split()
                n_results = int(line[2])
                
                message_size = int(line[1])
                if line[0] not in bytes_sent:
                    bytes_sent[line[0]] = [message_size]
                else:
                    bytes_sent[line[0]].append(message_size)

                for _ in range(n_results):
                    line = file.readline().split()
                    voted_category = line[0]
                    actual_category = line[1]
                    num_observations = int(line[2])
                    n_observations.append(num_observations)
                    total_time = int(line[3])
                    X = float(line[4])
                    Y = float(line[5])
                    Z = float(line[6])
                    results.append((timestep, voted_category, actual_category, num_observations, total_time, (X, Y, Z)))
            line = file.readline().split()
            load.append(float(line[0]))
            n_observed_categories.append(int(line[1]))
    return results, load, bytes_sent, n_observations, n_point_clouds, n_observed_categories

results_dict = {}
load_dict = {}
bytes_sent_dict = {}
n_observations_dict = {}
n_point_clouds_dict = {}
n_observed_categories_dict = {}

for name in file_names:
    n = name.split('/')
    n = n[-1].split('.')
    n = n[0].split('_')
    min_votes = n[1]

    results, load, bytes_sent, n_observations, n_point_clouds, n_observed_categories = get_data(name)
    results_dict[min_votes] = results
    load_dict[min_votes] = load
    bytes_sent_dict[min_votes] = bytes_sent
    n_observations_dict[min_votes] = n_observations
    n_point_clouds_dict[min_votes] = n_point_clouds
    n_observed_categories_dict[min_votes] = n_observed_categories





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
        if i % 10 == 0:
            x.append(result[0] // 10)
            y.append(accuracy)

    plt.plot(x, y, label = key, alpha = 0.7)
plt.xlabel('Time (sec)')
plt.ylabel('Accuracy ')
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
        x.append(result[0] // 10)
        coverage = len(seen) / n_point_clouds
        y.append(coverage)

    plt.plot(x, y, label = key, alpha = 0.7)
plt.xlabel('Time (sec)')
plt.ylabel('Consolidated Coverage')
# plt.title(n_robots + ' robots for differing min_votes')
plt.legend()


########### Plotting accuracy, coverage vs time ###########################
plt.figure()
for key in results_dict.keys():
    results = results_dict[key]
    x = [0]
    y = [0.0]
    correct = 0
    z = [0]
    seen = set()
    for i, result in enumerate(results):
        if result[1] == result[2]:
            correct += 1
        accuracy = correct / (i + 1)
        seen.add(result[5])
        coverage = len(seen) / n_point_clouds
        if i % 10 == 0:
            x.append(result[0] // 10)
            y.append(accuracy)
            z.append(coverage)

        

    plt.plot(x, y, label = key, alpha = 0.7)
    plt.plot(x, z, label = key, alpha = 0.7)
plt.xlabel('Time (sec)')
plt.ylabel('Accuracy, Coverage')
# plt.title(n_robots + ' robots for differing min_votes')
plt.legend()


# ######### Plotting coverage vs accuracy #############################
plt.figure()
for key in results_dict.keys():
    results = results_dict[key]
    n_point_clouds = n_point_clouds_dict[key]
    x = [0]
    y = [0.0]
    seen = set()
    correct = 0
    for i, result in enumerate(results):
        seen.add(result[5])
        coverage = len(seen) / n_point_clouds
        x.append(coverage)

        if result[1] == result[2]:
            correct += 1
        accuracy = correct / (i + 1)
        y.append(accuracy)


    plt.plot(x, y, label = key, alpha = 0.7)
plt.xlabel('Consolidated Coverage')
plt.ylabel('Accuracy')
# plt.title(n_robots + ' robots for differing min_votes')
plt.legend()

# ######### Plotting observation coverage vs time #############################
plt.figure()
for key in results_dict.keys():
    n_observed_categories = n_observed_categories_dict[key]
    n_point_clouds = n_point_clouds_dict[key]
    x = [0]
    y = [0.0]
    for i, n_observed in enumerate(n_observed_categories):
        y.append(n_observed / n_point_clouds)
        x.append((i + 1) // 10)

    plt.plot(x, y, label = key, alpha = 0.7)
plt.xlabel('Time (sec)')
plt.ylabel('Observation Coverage')
# plt.title(n_robots + ' robots for differing min_votes')
plt.legend()


# ######### Plotting observation coverage vs accuracy #############################
plt.figure()
for key in results_dict.keys():
    n_observed_categories = n_observed_categories_dict[key]
    n_point_clouds = n_point_clouds_dict[key]
    results = results_dict[key]
    x = [0]
    
    for i, n_observed in enumerate(n_observed_categories):
        x.append(n_observed / n_point_clouds)
    y = [0.0 for _ in range(len(x))]
    correct = 0
    min_time = results[0][0]
    for (i, result) in enumerate(results):
        if result[1] == result[2]:
            correct += 1
        accuracy = correct / (i + 1)
        timestep = result[0]
        y[timestep] = accuracy
    val = accuracy
    for i in range(len(x) - 1, min_time - 1, -1):
        if y[i] <= 0.001:
            y[i] = val
        else:
            val = y[i]


    plt.plot(x, y, label = key, alpha = 0.7)
plt.xlabel('Observation Coverage')
plt.ylabel('Accuracy')
# plt.title(n_robots + ' robots for differing min_votes')
plt.legend()




# ############ Plotting number of tuples stored vs time ###########
plt.figure()
for key in load_dict.keys():
    load =  load_dict[key]
    timesteps = np.arange(len(load))
    timesteps = timesteps / 10
    plt.plot(timesteps, load, label = key, alpha = 0.7)
plt.xlabel('Time (sec)')
plt.ylabel('Storage Load')
# plt.title(n_robots + ' robots for differing min_votes')
plt.legend()

# ############ Plotting bytes sent vs time ###########
plt.figure()
frames = []
for key in bytes_sent_dict.keys():
    bytes_sent = bytes_sent_dict[key] 
    temp = []
    for _, value in bytes_sent.items():
        temp.append(value)
    temp = np.array(temp)

    temp = temp.transpose()
    timesteps = temp.shape[0]
    n_robots = temp.shape[1]
    time = []
    data = []


    for i in range(timesteps):
        if i % 10 == 0:
            for j in range(n_robots):
                data.append(temp[i][j])
                time.append(i // 100)
    data = np.array(data)
    time = np.array(time)
    keys = [key] * time.shape[0]

    dataset = pd.DataFrame({'bytes': data, 'timesteps': time, 'votes': keys}, columns=['bytes', 'timesteps', 'votes'])
    frames.append(dataset)
dataset = pd.concat(frames)    
seaborn.boxplot(x = "timesteps", y = "bytes", hue = 'votes', data = dataset, whis = 500.0)

    # data = data[:, :1000]
    
    # max_array = np.max(data, axis = 0)
    # min_array = np.min(data, axis = 0)
    # mean_array = np.mean(data, axis = 0)
    # median_array = np.median(data, axis = 0)
    # data = np.vstack([max_array, min_array, mean_array, median_array])
    # new_median_array = []
    # new_min_array = []
    # new_max_array = []
    # new_mean_array = []
    # for i in range(0, median_array.shape[0], 10):
    #     new_median_array.append(np.median(data[:, i:i + 10]))
    #     new_mean_array.append(np.mean(data[:, i:i + 10]))
    #     new_min_array.append(np.min(min_array[i:i + 10]))
    #     new_max_array.append(np.max(max_array[i:i + 10]))

    # timesteps = np.arange(len(new_median_array))
    # # plt.scatter(timesteps, new_mean_array, marker = '.', label = key, alpha = 0.8)
    # plt.plot(timesteps, new_median_array, 'o', label = key)
    # for i in range(len(new_max_array)):
    #     plt.plot([timesteps[i], timesteps[i]], [new_min_array[i], new_max_array[i]], 'r-')

# plt.xlabel('Time (sec)')
# plt.ylabel('Bytes sent')
# plt.title(n_robots + ' robots for differing min_votes')
# plt.legend()


# # ######### Plotting histogram of number of observations #######
# plt.figure()
# for key in n_observations_dict.keys():
#     n_observations = n_observations_dict[key]
#     plt.hist(n_observations, bins = 10, label = key, alpha = 0.3)
# plt.xlabel('Number of observations')
# plt.ylabel('Count')
# plt.legend()
plt.show()
