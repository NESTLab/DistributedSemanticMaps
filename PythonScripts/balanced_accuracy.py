# -*- coding: utf-8 -*-
"""
Created on Sat Jul 18 15:12:20 2020

@author: danieljeswin
"""


from scipy.stats import beta
from scipy.optimize import fsolve, fmin
import numpy as np
import matplotlib.pyplot as plt
import os
import pandas as pd
import yaml


PRECISION = 1e-3
NUM_STARTING_POINTS = 10
STARTING_POINTS = np.arange(0.0, 1.1, 1.0 / NUM_STARTING_POINTS)


class PosteriorBalAccuracy:
    def __init__(self, confusion_matrix = None, beta_params = None):
        """
        confusion_matrix : array, shape = [n_classes, n_classes] (optional)
            Where entry c_{ij} is the number of observations in class i but
            are classified as class j. 
        beta_params: beta parameters from confusion_matrix (optional)
        
        Either confusion_matrix or beta_params must be non null
        """
        if confusion_matrix is not None:
            assert confusion_matrix.shape[0] == confusion_matrix.shape[1], "confusion_matrix must be a square matrix"
            self.confusion_matrix = confusion_matrix
            self.n_classes = self.confusion_matrix.shape[0]
            
            self.get_beta_params()
            self.sum_beta_distributions()
        else:
            assert beta_params is not None, "Both beta_params and confusion_matrix cannot be None"
            self.beta_params = beta_params
            self.n_classes = len(self.beta_params)
            self.sum_beta_distributions()
            
    
    def get_beta_params(self):    
        alphas = []
        betas = []
        
        total_instances_per_class = np.sum(self.confusion_matrix, axis = 1)
        for i in range(self.n_classes):
            alphas.append(self.confusion_matrix[i][i] + 1)
            betas.append(total_instances_per_class[i] - self.confusion_matrix[i][i] + 1)
        
        self.beta_params = list(zip(alphas, betas))

    def sum_beta_distributions(self):

        x = np.arange(0, self.n_classes + PRECISION, PRECISION)
        k = 1.0 / self.n_classes
    
        for (a, b) in self.beta_params:
            distribution = beta.pdf(x, a, b)
            fft = np.fft.fft(distribution)
            k = k * fft        
            
        y = np.fft.ifft(k)
        
        y = y[:len(x)]
        x = []
        for i in y:
            x.append(np.abs(i))
        y = np.array(x)
        y = y / np.sum(y)
        self.posterior_distribution = y
        return y
      
    def balanced_accuracy_mean(self):
        k = 1.0 / self.n_classes
        x = np.arange(0, self.n_classes + PRECISION, PRECISION)
        
        self.mean = k * np.dot(x, self.posterior_distribution)

        return self.mean
    
    def beta_sum_cdf(self, x):
        
        index = np.round(x / PRECISION)
        index = int(index)
        if index < 1:
            area = 0
        elif index > len(self.posterior_distribution):
            area = 1
        else:
            area = np.trapz(self.posterior_distribution[:index])
        return area
    
    def beta_avg_cdf(self, x, diff):
        avg_pdf = self.beta_sum_cdf(self.n_classes * x) - diff
    
        return avg_pdf
        
    def beta_avg_inv(self, x):
        best_error = float('inf')
        best_avg_inf = 0.0
        for starting_point in STARTING_POINTS:
            avg_inv = fsolve(self.beta_avg_cdf, x0 = starting_point, args = (x,))[0]
            
            error = self.beta_avg_cdf(avg_inv, x)
            if abs(error) < best_error:
                best_error = abs(error)
                best_avg_inv = avg_inv
          
        return best_avg_inv
    
    def balanced_accuracy_median(self):
        self.median = self.beta_avg_inv(0.5)
        
        return self.median
     
    def beta_sum_pdf(self, x):        
        index = int(x / PRECISION) + 1
        y = np.zeros_like(x)
        y[y == 0] = self.posterior_distribution[index]
        y[x < 0] = 0
        y[x > self.n_classes] = 0
        
        return y
            
    def beta_avg_pdf(self, x):
        y = self.beta_sum_pdf(self.n_classes * x) * self.n_classes
        y[y < 0] = 0
        return -y
    
    def balanced_accuracy_mode(self):
        self.balanced_accuracy_mean()
        self.mode = fmin(self.beta_avg_pdf, x0 = self.mean)[0]
    
        return self.mode
    
    def naive_balanced_accuracy(self):
        total = 0.0
        for (a, b) in self.beta_params:
            a = a - 1.0
            b = b - 1.0
            total += a / (a + b)
        
        self.naive_accuracy = total / self.n_classes
        return self.naive_accuracy
    
    def probability_interval(self, alpha):
        lower = self.beta_avg_inv(alpha / 2)  
        upper = self.beta_avg_inv(1 - alpha / 2)
        
        return [lower, upper]
    
    def balanced_accuracy_confidence_interval(self, confidence):
        alpha = 1 - confidence
        return self.probability_interval(alpha)
    
    def display_stats(self):
        self.balanced_accuracy_mean()
        self.balanced_accuracy_median()
        self.balanced_accuracy_mode()
        self.naive_balanced_accuracy()
        
        confidence = 0.95
        interval = self.balanced_accuracy_confidence_interval(confidence)
        
        print(f"Balanced Accuracy Mean : {self.mean}")
        print(f"Balanced Accuracy Median : {self.median}")
        print(f"Balanced Accuracy Mode : {self.mode}")
        print(f"Naive Balanced Accuracy : {self.naive_accuracy}")
        print(f"{confidence * 100}% confidence interval : {interval}")
        
        # step = PRECISION / self.n_classes
        # x = np.arange(0, 1 + step, step)
        # plot = plt.plot(x, self.posterior_distribution)
        # plt.legend(plot[:1], ['posterior balanced accuracy pdf'])
        # plt.xlabel('Balanced Accuracy')
        # plt.ylabel('Probability')
        # plt.show()
        
    def save_distribution(self, file_name):
        file_name = file_name + '.txt'
        file_path = os.path.join(os.getcwd(), 'data', file_name)
        
        np.savetxt(file_path, self.posterior_distribution)
        



def get_beta_params(method):
    classes = counts.keys()
    
    alphas = []
    betas = []
    
    for c in classes:
        count = counts[c]
        accuracy = data[data['Method'] == method][c]
        
        tp = int(np.round((accuracy * count) / 100.0))
        fn = count - tp
        alphas.append(tp + 1)
        betas.append(fn + 1)
    
    return list(zip(alphas, betas))

if __name__ == '__main__':
    os.chdir('..')  # Changing working directory to DistributedSemanticMaps
    
    csv_path = os.path.join(os.getcwd(), 'data', 'benchmark.csv')
    data = pd.read_csv(csv_path)

    yaml_path = os.path.join(os.getcwd(), 'config', 'classes.yml')
    with open(yaml_path, 'r') as f:
        counts = yaml.load(f, Loader = yaml.FullLoader)

    for method in data['Method']:
        beta_params = get_beta_params(method)
        post = PosteriorBalAccuracy(beta_params = beta_params)
        post.display_stats()
        post.save_distribution(method)


