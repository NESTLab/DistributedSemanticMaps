"""
Created on October 23

@author: nmajcher
"""

import matplotlib.pyplot as plt
from scipy.stats import binom
import numpy as np
import os
import matplotlib.colors as mcolors

MIN_SUCCESS_PROBA = 0.95
MAX_VOTES = 20

class EnsembleAccuracy:
    def __init__(self, category, individual_success_proba, min_success_proba):
        self.category = category
        self.p = individual_success_proba
        self.min_success_proba = min_success_proba
        self.find_min_votes()

    def find_min_votes(self):
        n = 2
        k = np.floor(n/2) + n%2
        success_proba = self.binomial_CCDF(k, n, self.p)
        while(success_proba < self.min_success_proba and n <= MAX_VOTES):
            n += 1
            k = np.floor(n/2) + n%2
            success_proba = self.binomial_CCDF(k, n, self.p) + 0.5 * binom.pmf(k, n, self.p) + 0.5 * (n%2) * binom.pmf(k, n, self.p)
            # print(success_proba, n)
        self.min_n = n
        self.success_proba = success_proba
        return n

    def display_min_votes(self):
        print(self.category, self.min_n, self.success_proba)

    def plot_accuracy_vs_number_votes(self, max_votes, ax):
        x = np.arange(1, max_votes)
        success_proba = []
        for n in x:
            k = np.floor(n/2) + n%2
            # print(k, n, self.p)
            success_proba.append(self.binomial_CCDF(k, n, self.p) + 0.5 * binom.pmf(k, n, self.p) + 0.5 * (n%2) * binom.pmf(k, n, self.p))
        ax.plot(x, success_proba, label=self.category)

    def binomial_CCDF(self, k, n, p):
        """ 
        Complementary CDF
        """
        return 1 - binom.cdf(k, n, p)

if __name__ == '__main__':

    categories = [
                ('overall', 0.799),
                ('bed', 0.727),
                ('toilet', 0.729),
                ('table', 0.741),
                ('desk', 0.773),
                ('pillow', 0.781),
                ('sink', 0.792),
                ('display', 0.804),
                ('shelf', 0.805),
                ('bin', 0.819),
                ('cabinet', 0.844),
                ('sofa', 0.91),
                ('door', 0.924),
                ('chair', 0.926)]
    
    NUM_COLORS = 14
    cm = plt.get_cmap('tab20')
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_color_cycle([cm(1.*i/NUM_COLORS) for i in range(NUM_COLORS)])
    for elem in categories:
        accuracy = EnsembleAccuracy(category = elem[0] , individual_success_proba = elem[1], min_success_proba = MIN_SUCCESS_PROBA)
        accuracy.display_min_votes()
        accuracy.plot_accuracy_vs_number_votes(25, ax)
        # break

    ax.legend()#fontsize='12')
    ax.set_ylabel('Ensemble Probability of Success', fontsize='14')
    ax.set_xlabel('Number of Votes', fontsize='14')
    ax.axvline(x=3, ls='-.', lw='1', c='k')
    ax.axvline(x=6, ls='-.', lw='1', c='k')
    plt.xticks(fontsize="14")
    plt.yticks(fontsize="14")
    ax.grid()
    plt.show()
