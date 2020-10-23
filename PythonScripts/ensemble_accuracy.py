"""
Created on October 23

@author: nmajcher
"""

from scipy.stats import binom
import os

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
        k = n // 2
        success_proba = self.binomial_CCDF(k, n, self.p)
        while(success_proba < self.min_success_proba and n <= MAX_VOTES):
            success_proba = self.binomial_CCDF(k, n, self.p)
            n += 1
        self.min_n = n
        self.success_proba = success_proba
        return n

    def display_min_votes(self):
        print(self.category, self.min_n, self.success_proba)

    def binomial_CCDF(self, k, n, p):
        """ 
        Complementary CDF
        """
        return 1 - binom.cdf(k, n, p)

if __name__ == '__main__':

    categories = [('bed', 0.727),
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
    
    for elem in categories:
        accuracy = EnsembleAccuracy(category = elem[0] , individual_success_proba = elem[1], min_success_proba = MIN_SUCCESS_PROBA)
        accuracy.display_min_votes()