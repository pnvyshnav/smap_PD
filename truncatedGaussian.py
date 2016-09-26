from scipy.stats import norm
import numpy as np

from paramList import *


class TruncatedGaussian:
    def __init__(self, mean_inp, std_inp, lb_inp, ub_inp):
        self.lb = lb_inp
        self.ub = ub_inp
        if Par.sensor_truncatedGaussianNoise is False:
            self.lb = -np.inf
            self.ub = np.inf
        self.GaussianMean = mean_inp
        self.GaussianStd = std_inp
        self.GaussianArea = norm(0,1).cdf((ub_inp-self.GaussianMean)/self.GaussianStd) - norm(0,1).cdf((lb_inp-self.GaussianMean)/self.GaussianStd)

    def pdfValue(self, x):
        return (1./(self.GaussianStd*self.GaussianArea))*norm(0,1).pdf((x-self.GaussianMean)/self.GaussianStd)

    def cdfValue(self,x):
        return (1. / self.GaussianArea)*(norm(0,1).cdf((x-self.GaussianMean)/self.GaussianStd) - norm(0,1).cdf((self.lb-self.GaussianMean)/self.GaussianStd))

    def sample(self):
        # wikipedia says below algorithm is slow and not very accurate... see wiki for better smapling methods
        u = np.random.rand()
        cdf_tmp = norm(0,1).cdf((self.lb-self.GaussianMean)/self.GaussianStd)
        quantile_tmp = norm.ppf(self.GaussianArea*u + cdf_tmp)
        return self.GaussianMean + self.GaussianStd*quantile_tmp