#importing libraries
import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mtimg
import pickle

#now we must create the functions to create the line in the road tracking.
# lst's create a class called line.
# n - window size of moving average
class Line() :
    def __init__(self, n):
        self.n = n
        self.detected = False

        #polynominal coeffecients: x = A*y^2 + B*y + c
        #Each of A,B,C,D is a 'list-queue' with max length n
        self.A = []
        self.B = []
        self.C = []
        #creating avg values
        self.A.avg = 0.
        self.B.avg = 0.
        self.C.avg = 0.

    #fitting the line
    def get_fi(self):
        return (self.A_avg, self.B_avg, self.C_avg)

    def add_fit(self, fit_coeffs):
        #getting more line fit with updated more smooth coefficients.
        #fit_coeffs is a 3-elements of 2nd order polynominal coeffs.

        #coeffs queue is full?
        q_full = len(self.A) >= self.n

        #append the line fit coefficients
        self.A.append(fit_coeffs[0])
        self.B.append(fit_coeffs[1])
        self.C.append(fit_coeffs[2])

        #pop from index 0 if full
        if q_full:
            _ = self.A.pop(0)
            _ = self.B.pop(0)
            _ = self.C.pop(0)

        #simple average of line coefficients
        self.A_avg = np.mean(self.A)
        self.B_avg = np.mean(self.B)
        self.C_avg = np.mean(self.C)

        return (self.A_avg, self.B_avg, self.C_avg)
    
    