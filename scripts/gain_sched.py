#!/usr/bin/env python

import numpy as np
import math

init_grad = np.linspace(20,40,5).reshape(1,5)  # initial range of inclines
print(init_grad.shape)

gain_dict = {20: {'kp': 1.25, 'kd': 0.20, 'ki': 0.02},
              25: {'kp': 1.55, 'kd': 0.25, 'ki': 0.025},
              30: {'kp': 1.75, 'kd': 0.30, 'ki': 0.032},
              35: {'kp': 1.95, 'kd': 0.35, 'ki': 0.0352},
              40: {'kp': 2.05, 'kd': 0.40, 'ki': 0.042}}

grad_arr = [] # array of slope angles

# Linear Interpolation
for i in range(init_grad.shape[1]-1):
    x1 = gain_dict[init_grad[0,i]]
    x3 = gain_dict[init_grad[0,i+1]]
    
    x2 =  math.floor((init_grad[0,i] + init_grad[0,i+1]) / 2)

    gain1_arr = np.array([x1['kp'], x1['ki'], x1['kd']])
    gain3_arr = np.array([x3['kp'], x3['ki'], x3['kd']])
    
    y2 = ((x2 -init_grad[0,i])*(gain3_arr-gain1_arr) / (init_grad[0,i+1])) + gain1_arr
    print(y2)
    
for x,y in gain_dict.items():
    pass
    # print(x,":", y)
    #print(y['ki'])