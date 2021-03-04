#!/usr/bin/env python

import numpy as np
import math
import matplotlib as plt

# Displaying the contents of the text file 
file = open("/home/akshit/clifford_sim_ws/src/clifford_sim/data/file1.txt", "r") 
content = file.read() 

print(type(content))