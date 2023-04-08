#!/usr/bin/env python3

# read text files and plot them

import matplotlib.pyplot as plt
import numpy as np
import sys
import math

# data file to read 
file_name_e1 = "../data_files/question_e1.txt"
file_name_e2 = "../data_files/question_e2.txt"
file_name_f1 = "../data_files/question_f1.txt"
file_name_f2 = "../data_files/question_f2.txt"
file_name_g = "../data_files/question_g.txt"

mass_e1 = np.loadtxt(file_name_e1, skiprows=0)
mass_e2 = np.loadtxt(file_name_e2, skiprows=0)
grav_f1 = np.loadtxt(file_name_f1, skiprows=0)
grav_f2 = np.loadtxt(file_name_f2, skiprows=0)
grav_g = np.loadtxt(file_name_g, skiprows=0)


q3_dense = np.linspace(-90,90,251)
q3 = (-90, -60, -30, 0, 30, 60, 90)

d2_dense = np.linspace(0,2,251)
d2 = (0, 0.5, 1, 1.5, 2)