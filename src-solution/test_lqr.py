'''A simple example script, which implements an LQR controller for a double integrator.
'''

from __future__ import print_function, division

import controlpy
import scipy
import numpy as np

# Example system is a double integrator:
A = np.matrix([[0,1],[0,0]])
B = np.matrix([[0],[1]])

# Define our costs:
Q = np.matrix([[1,0],[0,0]])
R = np.matrix([[1]])

# Compute the LQR controller
gain, X, closedLoopEigVals = controlpy.synthesis.controller_lqr(A,B,Q,R)


M = 2;
m = 5;
b = 0.5;
I = 0.10500391309238813;
g = 9.8;
l = 0.1;

px = I*(M+m)+pow(M*m*l,2) #denominator for the A and B matrices

A = np.array([[0,1],[m*g*l*(M+m)/px,0]])#considering theta and theta dot
B = np.array([[  0.  ],
       [ m*l/px]])

Q = np.array([[ 10,   0],[  0, 1000]])
R = [[0.0001]]
gain, X, closedLoopEigVals = controlpy.synthesis.controller_lqr(A,B,Q,R)
print('The computed gain is:')
print(gain)

print('The closed loop eigenvalues are:')
print(closedLoopEigVals)


