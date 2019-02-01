from math import *
from matplotlib import pyplot as plt
import numpy as np

data = np.genfromtxt('trjtracklog219.csv', delimiter=',')
t = data[:, 0]

Px = data[:, 4]

def plot_covariance_ellipse(xEst, PEst):
    Pxy = PEst[0:2, 0:2]
    eigval, eigvec = np.linalg.eig(Pxy)

    if eigval[0] >= eigval[1]:
        bigind = 0
        smallind = 1
    else:
        bigind = 1
        smallind = 0

    t = np.arange(0, 2 * pi + 0.1, 0.1)
    a = sqrt(eigval[bigind])
    b = sqrt(eigval[smallind])
    x = [a * cos(it) for it in t]
    y = [b * sin(it) for it in t]
    angle = atan2(eigvec[bigind, 1], eigvec[bigind, 0])
    R = np.matrix([[cos(angle), sin(angle)],
                   [-sin(angle), cos(angle)]])
    fx = R * np.matrix([x, y])
    px = np.array(fx[0, :] + xEst[0, 0]).flatten()
    py = np.array(fx[1, :] + xEst[1, 0]).flatten()
    plt.plot(px, py, "--r")

def kalman_filter(x, P):
    for n in range(len(measurements)):
        # prediction
        x = (F * x) + u
        P = F * P * F.T + Q
        
        # measurement update
        Z = np.matrix([[measurements[n]]])
        y = Z - (H * x)
        S = H * P * H.T + R
        K = P * H.T * S.I
        x = x + (K * y)
        
        P = (I - (K * H)) * P
        
        mean.append(x)
        var.append(P)       
    
    return x, P


x = np.matrix([[Px[0]], [0.]]) # initial state (location and velocity)
P = np.matrix([[0.01, 0.], [0., 0.01]]) # initial uncertainty
u = np.matrix([[0.], [0.]]) # external motion
F = np.matrix([[1., 0.1], [0, 1.]]) # next state function
H = np.matrix([[1., 0.]]) # measurement function
R = 0.001 # measurement uncertainty
Q = np.matrix([[0.00001, 0.0], [0.0, 0.000001]])
I = np.matrix([[1., 0.], [0., 1.]]) # identity matrix

measurements = Px
print measurements
plt.plot(t, measurements)

var = []
mean = []

print (kalman_filter(x, P))
a = []
b =[]

plt.figure()
for i in range(len(mean)):
   plot_covariance_ellipse(mean[i], var[i])
   a.append(mean[i][0,0])
   b.append(mean[i][1,0])

print a
print b

plt.plot(a, b)
plt.show()
