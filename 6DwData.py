from math import *
from matplotlib import pyplot as plt
import numpy as np

data = np.genfromtxt('trjtracklog219.csv', delimiter=',')

t = data[:, 0]

Px = data[:, 4]

Py = data[:, 5]

Pz = data[:, 6]

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

def kalman_filter_in_X(x, P):
    for n in range(len(measurementsX)):
        # prediction
        x = (F * x) + u
        P = F * P * F.T + Q
        
        # measurement update
        Z = np.matrix([[measurementsX[n]]])
        y = Z - (H * x)
        S = H * P * H.T + R
        K = P * H.T * S.I
        x = x + (K * y)
        
        P = (I - (K * H)) * P
        
        meanX.append(x)
        varX.append(P)       
    
    return x, P


x1 = np.matrix([[Px[0]], [0.]]) # initial state (location and velocity)
P1 = np.matrix([[0.01, 0.], [0., 0.01]]) # initial uncertainty
u = np.matrix([[0.], [0.]]) # external motion
F = np.matrix([[1., 0.1], [0, 1.]]) # next state function
H = np.matrix([[1., 0.]]) # measurement function
R = 0.001 # measurement uncertainty
Q = np.matrix([[0.00001, 0.0], [0.0, 0.000001]])
I = np.matrix([[1., 0.], [0., 1.]]) # identity matrix

measurementsX = Px
measurementsY = Py
measurementsZ = Pz
#print measurementsX
plt.subplot(1, 3, 1)
plt.plot(t, measurementsX)
plt.subplot(1, 3, 2)
plt.plot(t, measurementsY)
plt.subplot(1, 3, 3)
plt.plot(t, measurementsZ)

varX = []
meanX = []

print (kalman_filter_in_X(x1, P1))
a = []
b =[]

plt.figure()
plt.subplot(1, 3, 1)
for i in range(len(meanX)):
   plot_covariance_ellipse(meanX[i], varX[i])
   a.append(meanX[i][0,0])
   b.append(meanX[i][1,0])

#print a
#print b

plt.plot(a, b)


def kalman_filter_in_Y(x, P):
    for n in range(len(measurementsY)):
        # prediction
        x = (F * x) + u
        P = F * P * F.T + Q
        
        # measurement update
        Z = np.matrix([[measurementsY[n]]])
        y = Z - (H * x)
        S = H * P * H.T + R
        K = P * H.T * S.I
        x = x + (K * y)
        
        P = (I - (K * H)) * P
        
        meanY.append(x)
        varY.append(P)       
    
    return x, P

x2 = np.matrix([[Py[0]], [0.]]) # initial state (location and velocity)
P2 = np.matrix([[0.01, 0.], [0., 0.01]]) # initial uncertainty
c = []
d =[]
meanY = []
varY = []

print (kalman_filter_in_Y(x2, P2))

#plt.figure()
plt.subplot(1, 3, 2)
for i in range(len(meanY)):
   plot_covariance_ellipse(meanY[i], varY[i])
   c.append(meanY[i][0,0])
   d.append(meanY[i][1,0])

#print a
#print b

plt.plot(c, d)

def kalman_filter_in_Z(x, P):
    for n in range(len(measurementsZ)):
        # prediction
        x = (F * x) + u
        P = F * P * F.T + Q
        
        # measurement update
        Z = np.matrix([[measurementsZ[n]]])
        y = Z - (H * x)
        S = H * P * H.T + R
        K = P * H.T * S.I
        x = x + (K * y)
        
        P = (I - (K * H)) * P
        
        meanZ.append(x)
        varZ.append(P)       
    
    return x, P

x3 = np.matrix([[Pz[0]], [0.]]) # initial state (location and velocity)
P3 = np.matrix([[0.01, 0.], [0., 0.01]]) # initial uncertainty
e = []
f =[]
meanZ = []
varZ = []

print (kalman_filter_in_Z(x3, P3))

#plt.figure()
plt.subplot(1, 3, 3)
for i in range(len(meanZ)):
   plot_covariance_ellipse(meanZ[i], varZ[i])
   e.append(meanZ[i][0,0])
   f.append(meanZ[i][1,0])

#print a
#print b

plt.plot(e, f)

plt.show()
