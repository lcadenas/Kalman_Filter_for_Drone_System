from math import *
from matplotlib import pyplot as plt
import numpy as np

data = np.genfromtxt('data.csv', delimiter=',')

t = data[:, 0]

Px = data[:, 1]

Py = data[:, 2]

Pz = data[:, 3]

freq = 10
freq_num = 0

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
    global freq_num
    if freq_num % freq == 0:
        plt.plot(px, py, "--r")
    freq_num += 1

def kalman_filter(x, P, measurements, mean, var):
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
plt.title('Position of Drone at a certain time')

plt.subplot(1, 3, 1)
plt.plot(t, measurementsX)
plt.xlabel('Time (s)')
plt.ylabel('X Position (m)')

plt.subplot(1, 3, 2)
plt.plot(t, measurementsY)
plt.xlabel('Time (s)')
plt.ylabel('Y Position (m)')

plt.subplot(1, 3, 3)
plt.plot(t, measurementsZ)
plt.xlabel('Time (s)')
plt.ylabel('Z Position (m)')

plt.subplots_adjust(wspace = 0.35)

varX = []
meanX = []

print (kalman_filter(x1, P1, measurementsX, meanX, varX))
a = []
b =[]

plt.figure()
plt.subplot(3, 1, 1)
for i in range(len(meanX)):
   plot_covariance_ellipse(meanX[i], varX[i])
   a.append(meanX[i][0,0])
   b.append(meanX[i][1,0])
plt.plot(a, b)
plt.xlabel('X Position (m)')
plt.ylabel('X Velocity (m/s)')

x2 = np.matrix([[Py[0]], [0.]]) # initial state (location and velocity)
P2 = np.matrix([[0.01, 0.], [0., 0.01]]) # initial uncertainty
c = []
d =[]
varY = []
meanY = []
print (kalman_filter(x2, P2, measurementsY, meanY, varY))

#plt.figure()
plt.subplot(3, 1, 2)
for i in range(len(meanY)):
   plot_covariance_ellipse(meanY[i], varY[i])
   c.append(meanY[i][0,0])
   d.append(meanY[i][1,0])
plt.plot(c, d)
plt.xlabel('Y Position (m)')
plt.ylabel('Y Velocity (m/s)')

x3 = np.matrix([[Pz[0]], [0.]]) # initial state (location and velocity)
P3 = np.matrix([[0.01, 0.], [0., 0.01]]) # initial uncertainty
e = []
f =[]
varZ = []
meanZ = []
print (kalman_filter(x3, P3, measurementsZ, meanZ, varZ))

#plt.figure()
plt.subplot(3, 1, 3)
for i in range(len(meanZ)):
   plot_covariance_ellipse(meanZ[i], varZ[i])
   e.append(meanZ[i][0,0])
   f.append(meanZ[i][1,0])
plt.plot(e, f)
plt.xlabel('Z Position (m)')
plt.ylabel('Z Velocity (m/s)')
plt.subplots_adjust(hspace = 0.5)

plt.show()
