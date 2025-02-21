from math import atan2, acos, asin, sqrt, sin, cos, pi
import numpy as np
import math


x = 0.25
y = 0.15
z = 0.05

link = [0.0600, 0.0820, 0.1320, 0.1664, 0.0480, 0.0040]
q1 = math.atan2(y,x)

X = math.sqrt(x**2 + y**2) - math.cos(pi/2)*link[5] - math.sin(pi/2)*link[4]
Y = z - link[0] - link[1] + math.sin(pi/2)*link[5] - math.cos(pi/2)*link[4]
 # recursive to find q2 q3
Ts = 0.01
Xd = np.array([[X],
        [Y]])

K = np.array([[1, 0],
      [0, 1]])

q = np.array([[0.1], [0.1]])


#print('qdot',qdot)
#print('q',q)

finished = False
iter = 0

while not finished:
        iter += 1
        Kinematic = np.array([0.1664 * np.sin(q[0] + q[1]) + 0.1320 * np.sin(q[0]),0.1664 * np.cos(q[0] + q[1]) + 0.1320 * np.cos(q[0])])
      
        e = Xd-Kinematic
        et = np.transpose(e)

        if (np.dot(et,e) <= 1e-6).all():
            finished = True
        else:
            qa = q[0,0]
            qb = q[1,0]
            J = np.array([[0.1664 * np.cos(qa + qb) + 0.1320 * np.cos(qa), 0.1664 * np.cos(qa + qb)], 
                          [-0.1664 * np.sin(qa + qb) - 0.1320 * np.sin(qa), -0.1664 * np.sin(qa + qb)]])
        Jt = np.transpose(J)
        qdot = Jt.dot(K).dot(e)
        q += Ts * qdot

q2 = q[0,0]
q3 = q[1,0]

print(q2)
print(q3)
