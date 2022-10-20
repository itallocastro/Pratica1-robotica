import numpy as np
import math as m
import roboticstoolbox as rtb
from zmqRemoteApi import RemoteAPIClient
import time
try:
    import sim
except:
    print('--------------------------------------------------------------')
    print('"sim.py" could not be imported. This means very probably that')
    print('either "sim.py" or the remoteApi library could not be found.')
    print('Make sure both are in the same folder as this file,')
    print('or appropriately adjust the file "sim.py"')
    print('--------------------------------------------------------------')
    print('')

def matrix_transformation(thetha, d, a, alpha):
    return np.array(
        [
            [np.cos(thetha), -np.sin(thetha) * np.cos(alpha), np.sin(thetha) * np.sin(alpha), a * np.cos(thetha)],
            [np.sin(thetha), np.cos(thetha) * np.cos(alpha), -np.cos(thetha) * np.sin(alpha), a * np.sin(thetha)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1],
        ]
    )


def f_kine(theta_1, theta_2, theta_3, d_4):
    l1 = 0.475
    l2 = 0.4
    return np.array(
        [
            [m.cos(theta_3 + theta_2 + theta_1), m.sin(theta_3 + theta_2 + theta_1), 0,
             l1 * m.cos(theta_1) + l2 * m.cos(theta_2 + theta_1)],
            [m.sin(theta_3 + theta_2 + theta_1), -m.cos(theta_3 + theta_2 + theta_1), 0,
             l1 * m.sin(theta_1) + l2 * m.sin(theta_2 + theta_1)],
            [0, 0, -1, -d_4],
            [0, 0, 0, 1]
        ]
    )

def calculate_f_kine(attr, scara):
    print(f"==== Attributes {attr} ====\n")
    print("Result Serial Link:")
    print(rtb.DHRobot.fkine(scara, attr))

    print("\nResult Our Fkine:")
    print(f_kine(attr[0], attr[1], attr[2], attr[3]))
    print()

def inv_f_kine(tm, l1, l2, theta1_ref, theta2_ref, theta3_ref):
    # x, y, z = tm[0:3, -1]
    x, y = tm[0:2, -1]
    z = 0
    phi = np.arctan2(tm[1, 0], tm[0, 0])
    if z != 0:
        raise Exception()
    c2 = (x ** 2 + y ** 2 - l1 ** 2 - l2 ** 2) / (2 * l1 * l2)
    if c2 > 1 or c2 < -1:
        raise Exception()
    s2_1 = np.sqrt(1 - (c2 ** 2))
    s2_2 = -np.sqrt(1 - (c2 ** 2))
    theta2_1 = np.arctan2(s2_1, c2)
    theta2_2 = np.arctan2(s2_2, c2)

    k1 = l2 * c2 + l1
    k2_1 = l2 * s2_1
    k2_2 = l2 * s2_2
    theta1_1 = np.arctan2(y, x) - np.arctan2(k2_1, k1)
    theta1_2 = np.arctan2(y, x) - np.arctan2(k2_2, k1)

    theta3_1 = phi - theta1_1 - theta2_1
    theta3_2 = phi - theta1_2 - theta2_2

    d4 = -tm[2, -1]

    opt1 = (theta1_ref - theta1_1)**2 + (theta2_ref - theta2_1)**2 + (theta3_ref - theta3_1)**2
    opt2 = (theta1_ref - theta1_2)**2 + (theta2_ref - theta2_2)**2 + (theta3_ref - theta3_2)**2

    if opt1 <= opt2:
        return theta1_1, theta2_1, theta3_1, d4
    return theta1_2, theta2_2, theta3_2, d4


l1 = 0.475
l2 = 0.4
offset = 0.1
dt = 0.025

if __name__ == "__main__":
    t01 = rtb.robot.DHLink(a=l1, offset=0)
    t12 = rtb.robot.DHLink(a=l2, offset=0)
    t23 = rtb.robot.DHLink(alpha=m.pi, offset=0)
    t34 = rtb.robot.DHLink(sigma=1, qlim=[0, 0.1])
    scara = rtb.robot.DHRobot([t01, t12, t23, t34], name='SCARA')

    print(scara)

    calculate_f_kine([0, 0, 0, 0], scara)
    calculate_f_kine([np.pi/2, -np.pi/2, 0, 0], scara)
    calculate_f_kine([np.pi/2, -np.pi/2, 0, 0.05], scara)


    print("Using a invfkine")
    example = f_kine(np.pi/2, -np.pi/2, 0, 0.05)
    print(example)
    print()
    inv_example = inv_f_kine(example, 0.475, 0.4, 0, 0, 0)
    print(inv_example)

