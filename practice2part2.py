import numpy as np
import math as m
import roboticstoolbox as rtb
from zmqRemoteApi import RemoteAPIClient
from practice2part1 import inv_f_kine, f_kine, l1, l2
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

sim.simxFinish(-1)  # just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to CoppeliaSim

client = RemoteAPIClient()
sim_ = client.getObject('sim')

if clientID != -1:
    print('Connected to remote API server\n')
    time.sleep(0.02)
    err, dummy = sim.simxGetObjectHandle(clientID, 'reference', sim.simx_opmode_oneshot_wait)
    err, floor = sim.simxGetObjectHandle(clientID, 'Floor', sim.simx_opmode_oneshot_wait)
    err, scara_coppeliasim = sim.simxGetObjectHandle(clientID, 'MTB', sim.simx_opmode_oneshot_wait)
    err, axis1 = sim.simxGetObjectHandle(clientID, 'axis1',sim.simx_opmode_oneshot_wait)
    err, axis2 = sim.simxGetObjectHandle(clientID, 'axis2',sim.simx_opmode_oneshot_wait)
    err, axis3 = sim.simxGetObjectHandle(clientID, 'axis3',sim.simx_opmode_oneshot_wait)
    err, axis4 = sim.simxGetObjectHandle(clientID, 'axis4',sim.simx_opmode_oneshot_wait)

    # Criando stream de dados
    err, position_dummy = sim.simxGetObjectPosition(clientID, dummy, floor, sim.simx_opmode_streaming)
    err, orientation_dummy = sim.simxGetObjectOrientation(clientID, dummy, floor, sim.simx_opmode_streaming)

    time.sleep(5)

    err, position_dummy = sim.simxGetObjectPosition(clientID, dummy, floor, sim.simx_opmode_buffer)
    err, orientation_dummy = sim.simxGetObjectOrientation(clientID, dummy, floor, sim.simx_opmode_buffer)

    orientation_dummy_degree = [d * (180 / m.pi) for d in orientation_dummy]
    print(f"Position Dummy: {position_dummy}\n")
    print(f"Orientation Dummy: {orientation_dummy_degree}\n")

    matrix_transformation = sim_.getObjectMatrix(dummy, floor)
    matrix_transformation = np.array(matrix_transformation).reshape((3, 4))

    move_position = inv_f_kine(matrix_transformation, l1, l2, 0, 0, 0)

    print(move_position)
    sim.simxSetJointPosition(clientID,axis1, move_position[0], sim.simx_opmode_oneshot_wait)
    sim.simxSetJointPosition(clientID,axis2, move_position[1], sim.simx_opmode_oneshot_wait)
    sim.simxSetJointPosition(clientID,axis3, move_position[2], sim.simx_opmode_oneshot_wait)
    sim.simxSetJointPosition(clientID,axis4, move_position[3], sim.simx_opmode_oneshot_wait)

    sim.simxGetPingTime(clientID)

    sim.simxFinish(clientID)
else:
    print('Failed connecting to remote API server')
print('Program ended')