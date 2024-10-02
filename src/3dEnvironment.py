import pybullet as pb
import numpy as np
import time
import csv


def read_csv_columns(file_path):
    with open(file_path, 'r') as csvfile:
        reader = csv.reader(csvfile)
        headers = next(reader)
        
        data = []
        for row in reader:
            converted_row = []
            for value in row:
                try:
                    converted_row.append(float(value))
                except ValueError:
                    converted_row.append(value)  # Keep original value if not convertible
            data.append(converted_row)
        
        # Transpose data to get columns
        columns = list(map(list, zip(*data)))
    
    return headers, columns



file_path = 'data.csv'
headers, columns = read_csv_columns(file_path)






def directionVectorToEuler(vector):
    x, y, z = vector
    pitch = np.arcsin(-y)
    yaw = np.arctan2(x, z)
    roll = 0
    pitch_deg = np.degrees(pitch)
    yaw_deg = np.degrees(yaw)
    roll_deg = np.degrees(roll)
    
    return pitch_deg, yaw_deg, roll_deg




physicsClient = pb.connect(pb.GUI)

cylinderStartPos = [columns[1][0], columns[2][0], columns[3][0]]
cylinderStartOrientation = pb.getQuaternionFromEuler(directionVectorToEuler([columns[4][0], columns[5][0], columns[6][0]]))


cylinderCollisionShapeId = pb.createCollisionShape(pb.GEOM_CYLINDER, radius=1, height=1)

# Optionally, create a visual shape for the cylinder (to render it)
cylinderVisualShapeId = pb.createVisualShape(pb.GEOM_CYLINDER, radius=1.85, length=41.2)

# Create the multi-body object with the visual and collision shapes
cylinderId = pb.createMultiBody(baseMass=1, 
                                baseCollisionShapeIndex=cylinderCollisionShapeId,
                                baseVisualShapeIndex=cylinderVisualShapeId,
                                basePosition=cylinderStartPos, 
                                baseOrientation=cylinderStartOrientation)

cameraDistance = 40
cameraYaw = 50
cameraPitch = -60

pb.setRealTimeSimulation(1)

start_time = time.time()

# Run the simulation for a certain time
count = 0
while True:
    # Get the real-time simulation clock
    elapsed_time = time.time() - start_time  # Real-time clock (seconds)
    
    while(elapsed_time > columns[0][count]):
        count = count + 1

    new_cylinder_pos = [columns[1][count], columns[2][count],columns[3][count]]  
    newCylinderOrientation = pb.getQuaternionFromEuler(directionVectorToEuler([columns[4][count], columns[5][count], columns[6][count]]))

    pb.resetBasePositionAndOrientation(cylinderId, new_cylinder_pos, newCylinderOrientation)
    
    # Get the current position of the cylinder
    cylinderPos, _ = pb.getBasePositionAndOrientation(cylinderId)
    
    # Update the camera view to lock onto the cylinder
    pb.resetDebugVisualizerCamera(cameraDistance, cameraYaw, cameraPitch, cylinderPos)
    
    # Sleep for a short duration to prevent excessive CPU usage
    time.sleep(1./240.)
