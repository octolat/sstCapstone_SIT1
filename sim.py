import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

instructions = """
F2.0
B90.0
F1.0
R45.0
B90.0
F1.4
B90.0
F1.0
R90.0
B90.0
F1.4
B90.0
F1.0
R45.0
B90.0
F2.0
B90.0
F1.0
""".strip().splitlines()

# --- Rotation helpers ---
def rot_x(theta):  # rotate about X-axis
    t = np.radians(theta)
    return np.array([
        [1, 0, 0],
        [0, np.cos(t), -np.sin(t)],
        [0, np.sin(t), np.cos(t)]
    ])

def rot_y(theta):  # rotate about Y-axis
    t = np.radians(theta)
    return np.array([
        [np.cos(t), 0, np.sin(t)],
        [0, 1, 0],
        [-np.sin(t), 0, np.cos(t)]
    ])

def rot_z(theta):  # rotate about Z-axis
    t = np.radians(theta)
    return np.array([
        [np.cos(t), -np.sin(t), 0],
        [np.sin(t), np.cos(t), 0],
        [0, 0, 1]
    ])

# --- Simulation setup ---
pos = np.array([0.0, 0.0, 0.0])
R = np.eye(3)  # orientation
points = [pos.copy()]

for line in instructions:
    cmd = line[0]
    val = float(line[1:])
    
    if cmd == "F":
        pos += R[:, 0] * val  # move along local x-axis
        points.append(pos.copy())
    elif cmd == "B":
        R = R @ rot_y(val)  # bend around local y-axis
    elif cmd == "R":
        R = R @ rot_x(val)  # rotate around local x-axis

points = np.array(points)

# --- Plot ---
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(points[:,0], points[:,1], points[:,2], marker='o')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title("3D Wire Bending Simulation")
ax.axis('equal')
plt.show()
