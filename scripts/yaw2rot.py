import numpy as np

roll, pitch, yaw = [], [], []

def read_gt():
    with open("rpy.txt", 'r') as file:
        for line in file:
            data = line.strip().split("\t")
            a, b, c = map(float, data)
            roll.append(a)
            pitch.append(b)
            yaw.append(c)

def euler_to_rotation_matrix(roll, pitch, yaw):
    r = np.radians(roll)
    p = np.radians(pitch)
    y = np.radians(yaw)
    
    R_roll = np.array([
        [1, 0, 0],
        [0, np.cos(r), -np.sin(r)],
        [0, np.sin(r), np.cos(r)]
    ])
    
    R_pitch = np.array([
        [np.cos(p), 0, np.sin(p)],
        [0, 1, 0],
        [-np.sin(p), 0, np.cos(p)]
    ])
    
    R_yaw = np.array([
        [np.cos(y), -np.sin(y), 0],
        [np.sin(y), np.cos(y), 0],
        [0, 0, 1]
    ])
    
    return R_yaw @ R_pitch @ R_roll

def save_rotation_matrices_to_file(roll, pitch, yaw, output_file="rotation_matrix.txt"):

    with open(output_file, 'w') as file:
        for r, p, y in zip(roll, pitch, yaw):
            rot_matrix = euler_to_rotation_matrix(r, p, y)
            rot_flat = rot_matrix.flatten()  
            rot_line = "\t".join(map(str, rot_flat))
            file.write(rot_line + "\n")

read_gt()
save_rotation_matrices_to_file(roll, pitch, yaw)
