import math
import numpy as np

# --- 1. Define Spiral Parameters ---
num_mics = 16
r_min = 12.0  # mm
r_max = 70.0 # mm
turns = 3.0
theta_max = turns * 2 * math.pi
a = math.log(r_max / r_min) / theta_max

theta_0 = 10 / 180 * 2 * math.pi

# --- 2. Define PCB Origin (in millimeters) ---
center_x_mm = 0.0
center_y_mm = 0.0

vert_fov = 2*np.pi/3
horiz_fov = 2*np.pi/3

vert_pix = 40
horiz_pix = 40

sample_rate = 48000
total_samples = 1024
v = 343e3

def cartesian_product(*arrays):
    la = len(arrays)
    dtype = np.result_type(*arrays)
    arr = np.empty([len(a) for a in arrays] + [la], dtype=dtype)
    for i, a in enumerate(np.ix_(*arrays)):
        arr[...,i] = a
    return arr.reshape(-1, la)

def get_phase(row, theta_x, theta_y):
    k = np.array([np.tan(theta_x), np.tan(theta_y), 1.0])
    k /= np.linalg.norm(k)
    tau = -(row[0]*k[0] + row[1]*k[1]) / v
    sample_delay = tau * sample_rate
    rel_phase = -1j * 2 * np.pi * sample_delay / total_samples
    return rel_phase

def update_microphone_positions():

    pos = []

    for i in range(num_mics):
            
        # Calculate mathematically exact coordinates
        theta = (i / (num_mics - 1)) * theta_max + theta_0
        r = r_min * math.exp(a * theta)
        
        x_mm = r * math.cos(theta)
        y_mm = r * math.sin(theta)
        
        pos.append((x_mm, y_mm))

    pix_x = np.linspace(-horiz_fov / 2, horiz_fov / 2, horiz_pix)
    pix_y = np.linspace(-vert_fov / 2, vert_fov / 2, vert_pix)
    
    locs = cartesian_product(pix_x, pix_y)
    phases = []
    for l in locs:
        phases.append([])
        for p in pos:
            phases[-1].append(get_phase(p, l[0], l[1]))
    
    return phases

if __name__ == "__main__":
    print(update_microphone_positions()[1])
    