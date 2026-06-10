import math
from kipy import KiCad
from kipy.geometry import Vector2

# --- 1. Define Spiral Parameters ---
num_mics = 16
r_min = 12.0  # mm
r_max = 70.0 # mm
turns = 3.0
theta_max = turns * 2 * math.pi
a = math.log(r_max / r_min) / theta_max

theta_0 = 10 / 180 * 2 * math.pi

# --- 2. Define PCB Origin (in millimeters) ---
center_x_mm = 150.0 
center_y_mm = 100.0

def update_microphone_positions():
    # Connect to the running KiCad instance via IPC
    client = KiCad()
    board = client.get_board()
    
    # Retrieve all footprints currently on the board
    footprints = board.get_footprints()
    
    # Map them by reference designator for quick O(1) lookup
    fp_dict = {fp.reference_field.text.value: fp for fp in footprints}
    
    # Keep track of the footprints we actually modify
    updated_footprints = []

    for i in range(num_mics):
        refdes = f"M{i+1}"
        
        if refdes in fp_dict:
            fp = fp_dict[refdes]
            
            # Calculate mathematically exact coordinates
            theta = (i / (num_mics - 1)) * theta_max + theta_0
            r = r_min * math.exp(a * theta)
            
            x_mm = r * math.cos(theta)
            y_mm = r * math.sin(theta)
            
            target_x = center_x_mm + x_mm
            target_y = center_y_mm - y_mm  # KiCad Y-axis points down
            
            # FIX: Use the explicit factory method for millimeter coordinates
            fp.position = Vector2.from_xy_mm(target_x, target_y)
            updated_footprints.append(fp)
            
            print(f"Calculated {refdes} at X: {target_x:.2f} mm, Y: {target_y:.2f} mm")
        else:
            print(f"⚠️ Warning: Could not find footprint {refdes}")

    # Push the mutated footprints back to the KiCad server via IPC
    if updated_footprints:
        board.update_items(updated_footprints)
        print("\nSuccessfully updated KiCad layout via IPC.")

if __name__ == "__main__":
    update_microphone_positions()