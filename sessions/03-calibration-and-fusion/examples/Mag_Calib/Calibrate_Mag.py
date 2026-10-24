import serial
import time
import numpy as np
import matplotlib.pyplot as plt

# --- CONFIGURATION ---
COM_PORT = 'COM7'  # Update to your Arduino port
BAUD_RATE = 115200
CALIBRATION_TIME = 200 
# ---------------------

def plot_final_comparison(raw, cal):
    fig = plt.figure(figsize=(14, 6))
    for idx, data, title in zip([121, 122], [raw, cal], ["Raw Ellipsoid", "Calibrated Sphere"]):
        ax = fig.add_subplot(idx, projection='3d')
        x, y, z = data[:, 0], data[:, 1], data[:, 2]
        ax.scatter(x, y, z, c='k', alpha=0.15, s=10)
        ax.scatter([0], [0], [0], c='y', s=100, zorder=5)
        
        limit = np.max(np.abs(data)) + 50
        min_val = -limit
        ax.scatter(x, y, np.full_like(z, min_val), c='r', alpha=0.1, s=5) 
        ax.scatter(x, np.full_like(y, min_val), z, c='g', alpha=0.1, s=5) 
        ax.scatter(np.full_like(x, min_val), y, z, c='b', alpha=0.1, s=5) 

        ax.set_title(title)
        ax.set_xlim([-limit, limit]); ax.set_ylim([-limit, limit]); ax.set_zlim([-limit, limit])
        ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    plt.tight_layout()
    plt.show()

ser = serial.Serial(COM_PORT, BAUD_RATE)
x_data, y_data, z_data = [], [], []

plt.ion()
live_fig = plt.figure(figsize=(8, 8))
live_ax = live_fig.add_subplot(111, projection='3d')
live_ax.set_title("Live 3D View - Fill the sphere!")
live_ax.set_xlim([-600, 600]); live_ax.set_ylim([-600, 600]); live_ax.set_zlim([-600, 600])
scatter_3d = live_ax.scatter([], [], [], c='blue', alpha=0.4, s=15)

print(f"Starting LIVE Calibration. Tumble the sensor in figure-8 patterns for {CALIBRATION_TIME}s.")
start_time = time.time()

try:
    while time.time() - start_time < CALIBRATION_TIME:
        if ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            vals = line.split(',')
            if len(vals) == 3:
                try:
                    x_data.append(float(vals[0]))
                    y_data.append(float(vals[1]))
                    z_data.append(float(vals[2]))
                    if len(x_data) % 15 == 0:
                        scatter_3d._offsets3d = (x_data, y_data, z_data)
                        live_ax.set_title(f"Time Left: {int(CALIBRATION_TIME - (time.time() - start_time))}s")
                        live_fig.canvas.draw_idle()
                        live_fig.canvas.flush_events()
                except ValueError:
                    pass
except KeyboardInterrupt:
    print("Collection stopped early.")

ser.close()
plt.ioff()
plt.close(live_fig)

raw_data = np.column_stack((x_data, y_data, z_data))
print(f"Collected {len(raw_data)} samples. Computing transformation matrices...")

# Hard Iron Offset
min_x, max_x = np.min(raw_data[:,0]), np.max(raw_data[:,0])
min_y, max_y = np.min(raw_data[:,1]), np.max(raw_data[:,1])
min_z, max_z = np.min(raw_data[:,2]), np.max(raw_data[:,2])

offset_x = (max_x + min_x) / 2.0
offset_y = (max_y + min_y) / 2.0
offset_z = (max_z + min_z) / 2.0

# Soft Iron Scale
avg_delta = ((max_x - min_x) + (max_y - min_y) + (max_z - min_z)) / 3.0
scale_x = avg_delta / (max_x - min_x) if (max_x - min_x) != 0 else 1
scale_y = avg_delta / (max_y - min_y) if (max_y - min_y) != 0 else 1
scale_z = avg_delta / (max_z - min_z) if (max_z - min_z) != 0 else 1

soft_iron_matrix = np.array([[scale_x, 0, 0], [0, scale_y, 0], [0, 0, scale_z]])
calibrated_data = np.array([np.dot(soft_iron_matrix, (pt - [offset_x, offset_y, offset_z])) for pt in raw_data])

print("\n--- COPY THESE CONSTANTS INTO YOUR PRODUCTION ARDUINO CODE ---")
print(f"const float MAG_BIAS[3]  = {{{offset_x:.2f}, {offset_y:.2f}, {offset_z:.2f}}};")
print(f"const float MAG_SCALE[3] = {{{scale_x:.4f}, {scale_y:.4f}, {scale_z:.4f}}};")
print("-------------------------------------------------------------")

plot_final_comparison(raw_data, calibrated_data)