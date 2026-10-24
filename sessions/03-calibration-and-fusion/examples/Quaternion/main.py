import math 
import pygame 
import serial 
import json
import numpy as np

# === [ Configuration Loading ] ===
def load_config(filename="config.json"):
    try:
        with open(filename, 'r') as f:
            return json.load(f)
    except Exception as e:
        print(f"Error loading config: {e}. Using defaults.")
        return {
            "COM_PORT": "COM7", "BAUD_RATE": 115200,
            "ACCEL_SENSITIVITY": 8192.0, "GYRO_SENSITIVITY": 65.5,
            "ACCEL_BIAS": [0.0, 0.0, 0.0], 
            "ACCEL_SCALE": [1.0, 1.0, 1.0],
            "GYRO_BIAS": [0.0, 0.0, 0.0] 
        }

CONFIG = load_config()

# === [ System_Config ] ===
SERIAL_PORT = CONFIG["COM_PORT"]
BAUD_RATE = CONFIG["BAUD_RATE"]
WIDTH, HEIGHT = 1600, 600  # 4-panel width layout
FPS = 60 
GYRO_SENSITIVITY = CONFIG["GYRO_SENSITIVITY"]  
DT = 0.02                
ALPHA = 0.02 # Accelerometer trust factor for quaternion fusion

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1) 
except Exception as e: 
    print(f'Error: {e}') 
    exit() 

# === [ Mathematics Library ] ===
def quat_mult(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

def rotate_vector_by_quat(v, q):
    q_conj = np.array([q[0], -q[1], -q[2], -q[3]])
    v_quat = np.array([0.0, v[0], v[1], v[2]])
    res = quat_mult(quat_mult(q, v_quat), q_conj)
    return res[1:4]

def quat_to_euler(q):
    w, x, y, z = q
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
    
    sinp = 2 * (w * y - z * x)
    pitch = math.degrees(math.copysign(math.pi / 2, sinp)) if abs(sinp) >= 1 else math.degrees(math.asin(sinp))
    
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    
    return pitch, roll, yaw

def euler_to_quat(roll, pitch, yaw):
    cr = math.cos(math.radians(roll) * 0.5)
    sr = math.sin(math.radians(roll) * 0.5)
    cp = math.cos(math.radians(pitch) * 0.5)
    sp = math.sin(math.radians(pitch) * 0.5)
    cy = math.cos(math.radians(yaw) * 0.5)
    sy = math.sin(math.radians(yaw) * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    q = np.array([qw, qx, qy, qz])
    return q / np.linalg.norm(q)

def get_tilt(ax, ay, az):
    roll = math.atan2(ay, az) * (180.0 / math.pi) 
    pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az)) * (180.0 / math.pi)
    return pitch, roll

# === [ 3D Rendering Geometry ] ===
l, w, h = 3.0, 2.0, 0.5 
vertices = [
    [-l, -w, -h], [l, -w, -h], [l, w, -h], [-l, w, -h], 
    [-l, -w,  h], [l, -w,  h], [l, w,  h], [-l, w,  h]  
]
edges = [(0,1), (1,2), (2,3), (3,0), (4,5), (5,6), (6,7), (7,4), (0,4), (1,5), (2,6), (3,7)] 

def project(x, y, z, center_x):
    distance = 8.0 
    factor = 450 / (x + distance) if (x + distance) != 0 else 0.1
    px = int(center_x - (y * factor))
    py = int(HEIGHT / 2 - (z * factor) - 30)
    return px, py

# === [ Track State Management ] ===
class PanelTrack:
    def __init__(self, name, center_x):
        self.name = name
        self.center_x = center_x
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.is_paused = False
        self.btn_rect = pygame.Rect(center_x - 60, HEIGHT - 70, 120, 35)

    def draw_button(self, surface, font):
        color = (200, 50, 50) if self.is_paused else (50, 200, 50)
        text = "UNFREEZE" if self.is_paused else "FREEZE"
        pygame.draw.rect(surface, color, self.btn_rect, border_radius=5)
        label = font.render(text, True, (255,255,255))
        surface.blit(label, (self.btn_rect.centerx - label.get_width()//2, self.btn_rect.centery - label.get_height()//2))

def calibrate_gyro(screen, font, ser, samples=100):
    screen.fill((20, 20, 30))
    warn_title = font.render("HANDS OFF DESK! SETTLING SENSOR...", True, (255, 50, 50))
    screen.blit(warn_title, (WIDTH//2 - warn_title.get_width()//2, HEIGHT//2))
    pygame.display.flip()
    pygame.time.wait(2000) 
    
    gx_sum, gy_sum, gz_sum = 0.0, 0.0, 0.0
    count = 0
    clock = pygame.time.Clock()
    ser.reset_input_buffer() 
    
    while count < samples:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit(); exit()
                
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    parts = line.split(',')
                    if len(parts) == 6:
                        gx_sum += float(parts[3])
                        gy_sum += float(parts[4])
                        gz_sum += float(parts[5])
                        count += 1
            except Exception: pass
        
        screen.fill((20, 20, 30))
        title = font.render("CALIBRATING GYROSCOPE - KEEP SENSOR STILL", True, (255, 255, 255))
        screen.blit(title, (WIDTH//2 - title.get_width()//2, HEIGHT//2 - 60))
        
        progress = int((count / samples) * 100)
        pct_text = font.render(f"{progress}%", True, (0, 255, 150))
        screen.blit(pct_text, (WIDTH//2 - pct_text.get_width()//2, HEIGHT//2 - 20))
        
        pygame.draw.rect(screen, (50, 50, 60), (WIDTH//2 - 150, HEIGHT//2 + 20, 300, 30), border_radius=5)
        pygame.draw.rect(screen, (0, 255, 150), (WIDTH//2 - 150, HEIGHT//2 + 20, 3 * progress, 30), border_radius=5)
        
        pygame.display.flip()
        clock.tick(FPS)
        
    return [gx_sum / samples, gy_sum / samples, gz_sum / samples]

# Initialize 4 Viewports
tracks = {
    "ACCEL": PanelTrack("1. ACCEL (EULER)", 200),
    "GYRO_EULER": PanelTrack("2. GYRO (EULER MATRIX)", 600),
    "GYRO_QUAT": PanelTrack("3. GYRO (QUATERNION)", 1000),
    "FUSED": PanelTrack("4. QUAT FUSION", 1400)
}

pygame.init() 
screen = pygame.display.set_mode((WIDTH, HEIGHT)) 
pygame.display.set_caption("4-Panel Attitude & Gimbal Lock Demonstration")
font = pygame.font.SysFont('Consolas', 16, bold=True)
small_font = pygame.font.SysFont('Consolas', 12)
clock = pygame.time.Clock() 

CONFIG["GYRO_BIAS"] = calibrate_gyro(screen, font, ser, samples=100)

running = True 
while running:
    screen.fill((20, 20, 30)) 

    for event in pygame.event.get(): 
        if event.type == pygame.QUIT: 
            running = False 
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:
                for track in tracks.values():
                    if track.btn_rect.collidepoint(event.pos):
                        track.is_paused = not track.is_paused

    # --- Telemetry Processing ---
    while ser.in_waiting > 0: 
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip() 
            if line: 
                parts = line.split(',') 
                if len(parts) == 6: 
                    raw_ax, raw_ay, raw_az = float(parts[0]), float(parts[1]), float(parts[2])
                    raw_gx, raw_gy, raw_gz = float(parts[3]), float(parts[4]), float(parts[5])
                    
                    cal_ax = (raw_ax - CONFIG["ACCEL_BIAS"][0]) * CONFIG["ACCEL_SCALE"][0]
                    cal_ay = (raw_ay - CONFIG["ACCEL_BIAS"][1]) * CONFIG["ACCEL_SCALE"][1]
                    cal_az = (raw_az - CONFIG["ACCEL_BIAS"][2]) * CONFIG["ACCEL_SCALE"][2]

                    cal_gx = raw_gx - CONFIG["GYRO_BIAS"][0]
                    cal_gy = raw_gy - CONFIG["GYRO_BIAS"][1]
                    cal_gz = raw_gz - CONFIG["GYRO_BIAS"][2]
                    
                    ax, ay, az = -cal_ax, -cal_ay, cal_az
                    gx_dps = cal_gx / GYRO_SENSITIVITY
                    gy_dps = cal_gy / GYRO_SENSITIVITY
                    gz_dps = cal_gz / GYRO_SENSITIVITY

                    # Panel 1: Pure Accelerometer (Euler)
                    if not tracks["ACCEL"].is_paused:
                        tracks["ACCEL"].pitch, tracks["ACCEL"].roll = get_tilt(ax, ay, az)
                        tracks["ACCEL"].yaw = 0.0
                        tracks["ACCEL"].q = euler_to_quat(tracks["ACCEL"].roll, tracks["ACCEL"].pitch, tracks["ACCEL"].yaw)

                    # Panel 2: Gyroscope with Euler Kinematic Matrix (Shows Gimbal Lock)
                    if not tracks["GYRO_EULER"].is_paused:
                        tr = tracks["GYRO_EULER"]
                        phi = math.radians(tr.roll)
                        theta = math.radians(tr.pitch)
                        p, q, r = math.radians(gx_dps), math.radians(gy_dps), math.radians(gz_dps)
                        
                        cos_theta = math.cos(theta)
                        tan_theta = math.tan(theta) if abs(cos_theta) > 1e-6 else 1e6

                        roll_dot = p + (math.sin(phi) * tan_theta * q) + (math.cos(phi) * tan_theta * r)
                        pitch_dot = (math.cos(phi) * q) - (math.sin(phi) * r)
                        yaw_dot = ((math.sin(phi) / cos_theta) * q) + ((math.cos(phi) / cos_theta) * r)

                        tr.roll += math.degrees(roll_dot) * DT
                        tr.pitch += math.degrees(pitch_dot) * DT
                        tr.yaw += math.degrees(yaw_dot) * DT
                        tr.q = euler_to_quat(tr.roll, tr.pitch, tr.yaw)

                    # Panel 3: Pure Gyroscope (Quaternion Integration)
                    if not tracks["GYRO_QUAT"].is_paused:
                        tr = tracks["GYRO_QUAT"]
                        p_rad = math.radians(gx_dps)
                        q_rad = math.radians(gy_dps)
                        r_rad = math.radians(gz_dps)
                        q_dot = 0.5 * quat_mult(tr.q, [0, p_rad, q_rad, r_rad])
                        q_new = tr.q + (q_dot * DT)
                        tr.q = q_new / np.linalg.norm(q_new)
                        tr.pitch, tr.roll, tr.yaw = quat_to_euler(tr.q)

                    # Panel 4: Quaternion Complementary Fusion Filter
                    if not tracks["FUSED"].is_paused:
                        tr = tracks["FUSED"]
                        p_rad = math.radians(gx_dps)
                        q_rad = math.radians(gy_dps)
                        r_rad = math.radians(gz_dps)
                        
                        q_dot_f = 0.5 * quat_mult(tr.q, [0, p_rad, q_rad, r_rad])
                        q_pred = tr.q + (q_dot_f * DT)
                        q_pred = q_pred / np.linalg.norm(q_pred)

                        a_norm = np.linalg.norm([ax, ay, az])
                        if a_norm > 0:
                            a_vec = np.array([ax, ay, az]) / a_norm
                            g_est = rotate_vector_by_quat([0, 0, 1], [q_pred[0], -q_pred[1], -q_pred[2], -q_pred[3]])
                            error = np.cross(a_vec, g_est)
                            
                            q_corr = np.array([1.0, ALPHA * error[0], ALPHA * error[1], ALPHA * error[2]])
                            q_corr = q_corr / np.linalg.norm(q_corr)
                            
                            tr.q = quat_mult(q_pred, q_corr)
                            tr.q /= np.linalg.norm(tr.q)
                        else:
                            tr.q = q_pred
                        tr.pitch, tr.roll, tr.yaw = quat_to_euler(tr.q)

        except Exception: 
            pass
            
    # --- Rendering Engine ---
    for track in tracks.values():
        cx = track.center_x
        title = font.render(track.name, True, (200, 200, 200))
        screen.blit(title, (cx - title.get_width()//2, 20))
        
        ang_str = f"P:{track.pitch:5.1f} R:{track.roll:5.1f} Y:{track.yaw:5.1f}"
        angles = small_font.render(ang_str, True, (150, 255, 255))
        screen.blit(angles, (cx - angles.get_width()//2, 45))

        proj_verts = [project(*rotate_vector_by_quat(v, track.q), cx) for v in vertices]
        for edge in edges: 
            pygame.draw.line(screen, (0, 255, 150), proj_verts[edge[0]], proj_verts[edge[1]], 2) 

        origin = project(*rotate_vector_by_quat([0,0,0], track.q), cx)
        px = project(*rotate_vector_by_quat([5,0,0], track.q), cx)
        py = project(*rotate_vector_by_quat([0,5,0], track.q), cx)
        pz = project(*rotate_vector_by_quat([0,0,5], track.q), cx)
        
        pygame.draw.line(screen, (255, 50, 50), origin, px, 3)
        pygame.draw.line(screen, (50, 200, 50), origin, py, 3)
        pygame.draw.line(screen, (50, 100, 255), origin, pz, 3)

        track.draw_button(screen, font)

    # Panel Dividers
    for x_pos in [400, 800, 1200]:
        pygame.draw.line(screen, (50, 50, 60), (x_pos, 0), (x_pos, HEIGHT), 2)

    pygame.display.flip() 
    clock.tick(FPS) 

ser.close() 
pygame.quit()