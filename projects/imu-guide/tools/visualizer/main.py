import math 
import pygame 
import serial 
import json

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
WIDTH, HEIGHT = 1220, 600 
FPS = 60 
GYRO_SENSITIVITY = CONFIG["GYRO_SENSITIVITY"]  
DT = 0.02                
ALPHA = 0.98             

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1) 
except Exception as e: 
    print(f'Error: {e}') 
    exit() 

# === [ Right-Hand Rule Geometry ] ===
l, w, h = 3.0, 2.0, 0.5 
vertices = [
    [-l, -w, -h], [l, -w, -h], [l, w, -h], [-l, w, -h], 
    [-l, -w,  h], [l, -w,  h], [l, w,  h], [-l, w,  h]  
]
edges = [(0,1), (1,2), (2,3), (3,0), (4,5), (5,6), (6,7), (7,4), (0,4), (1,5), (2,6), (3,7)] 

def get_tilt(ax, ay, az):
    roll = math.atan2(ay, az) * (180.0 / math.pi) 
    pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az)) * (180.0 / math.pi)
    return pitch, roll

def rotate_3d(x, y, z, pitch, roll, yaw):
    p, r, yw = math.radians(pitch), math.radians(roll), math.radians(yaw)
    
    cr, sr = math.cos(r), math.sin(r)
    y1 = y * cr - z * sr
    z1 = y * sr + z * cr
    x1 = x

    cp, sp = math.cos(p), math.sin(p)
    x2 = x1 * cp + z1 * sp
    z2 = -x1 * sp + z1 * cp
    y2 = y1

    cy, sy = math.cos(yw), math.sin(yw)
    x3 = x2 * cy - y2 * sy
    y3 = x2 * sy + y2 * cy
    z3 = z2
    
    return x3, y3, z3

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
        self.is_paused = False
        self.btn_rect = pygame.Rect(center_x - 70, HEIGHT - 70, 140, 40)

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
                pygame.quit()
                exit()
                
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
            except Exception:
                pass
        
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

# Initialize the 3 viewports
tracks = {
    "ACCEL": PanelTrack("ACCELEROMETER", 200),
    "GYRO": PanelTrack("GYROSCOPE", 600),
    "FUSED": PanelTrack("COMPLEMENTARY", 1000)
}

pygame.init() 
screen = pygame.display.set_mode((WIDTH, HEIGHT)) 
pygame.display.set_caption("3-Panel Sensor Fusion Analysis")
font = pygame.font.SysFont('Consolas', 18, bold=True)
small_font = pygame.font.SysFont('Consolas', 14)
clock = pygame.time.Clock() 

CONFIG["GYRO_BIAS"] = calibrate_gyro(screen, font, ser, samples=100)

# === [ Theme & UI State ] ===
is_light_mode = False
theme_btn_rect = pygame.Rect(10, 10, 120, 30)

running = True 
while running:
    # 1. Dynamic Theme Assignment
    colors = {
        "bg": (240, 240, 245) if is_light_mode else (20, 20, 30),
        "text": (30, 30, 30) if is_light_mode else (200, 200, 200),
        "angles": (0, 100, 100) if is_light_mode else (150, 255, 255),
        "cube": (0, 150, 80) if is_light_mode else (0, 255, 150),
        "dividers": (200, 200, 200) if is_light_mode else (50, 50, 60),
        "theme_btn": (200, 200, 210) if is_light_mode else (60, 60, 70),
        "theme_txt": (0, 0, 0) if is_light_mode else (255, 255, 255)
    }
    
    screen.fill(colors["bg"]) 

    # --- Event Handling (Buttons) ---
    for event in pygame.event.get(): 
        if event.type == pygame.QUIT: 
            running = False 
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:
                if theme_btn_rect.collidepoint(event.pos):
                    is_light_mode = not is_light_mode
                for key, track in tracks.items():
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
                    
                    ax, ay, az = cal_ax, cal_ay, cal_az
                    gx, gy, gz = cal_gx, cal_gy, cal_gz
                    
                    gx_dps = gx / GYRO_SENSITIVITY
                    gy_dps = gy / GYRO_SENSITIVITY
                    gz_dps = gz / GYRO_SENSITIVITY

                    if not tracks["ACCEL"].is_paused:
                        tracks["ACCEL"].pitch, tracks["ACCEL"].roll = get_tilt(ax, ay, az)

                    if not tracks["GYRO"].is_paused:
                        tracks["GYRO"].roll += gx_dps * DT
                        tracks["GYRO"].pitch += gy_dps * DT
                        tracks["GYRO"].yaw += gz_dps * DT

                    if not tracks["FUSED"].is_paused:
                        f_pitch, f_roll = get_tilt(ax, ay, az)
                        tracks["FUSED"].roll = (ALPHA * (tracks["FUSED"].roll + gx_dps * DT)) + ((1.0 - ALPHA) * f_roll)
                        tracks["FUSED"].pitch = (ALPHA * (tracks["FUSED"].pitch + gy_dps * DT)) + ((1.0 - ALPHA) * f_pitch)
                        tracks["FUSED"].yaw += gz_dps * DT
        except Exception: 
            pass
            
    # --- Rendering Engine ---
    for track in tracks.values():
        cx = track.center_x
        
        title = font.render(track.name, True, colors["text"])
        screen.blit(title, (cx - title.get_width()//2, 20))
        
        ang_str = f"P: {track.pitch:5.1f} | R: {track.roll:5.1f} | Y: {track.yaw:5.1f}"
        angles = small_font.render(ang_str, True, colors["angles"])
        screen.blit(angles, (cx - angles.get_width()//2, 45))

        proj_verts = [project(*rotate_3d(*v, track.pitch, track.roll, track.yaw), cx) for v in vertices]
        for edge in edges: 
            pygame.draw.line(screen, colors["cube"], proj_verts[edge[0]], proj_verts[edge[1]], 2) 

        origin = project(*rotate_3d(0, 0, 0, track.pitch, track.roll, track.yaw), cx)
        px = project(*rotate_3d(5, 0, 0, track.pitch, track.roll, track.yaw), cx)
        py = project(*rotate_3d(0, 5, 0, track.pitch, track.roll, track.yaw), cx)
        pz = project(*rotate_3d(0, 0, 5, track.pitch, track.roll, track.yaw), cx)
        
        pygame.draw.line(screen, (255, 50, 50), origin, px, 3)
        pygame.draw.line(screen, (50, 200, 50), origin, py, 3)
        pygame.draw.line(screen, (50, 100, 255), origin, pz, 3)

        front_pos = project(*rotate_3d(l, 0, 0, track.pitch, track.roll, track.yaw), cx)
        top_pos = project(*rotate_3d(0, 0, h, track.pitch, track.roll, track.yaw), cx)
        
        screen.blit(small_font.render("F", True, (200, 200, 0) if is_light_mode else (255, 255, 0)), front_pos)
        screen.blit(small_font.render("T", True, (200, 0, 200) if is_light_mode else (255, 0, 255)), top_pos)

        track.draw_button(screen, font)

    # Render Theme Toggle Button
    pygame.draw.rect(screen, colors["theme_btn"], theme_btn_rect, border_radius=5)
    theme_label = small_font.render("Toggle Mode", True, colors["theme_txt"])
    screen.blit(theme_label, (theme_btn_rect.centerx - theme_label.get_width()//2, theme_btn_rect.centery - theme_label.get_height()//2))

    # Panel Dividers
    pygame.draw.line(screen, colors["dividers"], (400, 0), (400, HEIGHT), 2)
    pygame.draw.line(screen, colors["dividers"], (800, 0), (800, HEIGHT), 2)

    pygame.display.flip() 
    clock.tick(FPS) 

ser.close() 
pygame.quit()