"""
====================================================================
Architecture: Python Implementation of HAL, Middleware, Application
====================================================================
"""
import math 
import pygame 
import serial 

# === [System_Config] ===
SERIAL_PORT = 'COM7' 
BAUD_RATE = 115200 
WIDTH, HEIGHT = 800, 600 
FPS = 60 

# Kinematic Constants
GYRO_SENSITIVITY = 131.0  # LSB/(°/s) for ±250 dps range
DT = 0.03                 # 30ms integration time matching Arduino telemetry

# === [HAL: Serial Telemetry Interface] ===
print(f"Attempting to connect to {SERIAL_PORT}...") 
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1) 
    print('Connected successfully!') 
except Exception as e: 
    print(f'Error opening serial port: {e}') 
    exit() 

# === [Middleware: Hardware Geometry & Kinematics] ===
w, l, h = 1.5, 2.0, 0.1 
vertices = [
    [-w, -h, -l], [w, -h, -l], [w, h, -l], [-w, h, -l],
    [-w, -h, l], [w, -h, l], [w, h, l], [-w, h, l]
]

edges = [(0,1), (1,2), (2,3), (3,0), (4,5), (5,6), (6,7), (7,4), (0,4), (1,5), (2,6), (3,7)] 

def calculate_pitch_roll(ax, ay, az):
    """Accelerometer-based absolute tilt referencing Earth's gravity"""
    roll = math.atan2(ay, az) * (180.0 / math.pi) 
    pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az)) * (180.0 / math.pi)
    return pitch, roll

def integrate_yaw(current_yaw, gz, dt):
    """Gyroscope-based discrete time integration for heading"""
    # Convert raw integers to degrees per second, then multiply by time
    yaw_rate = gz / GYRO_SENSITIVITY
    new_yaw = current_yaw + (yaw_rate * dt)
    return new_yaw

def rotate_3d(x, y, z, pitch_angle, roll_angle, yaw_angle):
    """3D Euler Rotation Matrix (Pitch -> Yaw -> Roll) mapped to Pygame's coordinates"""
    # 1. Apply Pitch (Rotation around Pygame's X-axis)
    rad_p = math.radians(pitch_angle)
    cos_p, sin_p = math.cos(rad_p), math.sin(rad_p)
    y1 = y * cos_p - z * sin_p
    z1 = y * sin_p + z * cos_p
    x1 = x

    # 2. Apply Yaw (Rotation around Pygame's Y-axis)
    rad_y = math.radians(yaw_angle)
    cos_y, sin_y = math.cos(rad_y), math.sin(rad_y)
    x2 = x1 * cos_y + z1 * sin_y
    z2 = -x1 * sin_y + z1 * cos_y
    y2 = y1

    # 3. Apply Roll (Rotation around Pygame's Z-axis)
    rad_r = math.radians(roll_angle)
    cos_r, sin_r = math.cos(rad_r), math.sin(rad_r)
    x3 = x2 * cos_r - y2 * sin_r
    y3 = x2 * sin_r + y2 * cos_r
    
    return x3, y3, z2

# === [Application: User Interface & Main Loop] ===
pygame.init() 
screen = pygame.display.set_mode((WIDTH, HEIGHT)) 
pygame.display.set_caption('GY-91 Diagnostic 3D Visualizer') 
clock = pygame.time.Clock() 

current_pitch, current_roll, current_yaw = 0.0, 0.0, 0.0 
running = True 

print('Visualizer running. Watch your terminal for incoming data logs below:') 

while running: 
    screen.fill((20, 20, 30)) 

    for event in pygame.event.get(): 
        if event.type == pygame.QUIT: 
            running = False 

    # 1. HAL READ: Fetch serial string from MCU
    if ser.in_waiting > 0: 
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip() 
            if line: 
                print(f'Received -> {line}') 
                
                # 2. MIDDLEWARE PARSE
                parts = line.split(',') 
                if len(parts) == 6: 
                    raw_ax = float(parts[0])
                    raw_ay = float(parts[1])
                    raw_az = float(parts[2])
                    raw_gx = float(parts[3])
                    raw_gy = float(parts[4])
                    raw_gz = float(parts[5])
                  
                    # --- SENSOR TO WORLD FRAME REMAPPING ---
                    ax = raw_ax       
                    ay = -raw_ay      
                    az = raw_az        
                    
                    # Depending on sensor orientation, you may need to invert raw_gz
                    # to match the visual Yaw rotation direction.
                    gz = -raw_gz 
                  
                    # 3. MIDDLEWARE MATH: Update global kinematics
                    current_pitch, current_roll = calculate_pitch_roll(ax, ay, az)
                    current_yaw = integrate_yaw(current_yaw, gz, DT)
                    
        except Exception as e: 
            print(f'Parsing Error: {e}') 

    # 4. APPLICATION RENDER: Draw the 3D Object
    projected_points = [] 
    for vertex in vertices: 
        x, y, z = rotate_3d(vertex[0], vertex[1], vertex[2], current_pitch, current_roll, current_yaw)
        
        fov = 500 
        distance = 5.0  # Pushed back slightly to ensure the rotating edges stay on screen
        factor = fov / (z + distance) 
        x_proj = int(x * factor + WIDTH / 2) 
        y_proj = int(y * factor + HEIGHT / 2) 
        projected_points.append((x_proj, y_proj)) 

    for edge in edges: 
        p1 = projected_points[edge[0]] 
        p2 = projected_points[edge[1]] 
        pygame.draw.line(screen, (0, 255, 150), p1, p2, 3) 

    pygame.display.flip() 
    clock.tick(FPS) 

ser.close() 
pygame.quit()