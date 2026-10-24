import math 
import pygame 
import serial 

# === [ System Configuration ] ===
SERIAL_PORT = "COM7"  # Update to match your active port
BAUD_RATE = 115200
WIDTH, HEIGHT = 800, 600 
FPS = 60 

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1) 
except Exception as e: 
    print(f"Error opening port {SERIAL_PORT}: {e}") 
    exit() 

# === [ Right-Hand Rule Geometry ] ===
l, w, h = 3.0, 2.0, 0.5
vertices = [
    [-l, -w, -h], [l, -w, -h], [l, w, -h], [-l, w, -h], 
    [-l, -w,  h], [l, -w,  h], [l, w,  h], [-l, w,  h]  
]
edges = [(0,1), (1,2), (2,3), (3,0), (4,5), (5,6), (6,7), (7,4), (0,4), (1,5), (2,6), (3,7)]

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

# === [ Pygame Initialization ] ===
pygame.init() 
screen = pygame.display.set_mode((WIDTH, HEIGHT)) 
pygame.display.set_caption("Madgwick 9-DoF Single Panel Visualizer")
font = pygame.font.SysFont('Consolas', 24, bold=True)
small_font = pygame.font.SysFont('Consolas', 18)
clock = pygame.time.Clock() 

roll, pitch, yaw = 0.0, 0.0, 0.0
cx = WIDTH // 2

running = True 
while running:
    screen.fill((20, 20, 30)) # Dark background
    
    # --- Event Handling ---
    for event in pygame.event.get(): 
        if event.type == pygame.QUIT: 
            running = False 

    # --- Telemetry Processing ---
    while ser.in_waiting > 0: 
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip() 
            if line: 
                parts = line.split(',') 
                # Expecting format: roll,pitch,yaw (e.g., "-0.17,-6.16,-13.96")
                if len(parts) == 3: 
                    roll = float(parts[0])
                    pitch = float(parts[1])
                    yaw = float(parts[2])
        except Exception: 
            pass # Ignore serial collisions during rotation
            
    # --- Rendering Engine ---
    title = font.render("MADGWICK QUATERNION FILTER", True, (200, 200, 200))
    screen.blit(title, (cx - title.get_width()//2, 20))
    
    ang_str = f"ROLL: {roll:5.1f}° | PITCH: {pitch:5.1f}° | YAW: {yaw:5.1f}°"
    angles = small_font.render(ang_str, True, (0, 255, 150))
    screen.blit(angles, (cx - angles.get_width()//2, 60))

    # Project and draw 3D cube edges
    proj_verts = [project(*rotate_3d(*v, pitch, roll, yaw), cx) for v in vertices]
    for edge in edges: 
        pygame.draw.line(screen, (0, 150, 255), proj_verts[edge[0]], proj_verts[edge[1]], 2) 

    # Draw internal axis lines
    origin = project(*rotate_3d(0, 0, 0, pitch, roll, yaw), cx)
    px = project(*rotate_3d(5, 0, 0, pitch, roll, yaw), cx)
    py = project(*rotate_3d(0, 5, 0, pitch, roll, yaw), cx)
    pz = project(*rotate_3d(0, 0, 5, pitch, roll, yaw), cx)
    
    pygame.draw.line(screen, (255, 50, 50), origin, px, 3)
    pygame.draw.line(screen, (50, 200, 50), origin, py, 3)
    pygame.draw.line(screen, (50, 100, 255), origin, pz, 3)

    # Draw F (Front) and T (Top) orientation markers
    front_pos = project(*rotate_3d(l, 0, 0, pitch, roll, yaw), cx)
    top_pos = project(*rotate_3d(0, 0, h, pitch, roll, yaw), cx)
    
    screen.blit(small_font.render("F", True, (255, 255, 0)), front_pos)
    screen.blit(small_font.render("T", True, (255, 0, 255)), top_pos)

    pygame.display.flip() 
    clock.tick(FPS) 

ser.close() 
pygame.quit()