import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

# =========================
# CONFIG
# =========================
PORT = "COM5"          
BAUD = 115200
MAX_POINTS = 500      

# =========================
# SERIAL SETUP
# =========================
ser = serial.Serial(PORT, BAUD, timeout=1)

# Data buffers
t_data = deque(maxlen=MAX_POINTS)
angle_deg_data = deque(maxlen=MAX_POINTS)
angle_rad_data = deque(maxlen=MAX_POINTS)
vel_data = deque(maxlen=MAX_POINTS)

# Skip lines until header found
print("Waiting for CSV header...")
while True:
    line = ser.readline().decode(errors="ignore").strip()
    if line == "time_s,angle_deg,angle_rad,velocity_rad_s":
        print("Header found. Starting plot...")
        break

# =========================
# PLOT SETUP
# =========================
fig, ax = plt.subplots()
line_angle_deg, = ax.plot([], [], label="Angle (deg)")
line_vel, = ax.plot([], [], label="Velocity (rad/s)")

ax.set_xlabel("Time (s)")
ax.set_ylabel("Value")
ax.set_title("Live Encoder Data")
ax.legend()
ax.grid(True)

def update(frame):
    while ser.in_waiting:
        line = ser.readline().decode(errors="ignore").strip()

        # Ignore empty or malformed lines
        if not line or "," not in line:
            continue

        parts = line.split(",")
        if len(parts) != 4:
            continue

        try:
            t = float(parts[0])
            angle_deg = float(parts[1])
            angle_rad = float(parts[2])
            vel = float(parts[3])
        except ValueError:
            continue

        t_data.append(t)
        angle_deg_data.append(angle_deg)
        angle_rad_data.append(angle_rad)
        vel_data.append(vel)

    if len(t_data) == 0:
        return line_angle_deg, line_vel

    line_angle_deg.set_data(t_data, angle_deg_data)
    line_vel.set_data(t_data, vel_data)

    ax.set_xlim(min(t_data), max(t_data))

    all_y = list(angle_deg_data) + list(vel_data)
    y_min = min(all_y)
    y_max = max(all_y)

    if y_min == y_max:
        y_min -= 1
        y_max += 1

    margin = 0.1 * (y_max - y_min)
    ax.set_ylim(y_min - margin, y_max + margin)

    return line_angle_deg, line_vel

ani = FuncAnimation(fig, update, interval=50, blit=False)

try:
    plt.show()
finally:
    ser.close()