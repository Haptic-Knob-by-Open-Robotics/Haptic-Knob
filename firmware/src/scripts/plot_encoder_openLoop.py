import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

PORT = "COM5"
BAUD = 115200
MAX_POINTS = 500

ser = serial.Serial(PORT, BAUD, timeout=1)

t_data = deque(maxlen=MAX_POINTS)
wrapped_deg_data = deque(maxlen=MAX_POINTS)
unwrapped_deg_data = deque(maxlen=MAX_POINTS)
vel_data = deque(maxlen=MAX_POINTS)
target_data = deque(maxlen=MAX_POINTS)

print("Waiting for CSV header...")
while True:
    line = ser.readline().decode(errors="ignore").strip()
    if line == "time_s,target_rad_s,wrapped_deg,wrapped_rad,unwrapped_deg,unwrapped_rad,velocity_rad_s":
        print("Header found. Starting plot...")
        break

fig, ax = plt.subplots()
line_unwrapped, = ax.plot([], [], label="Unwrapped angle (deg)")
line_wrapped, = ax.plot([], [], label="Wrapped angle (deg)")
line_vel, = ax.plot([], [], label="Velocity (rad/s)")

ax.set_xlabel("Time (s)")
ax.set_ylabel("Value")
ax.set_title("Live Encoder Data")
ax.legend()
ax.grid(True)

def update(frame):
    while ser.in_waiting:
        line = ser.readline().decode(errors="ignore").strip()

        if not line or "," not in line:
            continue

        parts = line.split(",")
        if len(parts) != 7:
            continue

        try:
            t = float(parts[0])
            target = float(parts[1])
            wrapped_deg = float(parts[2])
            wrapped_rad = float(parts[3])
            unwrapped_deg = float(parts[4])
            unwrapped_rad = float(parts[5])
            vel = float(parts[6])
        except ValueError:
            continue

        t_data.append(t)
        target_data.append(target)
        wrapped_deg_data.append(wrapped_deg)
        unwrapped_deg_data.append(unwrapped_deg)
        vel_data.append(vel)

    if len(t_data) == 0:
        return line_unwrapped, line_wrapped, line_vel

    line_unwrapped.set_data(t_data, unwrapped_deg_data)
    line_wrapped.set_data(t_data, wrapped_deg_data)
    line_vel.set_data(t_data, vel_data)

    ax.set_xlim(min(t_data), max(t_data))

    all_y = list(unwrapped_deg_data) + list(wrapped_deg_data) + list(vel_data)
    y_min = min(all_y)
    y_max = max(all_y)

    if y_min == y_max:
        y_min -= 1
        y_max += 1

    margin = 0.1 * (y_max - y_min)
    ax.set_ylim(y_min - margin, y_max + margin)

    return line_unwrapped, line_wrapped, line_vel

ani = FuncAnimation(fig, update, interval=50, blit=False)

try:
    plt.show()
finally:
    ser.close()