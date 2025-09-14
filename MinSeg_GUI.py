import serial
import time
import tkinter as tk
from tkinter import ttk
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

# Setup
BAUD = 115200
TIME_WINDOW = 5
port = input("Enter your USB port: ") # e.g. /dev/tty.usbmodem1101
ser = serial.Serial(port, BAUD, timeout=1)
time.sleep(1.5)

# Data containers
times = deque()
motor_values = deque()
angle_values = deque()
angvel_values = deque()
position_values = deque()
velocity_values = deque()
refpos_values = deque()

# Creating the window 
root = tk.Tk()
root.title("MinSeg GUI")
window_width = 1200
window_height = 800
screen_width = root.winfo_screenwidth()
screen_height = root.winfo_screenheight()
x = int((screen_width - window_width) / 2)
y = int((screen_height - window_height) / 2)
root.geometry(f"{window_width}x{window_height}+{x}+{y}")

# Creating the axes
fig, (motor_axis, angle_axis, position_axis) = plt.subplots(nrows=3, ncols=1, sharex=True, figsize=(12, 7))
angvel_axis = angle_axis.twinx()
velocity_axis = position_axis.twinx()

# Creating the lines
motor_line, = motor_axis.plot([], [], 'g-', lw=2, label='Motor voltage')
angle_line, = angle_axis.plot([], [], 'r-', lw=2, label='Angle')
angvel_line, = angvel_axis.plot([], [], 'm-', lw=2, label='Angular velocity')
position_line, = position_axis.plot([], [], 'b-', lw=2, label='Position')
velocity_line, = velocity_axis.plot([], [], 'c-', lw=2, label='Velocity')
refpos_line, = velocity_axis.plot([], [], 'k-', lw=1, label='Reference position')

# Labelling the axes
motor_axis.set_ylabel('(V)', rotation=0, labelpad=10)
angle_axis.set_ylabel('(rad)', rotation=0, labelpad=10)
angvel_axis.set_ylabel('(rad/s)', rotation=0, labelpad=20)
position_axis.set_ylabel('(m)', rotation=0, labelpad=10)
velocity_axis.set_ylabel('(m/s)', rotation=0, labelpad=20)
position_axis.set_xlabel('(s)', rotation=0, labelpad=10)

# Setting limits of axes
motor_axis.set_ylim(-4.0, 4.0)
angle_axis.set_ylim(-0.02, 0.02)
angvel_axis.set_ylim(-1, 1)
position_axis.set_ylim(-0.05, 0.05)
velocity_axis.set_ylim(-0.05, 0.05)

# Enabling grid and legend
motor_axis.grid(True)
angle_axis.grid(True)
position_axis.grid(True)
fig.legend(loc='upper center', ncol=6, bbox_to_anchor=(0.5, 0.97))

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.draw()
canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

# Setting time variable 
start_time = time.time()
slider_value = tk.DoubleVar(value=0.0)

# Initializing the plots
def init():
    motor_axis.set_xlim(0, TIME_WINDOW)
    return motor_line, angle_line, angvel_line, position_line, velocity_line, refpos_line

def update(frame):
    # Reading data from the MinSeg
    now = time.time() - start_time
    ser.reset_input_buffer()
    raw = ser.readline().decode('utf-8', errors='ignore').strip()
    if not raw or not raw.startswith('S'):
        return motor_line, angle_line, angvel_line, position_line, velocity_line, refpos_line
    
    # Parsing the data and storing it in the deques
    motor, angle, angular_velocity, position, velocity, reference_position = raw.strip('S').split(',')
    times.append(now)
    motor_values.append(float(motor))
    angle_values.append(float(angle))
    angvel_values.append(float(angular_velocity))
    position_values.append(float(position))
    velocity_values.append(float(velocity))
    refpos_values.append(float(reference_position))

    # Discarding old values and making the window slide with time 
    cutoff = now - TIME_WINDOW
    while times[0] < cutoff:
        times.popleft()
        motor_values.popleft()
        angle_values.popleft()
        angvel_values.popleft()
        position_values.popleft()
        velocity_values.popleft()
        refpos_values.popleft()
    motor_axis.set_xlim(max(0, cutoff), now)
    
    # Updating the plots
    motor_line.set_data(times, motor_values)
    angle_line.set_data(times, angle_values)
    angvel_line.set_data(times, angvel_values)
    position_line.set_data(times, position_values)
    velocity_line.set_data(times, velocity_values)
    refpos_line.set_data(times, refpos_values)
    return motor_line, angle_line, angvel_line, position_line, velocity_line, refpos_line

# Sending the slider value to the MinSeg
def on_slider(val):
    message = f"{float(val):.4f}\n".encode('utf-8')
    ser.write(message)
    ser.flush()

# Creating the slider
label = tk.Label(root, text="Reference position", font=("Helvetica", 18))
label.pack()
slider = tk.Scale(
    root,
    from_=-0.05,
    to=0.05,
    resolution=0.001,
    orient=tk.HORIZONTAL,
    variable=slider_value,
    length=400,
    command=on_slider
)
slider.pack(padx=20, pady=(0, 20))

# Closes the connection 
def on_close():
    ser.close()
    root.quit()
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)

ani = FuncAnimation(fig, update, init_func=init, interval=50, blit=False)

root.mainloop()
