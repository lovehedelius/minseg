This program was written as part of an assignment in the course Real-Time Systems at LTH. Control theory, signal processing and real-time programming techniques were used to make a MinSeg, a small two-wheeled robot similar to a Segway, balance on its own in a reference position set by the user. The Arduino sketch integrates readings from a gyroscope, accelerometer and wheel encoder to continuously estimate the robot's state and compute motor commands in real time. A Python GUI is included for monitoring the robot's behavior and adjusting the reference position interactively.

Key features:
- Real-time balancing using complementary filtering and state feedback control.
- Embedded C++ program running on Arduino-compatible hardware.
- Python GUI for live plotting of states and user input.
