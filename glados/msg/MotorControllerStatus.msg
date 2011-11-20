# This is the msg definition for the AX3500 motor controller status struct

Header  header                   # Timestamp, sequence number, and frame id

# Wheel information
float32 left_wheel_displacement  # m
float32 right_wheel_displacement # m
float32 left_wheel_velocity      # m/s
float32 right_wheel_velocity     # m/s

# AX3500 information
#float32 left_motor_current       # amps
#float32 right_motor_current      # amps
int32   temperature              # degrees Celsius
float32 motor_battery            # volts
float32 controller_battery       # volts

#bool    motors_enabled           # Enabled (true) or E-Stopped (false)
