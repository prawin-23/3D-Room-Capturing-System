#Prawin Premachandran - premap1 - 400468579
import serial
import math
import numpy as np
import open3d as o3d

# Opens the connection to UART connection
s = serial.Serial(
    port='COM4',  # serial uart port is port4
    baudrate=115200,
    parity=serial.PARITY_NONE, #default of no parity, 1 stop bit, 8 bits
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1  # Set a timeout for reads, so it doesn't wait forever
)

# initializing constants
steps_per_rot = 512  # set to 512 steps 
num_degrees_per_rot = 360 / steps_per_rot #360 degrees = 1 rotation
measurements_array = [] 
current_step = 16 #initial step
z_initial = 0  # initial z height = constant 0
interrupted = False # no keyboard interrupt pressed yet

# declaring functions
def degree_conversion(step): #function to convert the step to degrees 
    return step * num_degrees_per_rot
def cartesian_conversion(distance, angle_in_deg, z): #polar to cartesian convert
    angle_in_rad = math.radians(angle_in_deg)
    x = distance * math.cos(angle_in_rad)
    y = distance * math.sin(angle_in_rad)
    return x, y, z

s.reset_output_buffer() #resetting the buffers before user signal
s.reset_input_buffer()

input("Press Enter to start:")  # waiting for user's signal to start

# Reading the UART measurements

try:
    while not interrupted: #while user interrupt is false
        line = s.readline().decode().strip()
        if line:  # executes if line is not empty
            if line.replace(".", "", 1).isdigit():  # checks if line is a valid float number
                measurement = float(line)  # length to float conversion
                angle = degree_conversion(current_step)
                x, y, z = cartesian_conversion(measurement, angle, z_initial)
                measurements_array.append((x, y, z))
                print(f"current step is: {current_step}, the coordinates are: {measurement}, X: {x}, Y: {y}, Z: {z}")

                # resetting after full rotation
                if current_step < steps_per_rot:
                    current_step += 16
                else:
                    current_step = 16
                    print(f"Finished {(z_initial / 500)+1} rotations")
                    z_initial += 500
                    
except KeyboardInterrupt:#checks for user stopping the program when needed
        interrupted = True

with open("data.xyz", "w") as f: # Write measurements to file
    for x, y, z in measurements_array:
        f.write(f"{x} {y} {z}\n")

print(f" total measurements: {len(measurements_array)}")
print("now reading the dataset")
pcd = o3d.io.read_point_cloud("data.xyz", format="xyz")
point_cloud_data = np.asarray(pcd.points) # we are storing the data and keeping it into an array
points = len(point_cloud_data)
slices = points // 32  # since we are doing 32 steps, each slice has 32 points
print("this is the PCD array:")
print(point_cloud_data)                       
lines = [] # Store the lines connecting the points

# Connect all the points in each taken slice for 1 rotation and connect corresponding points between consecutive slices
for i in range(slices):
    for j in range(32):
        current = i * 32 + j  # Index of current point
        next_point = i * 32 + (j + 1) % 32  # Index of next point in the same slice
        lines.append([current, next_point])  # Append both together

        if i < slices - 1:
            next_slice_point = (i + 1) * 32 + j  # Index of next point in the next slice
            lines.append([current, next_slice_point])  # Append current point and corresponding point in next slice

lset = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(point_cloud_data),
    lines=o3d.utility.Vector2iVector(lines)
)

# visualizing the points plot, then terminating
print("the pcd plot will be shown here")
o3d.visualization.draw_geometries([pcd])
print("the line set will be shown here")
o3d.visualization.draw_geometries([lset])
s.close()
print("scan is done :)")
