from CarSupport import Motor,Steer,InertialMeasurementUnit
import time

m = Motor()
imu = InertialMeasurementUnit()
steer = Steer()

print("Sweep Motor Vals")
m.sweep()
steer.change_steering_angle(0.5)
print("Set Motor to 0.142 and Record IMU")
m.change_motor_speed(0.142)

MAXSIZE = 10000
Gx = [0] * MAXSIZE
Gy = [0] * MAXSIZE
Gz = [0] * MAXSIZE
for i in range(MAXSIZE):
    data = imu.fetchImuData()
    Gx[i] = data[3]
    Gy[i] = data[4]
    Gz[i] = data[5]
print("Done Recording")
m.change_motor_speed(0.13)

with open('Gx.txt','w') as f:
    for i in range(MAXSIZE):
        f.write(f"{Gx[i]}")
        f.write('\n')

with open('Gy.txt','w') as f:
    for i in range(MAXSIZE):
        f.write(f"{Gy[i]}")
        f.write('\n')


with open('Gz.txt','w') as f:
    for i in range(MAXSIZE):
        f.write(f"{Gz[i]}")
        f.write('\n')


print("done recording data")