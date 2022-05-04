from CarSupport import Motor,Steer,InertialMeasurementUnit
import time

m = Motor()
imu = InertialMeasurementUnit()
steer = Steer()

print("Sweep Motor Vals")
m.sweep()
steer.change_steering_angle(0.5)
print("Set Motor to 0.146")
m.change_motor_speed(0.146)

print("PUT CAR ON FLOOR!")
time.sleep(1)




MAXSIZE = 500
Gx = [0] * MAXSIZE
Gy = [0] * MAXSIZE
Gz = [0] * MAXSIZE
print("Start Recording")
for i in range(MAXSIZE):
    data = imu.fetchImuData()
    Gx[i] = data[3]
    Gy[i] = data[4]
    Gz[i] = data[5]
    print(i)
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


print("Done Writing Data")
