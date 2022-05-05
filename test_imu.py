from CarSupport import Motor,Steer,InertialMeasurementUnit
import time
motor = Motor()
imu = InertialMeasurementUnit()
steer = Steer()
delta = 0.0
print("Set Steering Angle Forward (0.0)")
steer.change_steering_angle(delta)

print("Start Sweep Sequence")
motor.sweep()

print("Set Motor Speed 0.147")
motor.change_motor_speed(0.147)
print("SET CAR ON FLOOR!")

MAXSIZE = 100000
Ax = [0] * MAXSIZE
Ay = [0] * MAXSIZE
Az = [0] * MAXSIZE
Gx = [0] * MAXSIZE
Gy = [0] * MAXSIZE
Gz = [0] * MAXSIZE
d = [0] * MAXSIZE
t = [0] * MAXSIZE

startTime, curTime = time.time(), time.time()
runTime = 5
turnTime = 1

i = 0
while (startTime + runTime > curTime):
    curTime = time.time()
    if (startTime + turnTime < curTime ): # TURN
        delta = 0.50
        steer.change_steering_angle(delta)
    data = imu.fetchImuData()
    Ax[i],Ay[i],Az[i] = data[0],data[1],data[2] 
    Gx[i],Gy[i],Gz[i] = data[3],data[4],data[5]
    d[i] = delta
    t[i] = float(curTime - startTime)
    i += 1

motor.change_motor_speed(0.13)



with open('Gx.txt','w') as f:
    for j in range(i):
        f.write(f"{Gx[j]}")
        f.write('\n')
with open('Gy.txt','w') as f:
    for j in range(i):
        f.write(f"{Gy[j]}")
        f.write('\n')
with open('Gz.txt','w') as f:
    for j in range(i):
        f.write(f"{Gz[j]}")
        f.write('\n')
with open('Ax.txt','w') as f:
    for j in range(i):
        f.write(f"{Ax[j]}")
        f.write('\n')
with open('Ay.txt','w') as f:
    for j in range(i):
        f.write(f"{Ay[j]}")
        f.write('\n')
with open('Az.txt','w') as f:
    for j in range(i):
        f.write(f"{Az[j]}")
        f.write('\n')
with open('delta.txt','w') as f:
    for j in range(i):
        f.write(f"{d[j]}")
        f.write('\n')
with open('t.txt','w') as f:
    for j in range(i):
        f.write(f"{t[j]}")
        f.write('\n')
