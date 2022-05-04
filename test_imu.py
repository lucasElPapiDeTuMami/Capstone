from MotorSupport import Motor,Steer,InertialMeasurementUnit
import time

m = Motor()
imu = InertialMeasurementUnit()
steer = Steer()

m.sweep()
m.change_motor_speed(0.142)
steer.change_steering_angle(0.5)

while True:
    data = imu.fetchImuData()
    print(data)

