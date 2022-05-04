# Import Modules
import cv2
import picamera.array
import smbus
import math
import busio
import time
import numpy as np
from picamera import PiCamera
from board import SCL, SDA
from adafruit_pca9685 import PCA9685











class Camera:
    """
    This Class combines PiCamera and computes states asociated with car yaw angle.

    ...
    Attributes
    ----------
    camera : PiCamera
        This sets up an instance of PiCamera to stream images at framerate (default 30=fps).
        The camera is setup to output BGR 3-dimensional arrays at resolution (default = (80,52)). 
    
    Methods
    -------
    take_picture()
        Capture images and convert info to BGR array.
        
    compute_psi(img)
        Compute yaw angle (denoted by psi) w.r.t. lane using Edge detection and Hough Transform.
    
    """
    def __init__(self):
        self.camera = PiCamera()
        self.camera.framerate = 30
        self.stream = picamera.array.PiRGBArray(self.camera)
        self.camera.resolution = (320,208)

    def take_picture(self):
        """Capture images and convert info to BGR array."""
        img = np.empty((self.res[0]*self.res[1]*3), dtype = np.uint8) 
        self.camera.capture(self.stream, format = 'bgr', use_video_port = True)
        img = self.stream.array
        self.stream.truncate(0)
        return img

    def compute_psi(self,img):
        """ Compute yaw angle w.r.t. lane using Canny edge detectior and Hough transform.

        Arguments
        ---------
            img : n by m by 3 array
                Three-dimienional array representing image taken in BGR format.
            
        Returns
        -------
            psi : double
                Yaw angle of car w.r.t. lane.
        """

        # Perspective Transform
        pts1 = np.float32([[105,0],[210,0],[0,200],[320,200]])
        pts2 = np.float32([[0,0],[320,0],[0,208],[320,208]])
        M = cv2.getPerspectiveTransform(pts1,pts2)
        wrap = cv2.warpPerspective(img,M,(320,200))

        # Keep One Color Channel.
        #blue = wrap[:,:,0]
        #green = wrap[:,:,1]
        red = wrap[:,:,2]
        keep = red # NOTE: Adjust as necessary

        # Clean Noise. Find Edges.
        blur = cv2.blur(keep,(10,10))
        hist = cv2.calcHist([blur],[0],None,[256],[0,256])
        _,thrsh = cv2.threshold(blur,70,255,cv2.THRESH_BINARY)
        canny = cv2.Canny(thrsh,threshold1=25,threshold2=30)

        # Perform Hough Transform. Compute Psi.
        lines = cv2.HoughLines(canny,1,np.pi/180,35)
        rho, theta = lines[0,:,:] # Keep Candidate With Most Votes.\
        if theta > np.pi/2:
            psi = theta - np.pi
        else:
            psi = theta

        return psi






















class InertialMeasurementUnit:
    """ 
    Initializes IMU hardware and outputs 6-by-1 array with accel. & gyro. data.

    The IMU used is the MCP 6050. I took adapted some code from the
    "Gyro and Accelerometer by Interfacing Raspberry Pi with MPU 6050 using Python"
    on http://www.electronicwings.com

    Attributes
    ----------
    Device_Address : hex
        MPU6050 Device Address.

    bus: SMBus

    Methods
    -------

    read_raw_data()
        Interfaces with hardware to read raw data. NOTE: Must Be Converted To Accel.
    
    fetchIMUdata()
        Converts output from read_raw_data to translational acceleration (g)
        and rotational velocity (rad/s) on all three body axises. Outputs data
        into array
	
	[Gx, Gy, Gz, Ax, Ay, Az]

    

    """
    def __init__(self):
        self.Device_Address = 0x68 # MPU6050 device address 
        PWR_MGMT_1   = 0x6B
        SMPLRT_DIV   = 0x19
        CONFIG       = 0x1A
        GYRO_CONFIG  = 0x1B
        INT_ENABLE   = 0x38
        ACCEL_XOUT_H = 0x3B
        ACCEL_YOUT_H = 0x3D
        ACCEL_ZOUT_H = 0x3F
        GYRO_XOUT_H  = 0x43
        GYRO_YOUT_H  = 0x45
        GYRO_ZOUT_H  = 0x47
        # Initialize Bus
        self.bus = smbus.SMBus(1)
        self.bus.write_byte_data(self.Device_Address, SMPLRT_DIV, 7) # Write To Sample Rate Register
        self.bus.write_byte_data(self.Device_Address, PWR_MGMT_1, 1) # Write To Power Management Register
        self.bus.write_byte_data(self.Device_Address, CONFIG, 0) # Write To Configuration Register
        self.bus.write_byte_data(self.Device_Address, GYRO_CONFIG, 24) # Write To Gyro Config. Register
        self.bus.write_byte_data(self.Device_Address, INT_ENABLE, 1) # Write To Interrupt Enable Register

    def read_raw_data(self, addr):
	    #Accelero and Gyro value are 16-bit
            high = self.bus.read_byte_data(self.Device_Address, addr)
            low = self.bus.read_byte_data(self.Device_Address, addr+1)
    
            # concatenate higher and lower value
            value = ((high << 8) | low)
        
            # to get signed value from mpu6050
            if(value > 32768):
                value = value - 65536
            return value

    def fetchImuData(self):
        """
        Converts output from read_raw_data to translational acceleration (m/s**2)
        and rotational velocity (deg/s) on all three body axises. Outputs data
        into array.

	    If you look at MPU6050's labels and roate left, that will give you a
        positive Gz reading. (in deg/s)

        Returns
        -------

        data : [double]
            data[0:2] = translational acceleration for sensor x,y,z axes.
            data[3:5] = rotational velocity for sensor x,y,z axes.
            data = [Ax,Ay,Az,p,q,r]
        """
        ACCEL_XOUT_H = 0x3B
        ACCEL_YOUT_H = 0x3D
        ACCEL_ZOUT_H = 0x3F
        GYRO_XOUT_H  = 0x43
        GYRO_YOUT_H  = 0x45
        GYRO_ZOUT_H  = 0x47
        # Read Accelerometer raw value
        acc_x = self.read_raw_data(ACCEL_XOUT_H)
        acc_y = self.read_raw_data(ACCEL_YOUT_H)
        acc_z = self.read_raw_data(ACCEL_ZOUT_H)
        # Read Gyroscope raw value
        gyro_x = self.read_raw_data(GYRO_XOUT_H)
        gyro_y = self.read_raw_data(GYRO_YOUT_H)
        gyro_z = self.read_raw_data(GYRO_ZOUT_H)
        # Full scale range +/- 250 degree/C as per sensitivity scale factor
        Ax = acc_x/16384.0
        Ay = acc_y/16384.0
        Az = acc_z/16384.0
        Gx = gyro_x/131.0
        Gy = gyro_y/131.0
        Gz = gyro_z/131.0
        # organize data into list
	# Convert Gyro Data units from (deg/s) -> (rad/s)
        data = [Ax, Ay, Az, np.deg2rad(Gx), np.deg2rad(Gy), np.deg2rad(Gz)]
        return data




















class Motor:
    """
    A class that handles the initialization of the motor and allows for
    the user to set the speed.

    Attributes
    ----------
    pca : PCA96825
        Object instance from adafruit PCA9685 support class. 
    channel : int
        Channel through which data is transmitted.
    frequency : int
        Frequency at which motor operates.
    
    Methods
    -------
    
    sweep()
        Sweeps through all PWM signals until motor is unlocked. 
    
    set_motor_speed(percent)
        Changes the motor speed to the desired percent of 2 ** 16 bits.
        This input generates a PWM signal.

    """

    def __init__(self,channel = 8, frequency = 100):
        # Pass in channel
        self.channel = channel
        i2c_bus = busio.I2C(SCL,SDA)
        self.pca = PCA9685(i2c_bus)
        self.pca.frequency = frequency
    
    def sweep(self):
        print("Start Sweep")
        for i in range(20):
            percent = i*0.001+0.13
            bits = math.floor(percent * 65535)
            self.pca.channels[self.channel].duty_cycle = bits
            time.sleep(0.3)
            print(f"i {i} percent: {percent} bits: {bits}")
    
    def set_motor_speed(self,percent):
        self.pca.channels[self.channel].duty_cycle = math.floor(percent * 65535)
















    

class Steer:
    """ 
    This class handles the servo that steers the car.

    Attributes
    ----------
    pca : PCA9685
        Object instance from adafruit PCA9685 support class.
        This object contains the channel number, the i2c bus, and the frequency.
    channel : int
        Channel through which data is transmitted
    frequency : int
        Frequency at which servo operates

    Methods
    -------
    compute_steering_angle(psi,r,psiIE)
        Computes Steering Angle (rad) to send to steer servo.
    change_steering_angle(delta_steering, channel = 11)
        Changes the steering angle speed to the desired angle of car (in radians).

    """
    def __init__(self, channel = 0, frequency = 100):
        self.frequency = frequency
        self.channel = channel
        i2c_bus = busio.I2C(SCL,SDA)
        self.pca = PCA9685(i2c_bus)
        self.pca.frequency = frequency
    
    def compute_steering_angle(psi,r):
        """
        Computes Steering Angle (rad) to send to steer servo. This is the Robust
        Servomechanism Linear Quadratic Regulator control algorithm designed,
        simulated, and analyzed in MATLAB.
        """
        steering_angle =  - 1.8654 * psi - 0.9638 * r
        return steering_angle
    
    
    def change_steering_angle(self,steering_angle):
        """Change car steering angle tire to delta_steer (in radians)"""
        BIT = 65536
        duty = 347.8 * (steering_angle ** 2) + 4530 * steering_angle + 9648

        # Virtual Saturation Limit for safety:
        if duty > 13107:
            duty = 13107
            print("WARNING: Actuator Saturated")
        elif duty < 6553:
            duty = 6553
            print("WARNING: Actuator Saturated")
        else:
            duty = math.floor(duty)
        self.pca.channels[self.channel].duty_cycle = duty
