import sys
sys.path.append("..")
import Kinematics.kinematics as kn
import numpy as np
from smbus2 import SMBus
import time

# PCA9685 registers/addresses
PCA9685_ADDRESS = 0x40
PCA9685_MODE1 = 0x00
PCA9685_PRESCALE = 0xFE
PCA9685_LED0_ON_L = 0x06
PCA9685_LED0_ON_H = 0x07
PCA9685_LED0_OFF_L = 0x08
PCA9685_LED0_OFF_H = 0x09

class Controllers:
    def __init__(self):
        print("Initializing Servos")
        self._bus = SMBus(1)  # Use I2C bus 1 (corresponds to SCL_1, SDA_1)
        print("Initializing PCA9685 device")
        
        # Initialize the PCA9685 device
        self._init_pca9685(PCA9685_ADDRESS)
        
        print("Done initializing")
        
        # [0]~[2] : 왼쪽 앞 다리 // [3]~[5] : 오른쪽 앞 다리 // [6]~[8] : 왼쪽 뒷 다리 // [9]~[11] : 오른쪽 뒷 다리
        # centered position perpendicular to the ground
        self._servo_offsets = [170, 85, 90, 1, 95, 90, 172, 90, 90, 1, 90, 95]
        #self._servo_offsets = [90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90]
        self._val_list = np.zeros(12) #[ x for x in range(12) ]
        # All Angles for Leg 3 * 4 = 12 length
        self._thetas = []
    
    def _init_pca9685(self, address):
        # Reset PCA9685
        self._bus.write_byte_data(address, PCA9685_MODE1, 0x00)
        time.sleep(0.05)
        
        # Set PWM frequency to 50Hz (standard for servos)
        # Prescale value calculation: prescale = (25MHz / (4096 * freq)) - 1
        prescale_value = 121  # ~50Hz
        
        # To set prescale, we need to:
        # 1. Read MODE1 register
        mode1 = self._bus.read_byte_data(address, PCA9685_MODE1)
        # 2. Put the device to sleep (set SLEEP bit)
        self._bus.write_byte_data(address, PCA9685_MODE1, (mode1 & 0x7F) | 0x10)
        # 3. Set the prescale register
        self._bus.write_byte_data(address, PCA9685_PRESCALE, prescale_value)
        # 4. Restore MODE1 register (remove SLEEP bit)
        self._bus.write_byte_data(address, PCA9685_MODE1, mode1)
        time.sleep(0.005)
        # 5. Set MODE1 register with RESTART bit
        self._bus.write_byte_data(address, PCA9685_MODE1, mode1 | 0x80)
    
    def _set_pwm(self, channel, angle):
        # Map angle (0-180) to PWM value (150-600)
        # Adjust these values based on your servo's specific range
        pulse_length = int(150 + (angle / 180) * 450)
        
        # Calculate register addresses
        led_on_l = PCA9685_LED0_ON_L + (channel * 4)
        led_on_h = PCA9685_LED0_ON_H + (channel * 4)
        led_off_l = PCA9685_LED0_OFF_L + (channel * 4)
        led_off_h = PCA9685_LED0_OFF_H + (channel * 4)
        
        # Set PWM values (on at 0, off at pulse_length)
        self._bus.write_byte_data(PCA9685_ADDRESS, led_on_l, 0)
        self._bus.write_byte_data(PCA9685_ADDRESS, led_on_h, 0)
        self._bus.write_byte_data(PCA9685_ADDRESS, led_off_l, pulse_length & 0xFF)
        self._bus.write_byte_data(PCA9685_ADDRESS, led_off_h, pulse_length >> 8)
    
    def getDegreeAngles(self, La):
        # radian to degree
        La *= 180/np.pi
        La = [ [ int(x) for x in y ] for y in La ]
        self._thetas = La
    
    # Angle mapping from radian to servo angles
    def angleToServo(self, La):
        self.getDegreeAngles(La)
        #FL Lower
        self._val_list[0] = self._servo_offsets[0] - self._thetas[0][2]
        #FL Upper
        self._val_list[1] = self._servo_offsets[1] - self._thetas[0][1]    
        #FL Shoulder
        self._val_list[2] = self._servo_offsets[2] + self._thetas[0][0]
        #FR Lower
        self._val_list[3] = self._servo_offsets[3] + self._thetas[1][2]
        #FR Upper
        self._val_list[4] = self._servo_offsets[4] + self._thetas[1][1]    
        #FR Shoulder
        self._val_list[5] = self._servo_offsets[5] - self._thetas[1][0]
        #BL Lower
        self._val_list[6] = self._servo_offsets[6] - self._thetas[2][2]
        #BL Upper
        self._val_list[7] = self._servo_offsets[7] - self._thetas[2][1]    
        #BL Shoulder, Formula flipped from the front
        self._val_list[8] = self._servo_offsets[8] - self._thetas[2][0]
        #BR Lower. 
        self._val_list[9] = self._servo_offsets[9] + self._thetas[3][2]
        #BR Upper
        self._val_list[10] = self._servo_offsets[10] + self._thetas[3][1]    
        #BR Shoulder, Formula flipped from the front
        self._val_list[11] = self._servo_offsets[11] + self._thetas[3][0]
    
    def getServoAngles(self):
        return self._val_list
    
    def servoRotate(self, thetas):
        self.angleToServo(thetas)
        
        for x in range(len(self._val_list)):
            if x >= 0 and x < 12:
                self._val_list[x] = (self._val_list[x]-26.36)*(1980/1500)
                print(self._val_list[x])
                
                if (self._val_list[x] > 180):
                    print("Over 180!!")
                    self._val_list[x] = 179
                elif (self._val_list[x] <= 0):
                    print("Under 0!!")
                    self._val_list[x] = 1
                
                # Set the servo angle using our _set_pwm method
                self._set_pwm(x, self._val_list[x])

if __name__=="__main__":
    legEndpoints=np.array([[100,-100,87.5,1],[100,-100,-87.5,1],[-100,-100,87.5,1],[-100,-100,-87.5,1]])
    thetas = kn.initIK(legEndpoints) #radians
    
    controller = Controllers()
    # Get radian thetas, transform to integer servo angles
    # then, rotate servos
    controller.servoRotate(thetas)
    # Get AngleValues for Debugging
    svAngle = controller.getServoAngles()
    print(svAngle)
    # #plot at the end
    
