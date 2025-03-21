#!/usr/bin/env python3
import pylibi2c
import time
import math

class PCA9685:
    # PCA9685 Register addresses
    MODE1 = 0x00
    MODE2 = 0x01
    SUBADR1 = 0x02
    SUBADR2 = 0x03
    SUBADR3 = 0x04
    PRESCALE = 0xFE
    LED0_ON_L = 0x06
    LED0_ON_H = 0x07
    LED0_OFF_L = 0x08
    LED0_OFF_H = 0x09
    ALL_LED_ON_L = 0xFA
    ALL_LED_ON_H = 0xFB
    ALL_LED_OFF_L = 0xFC
    ALL_LED_OFF_H = 0xFD

    # Bits
    RESTART = 0x80
    SLEEP = 0x10
    ALLCALL = 0x01
    INVRT = 0x10
    OUTDRV = 0x04

    def __init__(self, i2c_bus=1, address=0x40):
        """
        Initialize the PCA9685 using pylibi2c
        """
        self.address = address
        self.i2c = pylibi2c.I2CDevice('/dev/i2c-' + str(i2c_bus), address)
        self.reset()
        self.set_pwm_freq(50)  # Default to 50Hz for servos

    def reset(self):
        """
        Reset the PCA9685
        """
        self.write_byte(self.MODE1, self.ALLCALL)
        self.write_byte(self.MODE2, self.OUTDRV)
        time.sleep(0.005)  # Wait for oscillator

    def write_byte(self, reg, value):
        """
        Write a single byte to a register
        """
        data = bytearray([value])
        self.i2c.ioctl_write(reg, data)

    def read_byte(self, reg):
        """
        Read a single byte from a register
        """
        data = bytearray(1)
        self.i2c.ioctl_read(reg, data)
        return data[0]

    def set_pwm_freq(self, freq_hz):
        """
        Set the PWM frequency in Hz
        """
        prescaleval = 25000000.0    # 25MHz
        prescaleval /= 4096.0       # 12-bit
        prescaleval /= float(freq_hz)
        prescaleval -= 1.0
        
        prescale = int(math.floor(prescaleval + 0.5))
        
        oldmode = self.read_byte(self.MODE1)
        newmode = (oldmode & 0x7F) | self.SLEEP
        
        self.write_byte(self.MODE1, newmode)  # Go to sleep
        self.write_byte(self.PRESCALE, prescale)
        self.write_byte(self.MODE1, oldmode)
        time.sleep(0.005)
        self.write_byte(self.MODE1, oldmode | self.RESTART)

    def set_pwm(self, channel, on, off):
        """
        Set the PWM for a specific channel
        """
        on_l = on & 0xFF
        on_h = (on >> 8) & 0x0F
        off_l = off & 0xFF
        off_h = (off >> 8) & 0x0F
        
        # Write the ON values
        self.write_byte(self.LED0_ON_L + 4 * channel, on_l)
        self.write_byte(self.LED0_ON_H + 4 * channel, on_h)
        
        # Write the OFF values
        self.write_byte(self.LED0_OFF_L + 4 * channel, off_l)
        self.write_byte(self.LED0_OFF_H + 4 * channel, off_h)

    def set_all_pwm(self, on, off):
        """
        Set the PWM for all channels
        """
        on_l = on & 0xFF
        on_h = (on >> 8) & 0x0F
        off_l = off & 0xFF
        off_h = (off >> 8) & 0x0F
        
        self.write_byte(self.ALL_LED_ON_L, on_l)
        self.write_byte(self.ALL_LED_ON_H, on_h)
        self.write_byte(self.ALL_LED_OFF_L, off_l)
        self.write_byte(self.ALL_LED_OFF_H, off_h)

    def set_servo_pulse(self, channel, pulse_ms):
        """
        Set servo pulse in milliseconds
        Typical servo pulse is between 1ms (0 degrees) and 2ms (180 degrees)
        """
        pulse_length = 1000000    # 1,000,000 us per second
        pulse_length //= 50       # 50 Hz
        pulse_length //= 4096     # 12 bits of resolution
        
        # Convert pulse from milliseconds to pulse length count
        pulse *= 1000
        pulse //= pulse_length
        
        self.set_pwm(channel, 0, pulse)

    def set_servo_angle(self, channel, angle):
        """
        Set servo angle (0-180 degrees)
        """
        # Map angle (0-180) to pulse (1-2ms)
        pulse = 1.0 + (angle / 180.0)
        self.set_servo_pulse(channel, pulse)


# Example usage
if __name__ == "__main__":
    # Initialize PCA9685
    pca = PCA9685(i2c_bus=1, address=0x40)
    
    try:
        # Example: Move servo on channel 0 from 0 to 180 degrees
        print("Moving servo on channel 0 from 0 to 180 degrees")
        for angle in range(0, 181, 5):
            pca.set_servo_angle(0, angle)
            print(f"Angle: {angle}")
            time.sleep(0.1)
        
        # Wait a second
        time.sleep(1)
        
        # Move back from 180 to 0 degrees
        print("Moving servo on channel 0 from 180 to 0 degrees")
        for angle in range(180, -1, -5):
            pca.set_servo_angle(0, angle)
            print(f"Angle: {angle}")
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        # Center the servo before exiting
        pca.set_servo_angle(0, 90)
        print("Program stopped by user")
