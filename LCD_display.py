from smbus2 import SMBus
import time

# Define LCD I2C address (typically 0x27 or 0x3F)
LCD_ADDR = 0x27

# Define PCA9685 I2C address (typically 0x40)
PCA9685_ADDR = 0x40

# LCD commands
LCD_CLEARDISPLAY = 0x01
LCD_RETURNHOME = 0x02
LCD_ENTRYMODESET = 0x04
LCD_DISPLAYCONTROL = 0x08
LCD_CURSORSHIFT = 0x10
LCD_FUNCTIONSET = 0x20
LCD_SETCGRAMADDR = 0x40
LCD_SETDDRAMADDR = 0x80

# LCD flags for display entry mode
LCD_ENTRYRIGHT = 0x00
LCD_ENTRYLEFT = 0x02
LCD_ENTRYSHIFTINCREMENT = 0x01
LCD_ENTRYSHIFTDECREMENT = 0x00

# LCD flags for display on/off control
LCD_DISPLAYON = 0x04
LCD_DISPLAYOFF = 0x00
LCD_CURSORON = 0x02
LCD_CURSOROFF = 0x00
LCD_BLINKON = 0x01
LCD_BLINKOFF = 0x00

# LCD flags for display/cursor shift
LCD_DISPLAYMOVE = 0x08
LCD_CURSORMOVE = 0x00
LCD_MOVERIGHT = 0x04
LCD_MOVELEFT = 0x00

# LCD flags for function set
LCD_8BITMODE = 0x10
LCD_4BITMODE = 0x00
LCD_2LINE = 0x08
LCD_1LINE = 0x00
LCD_5x10DOTS = 0x04
LCD_5x8DOTS = 0x00

# LCD backlight control bits
LCD_BACKLIGHT = 0x08
LCD_NOBACKLIGHT = 0x00

# Register flags for PCA9685
PCA9685_MODE1 = 0x00
PCA9685_PRESCALE = 0xFE

# Enable bit for LCD
En = 0b00000100
Rs = 0b00000001

def lcd_init():
    """Initialize the LCD display."""
    # Initialize display
    lcd_write(0x03, 0)
    lcd_write(0x03, 0)
    lcd_write(0x03, 0)
    lcd_write(0x02, 0)
    
    # Configure LCD: 4-bit mode, 2 lines, 5x8 dots
    lcd_write(LCD_FUNCTIONSET | LCD_2LINE | LCD_5x8DOTS | LCD_4BITMODE, 0)
    
    # Turn on display, cursor off, blink off
    lcd_write(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF, 0)
    
    # Set entry mode
    lcd_write(LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT, 0)
    
    # Clear display
    lcd_write(LCD_CLEARDISPLAY, 0)
    time.sleep(0.2)

def lcd_toggle_enable(val, bus):
    """Toggle the enable bit to execute LCD command."""
    time.sleep(0.0005)
    bus.write_byte(LCD_ADDR, (val & ~En) | LCD_BACKLIGHT)
    time.sleep(0.0005)
    bus.write_byte(LCD_ADDR, (val | En) | LCD_BACKLIGHT)
    time.sleep(0.0005)
    bus.write_byte(LCD_ADDR, (val & ~En) | LCD_BACKLIGHT)
    time.sleep(0.001)

def lcd_write(bits, mode):
    """Send command to LCD."""
    with SMBus(1) as bus:  # Use 1 for Raspberry Pi 2 or newer, 0 for older versions
        # High bits
        high_bits = mode | (bits & 0xF0) | LCD_BACKLIGHT
        bus.write_byte(LCD_ADDR, high_bits)
        lcd_toggle_enable(high_bits, bus)
        
        # Low bits
        low_bits = mode | ((bits << 4) & 0xF0) | LCD_BACKLIGHT
        bus.write_byte(LCD_ADDR, low_bits)
        lcd_toggle_enable(low_bits, bus)

def lcd_write_char(char):
    """Write a single character to the LCD."""
    lcd_write(ord(char), Rs)

def lcd_display_string(string, line):
    """Write a string to the specified LCD line."""
    line_addresses = [0x80, 0xC0, 0x94, 0xD4]
    
    lcd_write(line_addresses[line-1], 0)
    
    for char in string:
        lcd_write_char(char)

def lcd_clear():
    """Clear the LCD display."""
    lcd_write(LCD_CLEARDISPLAY, 0)
    time.sleep(0.002)

def check_pca9685():
    """Check if PCA9685 servo controller is accessible."""
    try:
        with SMBus(1) as bus:  # Use 1 for Raspberry Pi 2 or newer, 0 for older versions
            # Try to read the MODE1 register from PCA9685
            bus.read_byte_data(PCA9685_ADDR, PCA9685_MODE1)
        return True
    except Exception:
        return False

def main():
    # Initialize LCD
    try:
        lcd_init()
        print("LCD initialized successfully")
    except Exception as e:
        print(f"Error initializing LCD: {e}")
        return

    # Clear LCD and show project name
    lcd_clear()
    lcd_display_string("AICLUB_UIT", 1)
    
    # Check PCA9685 connection
    if check_pca9685():
        lcd_display_string("PCA9685: OK", 2)
        print("PCA9685 connected successfully")
    else:
        lcd_display_string("PCA9685: FAIL", 2)
        print("PCA9685 not detected")

if __name__ == "__main__":
    try:
        main()
        time.sleep(10)
    except KeyboardInterrupt:
        pass
    finally:
        # Clear display on exit
        try:
            lcd_clear()
            lcd_display_string("Shutting down...", 1)
            time.sleep(1)
            lcd_clear()
        except:
            pass
