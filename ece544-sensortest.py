import smbus
import time







# TSL2561 I2C address (default)
I2C_ADDR = 0x39  

# Register addresses
COMMAND_BIT = 0x80
CONTROL_REGISTER = 0x00
TIMING_REGISTER = 0x01
DATA0_LOW = 0x0C
DATA0_HIGH = 0x0D
DATA1_LOW = 0x0E
DATA1_HIGH = 0x0F

# Power on the sensor
POWER_ON = 0x03
POWER_OFF = 0x00

# Initialize I2C bus
bus = smbus.SMBus(1)  # Use bus 1 for Raspberry Pi

def tsl2561_setup():
    """Power on the sensor"""
    bus.write_byte_data(I2C_ADDR, COMMAND_BIT | CONTROL_REGISTER, POWER_ON)
    time.sleep(0.5)  # Allow time for the sensor to power up

def read_lux():
    """Read raw data from the sensor"""
    ch0_low = bus.read_byte_data(I2C_ADDR, COMMAND_BIT | DATA0_LOW)
    ch0_high = bus.read_byte_data(I2C_ADDR, COMMAND_BIT | DATA0_HIGH)
    ch1_low = bus.read_byte_data(I2C_ADDR, COMMAND_BIT | DATA1_LOW)
    ch1_high = bus.read_byte_data(I2C_ADDR, COMMAND_BIT | DATA1_HIGH)

    # Combine high and low bytes
    full_spectrum = (ch0_high << 8) | ch0_low
    infrared = (ch1_high << 8) | ch1_low

    return full_spectrum, infrared

# Setup sensor
tsl2561_setup()

print("TSL2561 Light Sensor Test")
try:
    while True:
        full, ir = read_lux()
        print(f"Full Spectrum: {full}, Infrared: {ir}")
        time.sleep(1)
except KeyboardInterrupt:
    print("\nTest stopped by user.")
    bus.write_byte_data(I2C_ADDR, COMMAND_BIT | CONTROL_REGISTER, POWER_OFF)
    print("Sensor powered off.")
