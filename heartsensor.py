import time
from smbus2 import SMBus
from gpiozero import LED, Button

# === Optional GPIO ===
led = LED(17)        # Change to the GPIO pin you connected the LED to
button = Button(2)   # Change to your button GPIO (or remove if unused)

# === MAX30102 Constants ===
I2C_ADDRESS = 0x57  # Default I2C address for MAX30102
REG_INTR_STATUS_1 = 0x00
REG_INTR_STATUS_2 = 0x01
REG_FIFO_DATA = 0x07
REG_MODE_CONFIG = 0x09
REG_SPO2_CONFIG = 0x0A
REG_LED1_PA = 0x0C  # IR LED
REG_LED2_PA = 0x0D  # RED LED

# === I2C Bus (usually bus 1 on Raspberry Pi) ===
bus = SMBus(1)

def max30102_init():
    # Reset the sensor
    bus.write_byte_data(I2C_ADDRESS, REG_MODE_CONFIG, 0x40)
    time.sleep(0.1)
    
    # Set to SpO2 mode
    bus.write_byte_data(I2C_ADDRESS, REG_MODE_CONFIG, 0x03)
    
    # SpO2 config: 100 Hz sample rate, 16-bit resolution
    bus.write_byte_data(I2C_ADDRESS, REG_SPO2_CONFIG, 0x27)
    
    # Set LED pulse amplitudes
    bus.write_byte_data(I2C_ADDRESS, REG_LED1_PA, 0x24)  # IR
    bus.write_byte_data(I2C_ADDRESS, REG_LED2_PA, 0x24)  # Red
    
    print("MAX30102 Initialized")

def read_fifo_sample():
    # Read 6 bytes from FIFO (3 bytes IR, 3 bytes Red)
    data = bus.read_i2c_block_data(I2C_ADDRESS, REG_FIFO_DATA, 6)
    
    ir = (data[0] << 16 | data[1] << 8 | data[2]) & 0x03FFFF
    red = (data[3] << 16 | data[4] << 8 | data[5]) & 0x03FFFF
    return ir, red

# === Initialize Sensor ===
max30102_init()

print("Reading data... Press button to stop (or Ctrl+C)")

try:
    while True:
        ir_val, red_val = read_fifo_sample()
        print(f"IR: {ir_val}, RED: {red_val}")
        
        led.on()
        time.sleep(0.05)
        led.off()

        # Stop if button is pressed
        if button.is_pressed:
            print("Button pressed — exiting...")
            break

        time.sleep(0.2)

except KeyboardInterrupt:
    print("Stopped by Ctrl+C")

finally:
    bus.close()
    led.off()
