#INSPIRE AWARD PROJECT
#(smart_watch)

#   !! main mode  !!
# add script start confirmation by led blinking(twice or thrice) 
# continuosly check for heart rate
# if heart rate is in danger zone(below 60 or more than 120) it will warn user
# WARNING!! flash led bulb and vibrate motor
# if button is not pressed in safe time(30 second) then..
# send message to hospital including location and heart rate data and...
# condition(patient responding or not) and it will cont. to alert the patient via motor
# if button is pressed the watch note a false alarm for 1 minute then do the normal operations after that

#   !!   demo mode !!
# fake heartrate will be provided to watch which will continously decrease
# rest of the functionality be same 

#!!!!!!!!!!!!!        add emergency button_press like if button is pressed there is some kind of panic is happening


"""
INDEX:
1. buzzer
2. lcd_screen
3. led_bulb or motor(same code or circuit)
4. sim800l

"""
#!!        BUZZER_CIRCUIT.          !!
"""
(+) -> gpio18 or pin 12
(-) -> GND
"""
#!!        LCD_SCREEN_CIRCUIT.          !!
"""
GND->GND
VCC->5v(pin2)
SDA->pin3
SLA->pin5
"""

#!!       LED_BULB & VIBRATION_MOTOR          !!
"""
circuit for both(change gpio for different functions)

led (+) -> R with very low resistance (like 100 ohm) -> gpio 17 or pin 11 of RPi
led (GND) -> any GND of RPi

"""

#!!          sim800l_circuit                   !!
#it works on serial=ttyAMA0 not serial0
# sim800l(vcc or vdd) -> 3.7v external source (+).  [buck converter not available so external power is needed]

"""
sim800l(gnd) -> common to RPi GND and external GND(-)
sim(TXD) -> RPi (gpio 15)RX.  #directly
sim(RXD) -> use 1k Ohm R(works for now)[use 2 R of one with 1k and another with 2k ohm value for best case]. -> RPi(gpio 14)TX 



"""

#speaker imports
from gpiozero import PWMOutputDevice
from time import sleep
import time

#screen imports
from luma.core.interface.serial import i2c      #led_screen_library install if not already (luma.oled)
from luma.oled.device import ssd1306			#led_screen_library 
from PIL import ImageDraw, ImageFont, Image     #image_show_library_for_screen

#bulb and vibration motor imports
from gpiozero import LED, Button        #install gpiozero

#sim800l imports
import serial         # install pyserial(pip install pyserial or sudo apt install python3-pyserial) if not already 

import threading      #multitasking

stop_event = threading.Event()

def ring(ring_oscillation):
    buzzer = PWMOutputDevice(18)                #gpio 18 check for gpio
    for i in range(ring_oscillation):
        buzzer.value = 0
        sleep(0.1)
        buzzer.value = 1
        sleep(0.1)
        buzzer.value = 0
        sleep(0.1)
        buzzer.value = 1
        sleep(0.6)


button2 = Button(22)                    #experimental    don't use anywhere!!

def button_pressed_threading():
    button = Button(2)                      #gpio 2
    while not stop_event.is_set():
        print("checking button press")
        if button.is_pressed:
            stop_event.set()
        sleep(0.1)

def button_pressed():
    button = Button(2)                      #gpio 2
    if button.is_pressed:
        sleep(0.1)
        return True
    else:
        sleep(0.1)
        return False
        


def blink_led_and_vibration_threading(repetation):
    led = LED(17)                           #gpio 17
    while not stop_event.is_set():
        for _ in range(repetation):
            led.on()
            sleep(0.3)
            led.off()
            sleep(0.3)

def send_sms(phone_number, message):
    
    # Configure the serial port
    ser = serial.Serial(
    port="/dev/ttyAMA0",  # Serial0 ttyAMA0 
    baudrate=9600,
    timeout=1
)
    screen_message("Initializing...", "top")
    ser.write(b'AT\r')
    sleep(1)
    ser.write(b'AT+CMGF=1\r')  # Set SMS text mode
    sleep(1)
    ser.write(('AT+CMGS="{}"\r'.format(phone_number)).encode())
    sleep(1)
    ser.write((message + chr(26)).encode())  # End with Ctrl+Z
    sleep(3)
    screen_message("Message sent!", "center")

    # Example usage
    #send_sms("+917717619230", "Hello from Raspberry Pi + SIM800L!")

    #working terminal commands for sim800minicom -b 9600 -D /dev/ttyAMA0


import smbus2
import time

class MAX30100Sensor:
    def __init__(self, bus=1, address=0x57):
        self.bus = smbus2.SMBus(bus)
        self.address = address
        self.initialize_sensor()

    def initialize_sensor(self):
        try:
            # Basic initialization — check your own init sequence
            self.bus.write_byte_data(self.address, 0x06, 0x03)  # mode config (e.g., SpO2 mode)
            self.bus.write_byte_data(self.address, 0x09, 0x27)  # SpO2 config
            print("MAX30100 Initialized.")
        except Exception as e:
            print(f"MAX30100 init error: {e}")

    def read_heartbeat(self):
        try:
            ir = self.bus.read_word_data(self.address, 0x05)
            red = self.bus.read_word_data(self.address, 0x06)
            return ir, red
        except Exception as e:
            print(f"I2C read error: {e}")
            return None, None

def send_sms_if_button_not_pressed(button_obj, heartbeat, gps_location):
    start_time = time.time()
    while time.time() - start_time < 5:
        if button_obj.is_pressed:
            return True
        sleep(0.1)
    send_sms("hospital number", f"person in need. heartbeat is {heartbeat}. gps location is {gps_location}")
    screen_message("message sent to hospital", "center")
    return False



def screen_message(message, position):   
    # I2C setup
    serial = i2c(port=1, address=0x3C)                  #can be checked with “i2cdetect -y 1”
    device = ssd1306(serial)                            #screenmodel

    # Load font
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 14)

    # Create a new image and draw text
    image = Image.new("1", device.size)
    draw = ImageDraw.Draw(image)
    if position == "top":
        draw.text((0, 20), f"{message}", font=font, fill=255)
    elif position == "bottom":
        draw.text((30, 20), f"{message}", font=font, fill=255)
    elif position == "center":
        draw.text((10, 20), f"{message}", font=font, fill=255)
    device.display(image)           # Turn ON (show text)
    sleep(0.3)

    device.clear()                  # Turn OFF (clear screen)
    sleep(0.3)

    # Optional: Keep screen blank or show final text
    device.clear()
    device.display()

def get_location():
    location = "https://maps.app.goo.gl/aKyVGF9V787wzoTN6"
    return location


#voilence help(police)
def police_emergency():
    button = Button(2)
    press_times = []
    police_number = "add some other number"

    while True:
        button.wait_for_press()
        press_times.append(time())

        # Keep only presses in the last 2 seconds
        press_times = [t for t in press_times if time() - t < 2]

        if len(press_times) >= 2:
            screen_message("sending message to police", "top")
            sleep(1)
            screen_message("sms sent", "bottom")
            press_times = []  # reset after sending

        sleep(0.1)  # debounce delay

        
        
def demo_mode():
    screen_message("Demo Mode", "center")        
    button = Button(2)                      #gpio 2
    led = LED(17)                           #gpio 17
    heartbeat = 100
    location = get_location()
    emergency_number = "+917717619230"
    emergency = False

    sensor = MAX30100Sensor()
    ir, red = sensor.read_heartbeat()
    print("IR:", ir, "RED:", red)

    
    while True:
        if heartbeat < 60 or heartbeat > 120:
            screen_message("WARNING!!", "top")
            screen_message("are you ok?", "center")
            sleep(0.1)
                
            start_time = time.time()
            emergency = True  
              
            while (time.time() - start_time) < 5:
                if button_pressed:               
                    emergency = False
                    heartbeat = 100
                    break                        
                else:                            
                    sleep(0.1)
                    blink_led_and_vibration_threading(1)
                    led.off()
                    
               #handle emergency 
            if emergency:
                screen_message("no response dtected", "top")
                sleep(1)
                screen_message("sending emergency alert", "center")
                sleep(1)
                screen_message("message sent!!", "bottom")
                while True:
                    for _ in range(12):
                        blink_led_and_vibration_threading(1)
                        screen_message("keep breaathing", "top")
                        screen_message("help is comming", "center")
                        sleep(5)
                        
                    #send location every minute
                    send_sms(emergency_number, f"latest location: {location}")
            else:
                screen_message("false alarm", "center")
                sleep(2)
                screen_message("resetting!!!", "bottom")
                sleep(10)
                continue
        
        #show health
        else:
            screen_message("health ok", "center")
            screen_message(f"heartbeat: {heartbeat}", "bottom")
            heartbeat -= 5
            sleep(0.1)

if __name__ == "__main__":
    demo_mode()

#             main mode            
def main():
    screen_message("Production mode", "center")
    button = Button(2)
    led = LED(17)
    heartbeat = 100           #remove this when the module will get connected !!!!!!!!!!
    gps_location = "gmaps location"
    
    
    condition_met = threading.Event()
    
    while True:
        if heartbeat < 60 or heartbeat > 120:
            screen_message("abnormal activity", "top")
            screen_message("press button for false alarm", "center")
        
            alert_cancelled = send_sms_if_button_not_pressed(button, heartbeat, gps_location)
            
            if alert_cancelled:
                sleep(60)
            else:
                sleep(300)
                
        else:
            sleep(0.5)
