# Import required Python libraries
import time
import RPi.GPIO as GPIO

# Use BCM GPIO references
# instead of physical pin numbers
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Define GPIO to use on Pi
GPIO_TRIGGER = 23
GPIO_ECHO    = 24

# Set pins as output and input
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)  # Trigger
GPIO.setup(GPIO_ECHO, GPIO.IN)      # Echo

def read():
    # Set trigger to False (Low)
    GPIO.output(GPIO_TRIGGER, False)

    print("Ultrasonic Measurement")

    # Allow module to settle
    time.sleep(0.5)

    # Send 10us pulse to trigger
    GPIO.output(GPIO_TRIGGER, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)

    # "The SRF05 will send out an 8 cycle burst of ultrasound at 40khz and raise its echo line high (or trigger line in mode 2)"
    # Wait no longer than 30ms
    if GPIO.wait_for_edge(self.echo_pin, GPIO.RISING, timeout=30) is None:
        return None

    start = time.time()

    # Measure pulse duration, again do not wait more than 30ms
    # "If nothing is detected then the SRF05 will lower its echo line anyway after about 30mS."
    if GPIO.wait_for_edge(self.echo_pin, GPIO.FALLING, timeout=30) is None:
        return None

    while GPIO.input(GPIO_ECHO)==1:
        stop = time.time()

    # Calculate pulse length
    elapsed = stop-start

    # Distance pulse travelled in that time is time
    # multiplied by the speed of sound (cm/s)
    distancet = elapsed * 34300

    # That was the distance there and back so halve the value
    distance = distancet / 2

    print("Distance :", distance)
    print("Elaspsed time :", elapsed)
    print("Total distance :", distancet)

    # Reset GPIO settings
    GPIO.cleanup()

while True:
    read()