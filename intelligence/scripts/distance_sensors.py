# Libraries
import RPi.GPIO as GPIO
import time


class DistanceSensor:
    def __init__(self):
        # GPIO Mode (BOARD / BCM)
        GPIO.setmode(GPIO.BOARD)

        # set GPIO Pins
        self.GPIO_ECHO = 29
        self.GPIO_TRIGGER = 31

        # set GPIO direction (IN / OUT)
        GPIO.setup(self.GPIO_TRIGGER, GPIO.OUT)
        GPIO.setup(self.GPIO_ECHO, GPIO.IN)

    def get_current_distance_to_obstacle(self):
        # set Trigger to HIGH
        GPIO.output(self.GPIO_TRIGGER, True)

        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(self.GPIO_TRIGGER, False)

        start_time = time.time()
        stop_time = time.time()

        # save StartTime
        while GPIO.input(self.GPIO_ECHO) == 0:
            start_time = time.time()

        # save time of arrival
        while GPIO.input(self.GPIO_ECHO) == 1:
            stop_time = time.time()

        # time difference between start and arrival
        time_elapsed = stop_time - start_time
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        distance = (time_elapsed * 34300) / 2

        return distance


if __name__ == '__main__':
    try:
        distance_sensor = DistanceSensor()
        while True:
            distance_sensor = DistanceSensor()
            print ("Measured Distance = %.1f cm" % distance_sensor.get_current_distance_to_obstacle())
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()