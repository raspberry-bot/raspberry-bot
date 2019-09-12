import time
import RPi.GPIO as GPIO
from threading import Thread

class SRF05_sensor(Thread):

    """Code for HY-SRF05 and HC-SRF04

    The HY-SRF05 is an evolution of the HC-SRF04
    There is just an additional pin, if not use
    the sensor is like the previous version, that's
    the reason why the code can run on both device

    We use BCM mode, the pin are identified with
    the GPIO number.

    Parameters
    ----------

    TRIG_PIN : GPIO Number of the pin use for the
    trig signal

    ECHO_PIN : GPIO Number of the pin use for the
    echo signal

    sensor_name : Use to identify each sensor

    """
    def __init__(self,TRIG_PIN,ECHO_PIN,sensor_name):
        Thread.__init__(self)
        self.TRIG_PIN= TRIG_PIN
        self.ECHO_PIN = ECHO_PIN
        self.sensor_name = sensor_name
        self.sonic_speed = 34300

    def setup_gpio(self):
        """Setup I/O GPIO"""

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.TRIG_PIN,GPIO.OUT)
        GPIO.setup(self.ECHO_PIN,GPIO.IN)

    def compute_distance(self):

        GPIO.output(self.TRIG_PIN,True)
        time.sleep(0.00001)
        GPIO.output(self.TRIG_PIN,False)

        while GPIO.input(self.ECHO_PIN) == 0:
            start = time.time()

        while GPIO.input(self.ECHO_PIN) == 1:
            end = time.time()

        duration = end - start

        # We multiply with the sonic speed divide by 2
        # Because of the wave there and back
        self.distance = duration * self.sonic_speed/2
        print("sensor %s " %self.sensor_name, self.distance)

    def get_distance(self):
        self.compute_distance()
	    return self.distance


if __name__ == '__main__':

    sensor_1 = SRF05_sensor(23, 24, "DistanceMeter")
    sensor_1.start()
    sensor_1.setup_gpio()

    try :
        while True :
            d1 = sensor_1.get_distance()
            print(d1)
                time.sleep(0.5)
            sensor_1.join()

    except KeyboardInterrupt:
        print("Stop measurement")
        GPIO.cleanup()