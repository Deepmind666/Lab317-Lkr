import rospy
import Jetson.GPIO as GPIO

if __name__ == "__main__":
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(15, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)  # GPIO1  GPIO.PUD_DOWN
    GPIO.setup(33, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)  # GPIO2  GPIO.PUD_DOWN

    while True:
        if GPIO.input(15) == GPIO.HIGH:
            print("GPIO1 High")
            GPIO.setup(15, GPIO.OUT)
            GPIO.setup(15, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(15, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)  # GPIO1  GPIO.PUD_DOWN
        else:
            print("GPIO1 LOW")
        if GPIO.input(33) == GPIO.HIGH:
            print("GPIO2 High")
            GPIO.setup(33, GPIO.OUT)
            GPIO.setup(33, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(33, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)  # GPIO1  GPIO.PUD_DOWN
        else:
            print("GPIO2 LOW")