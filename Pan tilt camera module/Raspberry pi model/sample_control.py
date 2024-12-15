import gpio
import time

servo_pin = 13
gpio.setmode(gpio. BCM)
gpio.setup(servo_pin,gpio. OUT)
pwm = gpio.PWM(servo_pin,50)
pwm. start(7)

