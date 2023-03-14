#!/usr/bin/python3
import RPi.GPIO as GPIO
import time

class Stepper:
    def __init__(self, out1, out2, out3, out4):
        self.out1 = out1
        self.out2 = out2
        self.out3 = out3
        self.out4 = out4

        self.step_sleep = 0.002 # Time between steps, cant be too low due to the motors physical limitations 
        self.step = 0 # absolute count of steps in either direction (left = negative, right = positive)
        self.max_steps = 360 / 1.8 # 1.8 degrees per step, 200 steps for full rotation with  # To account for direction ("Splits" rotation into 2 parts [-,+] for left and right)
        self.orientation = 0 # relative count of steps representing the orientation of the motor

       
    def __setup__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.out1, GPIO.OUT)
        GPIO.setup(self.out2, GPIO.OUT)
        GPIO.setup(self.out3, GPIO.OUT)
        GPIO.setup(self.out4, GPIO.OUT)

        GPIO.output(self.out1, GPIO.LOW)
        GPIO.output(self.out2, GPIO.LOW)
        GPIO.output(self.out3, GPIO.LOW)
        GPIO.output(self.out4, GPIO.LOW)


    def __cleanup__(self):
        GPIO.output(self.out1, GPIO.LOW )
        GPIO.output(self.out2, GPIO.LOW )
        GPIO.output(self.out3, GPIO.LOW )
        GPIO.output(self.out4, GPIO.LOW )
        GPIO.cleanup()

    def left_turn(self, steps : int):
        self._setup_()
        for i in range(0, steps):
            if i%4==0:
                GPIO.output(self.out4, GPIO.LOW)
                GPIO.output(self.out3, GPIO.LOW)
                GPIO.output(self.out2, GPIO.LOW)
                GPIO.output(self.out1, GPIO.HIGH)
            elif i%4==1:
                GPIO.output(self.out4, GPIO.LOW)
                GPIO.output(self.out3, GPIO.HIGH)
                GPIO.output(self.out2, GPIO.LOW)
                GPIO.output(self.out1, GPIO.LOW)
            elif i%4==2:
                GPIO.output(self.out4, GPIO.LOW)
                GPIO.output(self.out3, GPIO.LOW)
                GPIO.output(self.out2, GPIO.HIGH)
                GPIO.output(self.out1, GPIO.LOW)
            elif i%4==3:
                GPIO.output(self.out4, GPIO.HIGH)
                GPIO.output(self.out3, GPIO.LOW)
                GPIO.output(self.out2, GPIO.LOW)
                GPIO.output(self.out1, GPIO.LOW)
    
            time.sleep(self.step_sleep)

        self.step -= steps
        self.orientation = abs(self.step % self.max_steps)
        self.__cleanup__()
 
    def right_turn(self, steps : int):
        self.__setup__()
        for i in range(0, steps):
            if i%4==0:
                GPIO.output(self.out4, GPIO.HIGH)
                GPIO.output(self.out3, GPIO.LOW)
                GPIO.output(self.out2, GPIO.LOW)
                GPIO.output(self.out1, GPIO.LOW)
            elif i%4==1:
                GPIO.output(self.out4, GPIO.LOW)
                GPIO.output(self.out3, GPIO.LOW)
                GPIO.output(self.out2, GPIO.HIGH)
                GPIO.output(self.out1, GPIO.LOW)
            elif i%4==2:
                GPIO.output(self.out4, GPIO.LOW)
                GPIO.output(self.out3, GPIO.HIGH)
                GPIO.output(self.out2, GPIO.LOW)
                GPIO.output(self.out1, GPIO.LOW)
            elif i%4==3:
                GPIO.output(self.out4, GPIO.LOW)
                GPIO.output(self.out3, GPIO.LOW)
                GPIO.output(self.out2, GPIO.LOW)
                GPIO.output(self.out1, GPIO.HIGH)

            time.sleep(self.step_sleep)

        self.step += steps    
        self.orientation = abs(self.step % self.max_steps)
        self.__cleanup__()
        
    def home(self):
        while(self.step != 0):
            if self.step > 0:
                self.left_turn(1)
            else:
                self.right_turn(1)

    #Just for testing purposes, automated via switch later on
    def set_home(self):
        self.step = 0