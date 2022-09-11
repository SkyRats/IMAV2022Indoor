#!/usr/bin/env python2
import rospy
from std_msgs.msg import Int64
from std_msgs.msg import Float32
import time
import RPi.GPIO as GPIO
trig = 22 # 17 Fisico
echo = 12 # 21 Fisico


class Sensors():

    def __init__(self):
        self.rate = rospy.Rate(60)
    
        self.co2_pub = rospy.Publisher( '/jonas/CO2', Int64, queue_size = 15,  latch=True)
        self.sonar_pub = rospy.Publisher( '/jonas/sonar', Float32, queue_size = 15,  latch=True)
        self.temperature_pub = rospy.Publisher( '/jonas/temperature', Float32, queue_size = 15,  latch=True)   

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(echo, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(trig, GPIO.OUT)
        GPIO.output(trig, GPIO.LOW)
        time.sleep(2)
        print("setup done")

    def ultrassonico(self):
        sonar = Float32()
        GPIO.output(trig, GPIO.LOW)
        time.sleep(0.000002)
        GPIO.output(trig, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(trig, GPIO.LOW)
        time.sleep(0.00003)
        inicio = time.time()
        while (GPIO.input(echo) == GPIO.LOW) and (time.time()-inicio< 0.5):
            pass
        inicio = time.time()
        while (GPIO.input(echo) == GPIO.HIGH): #and (time.time()-inicio < 0.5):
            pass
        tempo = time.time() - inicio
        d = tempo * 17150
        d = round(d + 1.15, 2)

        sonar = d 
        self.sonar_pub.publish(sonar)

        
   
    def co2(self):
        co2 = Int64()

        concentracao = 5
        co2 = concentracao 
        self.co2_pub.publish(co2)

    def temperature(self):
        temp = Float32()

        temperature = 2.2

        temp = temperature 
        self.temperature_pub.publish(temperature)

    def run(self):
        while not rospy.is_shutdown():
            self.ultrassonico()
            self.co2()
            self.temperature()
            self.rate.sleep()

        

if __name__ == "__main__":
    print("a")
    rospy.init_node('sensors')
    print("a")
    sensors = Sensors()
    sensors.run()
   