#!/usr/bin/env python2
import rospy
import serial
import re
from std_msgs.msg import Int64
from std_msgs.msg import Float32
import time

class Sensors():
    def __init__(self):
        self.rate = rospy.Rate(60)
        self.co2_pub = rospy.Publisher( '/jonas/CO2', Int64, queue_size = 15,  latch=True)
        self.sonar_pub = rospy.Publisher( '/jonas/sonar', Float32, queue_size = 15,  latch=True)
        self.temperature_pub = rospy.Publisher( '/jonas/temperature', Float32, queue_size = 15,  latch=True)  
        time.sleep(2)
        print("setup done")

    def run(self):
        co2 = Int64()
        temp = Int64()
        dist = Int64()
        ser = serial.Serial('/dev/ttyUSB0') 
        pattern = r'[0-9]'
        while not rospy.is_shutdown():
            linha = str(ser.readline())
            if   (linha[2] == "D"): #DISTANCIA
                d = convert(re.findall(pattern, linha))
                self.sonar_pub.publish(d)
            elif (linha[2] == "C"): #CO2
                c = convert(re.findall(pattern, linha))
                self.co2_pub.publish(c)
            elif (linha[2] == "T"): #TEMPERATURA
                t = convert(re.findall(pattern, linha))
                self.temp_pub.publish(t)
            rospy.sleep()   
        ser.close()

def convert(list):
    return int("".join(map(str, list)))  

if __name__ == "__main__":
    rospy.init_node('sensors_jonas')
    sensors = Sensors()
    sensors.run()
   