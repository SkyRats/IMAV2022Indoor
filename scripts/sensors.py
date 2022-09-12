#!/usr/bin/env python2

import rospy
import serial
import re
import time

from std_msgs.msg import Int64, Float32

class Sensors():

    def __init__(self):
        
        self.rate = rospy.Rate(60)

        self.co2_pub = rospy.Publisher( '/jonas/CO2', Int64, queue_size = 15,  latch=True)
        self.sonar_pub = rospy.Publisher( '/jonas/sonar', Int64, queue_size = 15,  latch=True)
        self.temp_pub = rospy.Publisher( '/jonas/temperature', Int64, queue_size = 15,  latch=True)

        time.sleep(2)
        rospy.loginfo("Jonas ready!")

    def run(self):

        ser = serial.Serial('/dev/ttyUSB0')
        pattern = r'[0-9]'

        while not rospy.is_shutdown():
            linha = str(ser.readline())
            if (linha[0] == "D"):
                #distancia
                d = Int64()
                d = convert(re.fidall(pattern, linha))
                self.sonar_pub.publish(d)

            elif (linha[0] == "C"):
                #co2
                c = Int64()
                c = convert(re.fidall(pattern, linha))
                self.co2_pub.publish(c)
            
            elif (linha[0] == "T"):
                #temperature
                t = Int64()
                t = convert(re.fidall(pattern, linha))
                self.sonar_pub.publish(t)

        ser.close()

def convert(list):
    return int("".join(map(str, list)))

if __name__ == "__main__":
    rospy.init_node('sensors_jonas')
    sensors = Sensors()
    sensors.run()
