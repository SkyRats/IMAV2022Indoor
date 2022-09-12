#!/usr/bin/env python2

import rospy
from bebopbase import Bebopbase
from std_msgs.msg import Int64
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from mpl_toolkits.mplot3d.axes3d import get_test_data
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import csv
import random
import tf

class Map():

    def __init__(self):

        self.co2_sub = rospy.Subscriber('/jonas/CO2', Int64, self.co2_callback)
        self.sonar_sub = rospy.Subscriber('/jonas/sonar', Float32, self.sonar_callback)
        self.temp_sub = rospy.Subscriber('/jonas/temperature', Float32, self.temp_callback)
        self.local_sub = rospy.Subscriber('/bebop/odom', Odometry, self.local_callback)
        self.trajectory_sub = rospy.Subscriber('/trajectory/finish', Bool, self.trajectory_callback)

        self.trajectory = 0

        rospy.sleep(5)
        rospy.loginfo("Object Map() created!")

    def co2_callback(self, data):
        self.co2 = data

    def sonar_callback(self, data):
        self.sonar = data
    
    def temp_callback(self, data):
        self.temp = data

    def local_callback(self, data):
        try:
            position,quaternion = self.listener.lookupTransform('/base_link', '/odom', rospy.Time(0))
            self.pose = np.multiply(-1,position)
            [_,_,self.yaw] = tf.transformations.euler_from_quaternion(quaternion)
            self.yaw = np.multiply(-1,self.yaw)

            CONSTANTE = 1.8
            self.correct_pose_x = self.pose[0] * CONSTANTE
            self.correct_pose_y = self.pose[1] * CONSTANTE
            #print("YAW = : " + str(self.yaw))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass 
    
    def trajectory_callback(self, data):
        self.trajectory = data

    def createCSV(self):
        #create CSV file with titles

        header = ["PoseX", "PoseY", "CO2", "Temperature"]
        with open('/home/gabiyuri/skyrats_ws2/src/indoor22/auto.csv', 'w', encoding='UTF8') as f:
            writer = csv.writer(f)
            writer.writerow(header)

        rospy.loginfo("CSV created!")
        return("1")
    
    def addtoCSV(self):
        #save data to the CSV file

        data = [self.correct_pose_x, self.correct_pose_y, self.co2, self.temp]
        with open('/home/gabiyuri/skyrats_ws2/src/indoor22/auto.csv', 'a', encoding='UTF8') as f:
            writer = csv.writer(f)
            writer.writerow(data)
        return("1")
        
    def createGraph(self):
        #create the graph with the CSV data

        fig = plt.figure(figsize=plt.figaspect(0.5))

        ax = fig.add_subplot(1, 2, 1, projection='3d')

        columnsCO2 = ["PoseX", "PoseY", "CO2", "Temperature"]
        df = pd.read_csv("/home/gabiyuri/skyrats_ws2/src/indoor22/auto.csv", usecols=columnsCO2)
        print("Contents in csv file:\n", df)

        ax.plot_trisurf(df.PoseX, df.PoseY, df.CO2, cmap='viridis')
        ax.set_zlim(350, 450)
        ax.set_title('CO2')

        ax = fig.add_subplot(1, 2, 2, projection='3d')

        ax.plot_trisurf(df.PoseX, df.PoseY, df.Temperature, cmap='viridis')
        ax.set_zlim(10, 50)
        ax.set_title('Temperature')

        plt.show()
        pass

    def run(self):
        self.createCSV()
        while not self.trajectory:
            self.addtoCSV()

        self.createGraph()

if __name__ == '__main__':
    rospy.init_node("map")
    map = Map()
    map.run()
