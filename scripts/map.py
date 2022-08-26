#!/usr/bin/env python3

import rospy
from mavbase2.MAV2 import MAV2
from IndoorSensors import CSS, US100

from mpl_toolkits.mplot3d.axes3d import get_test_data
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import csv
import random

class Map():

    def __init__(self, mav):
        self.mav = mav

    def createCSV(self):
        header = ["PoseX", "PoseY", "CO2", "Temperature"]
        with open('/home/gabiyuri/skyrats_ws2/src/indoor22/auto.csv', 'w', encoding='UTF8') as f:
            writer = csv.writer(f)
            writer.writerow(header)
        return("1")
    
    def addtoCSV(self, co2, temp):
        data = [mav.drone_pose.pose.position.x, mav.drone_pose.pose.position.y, co2, temp]
        with open('/home/gabiyuri/skyrats_ws2/src/indoor22/auto.csv', 'a', encoding='UTF8') as f:
            writer = csv.writer(f)
            writer.writerow(data)
        return("1")
        
    def createGraph():
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

if __name__ == '__main__':
    rospy.init_node("map")
    mav = MAV2()
    map = Map(mav)
    map.createCSV()
    for i in range(100):
        co2 = CSS().retornaValorCO2()
        dist, temp = US100().medicaoUS100()
        map.addtoCSV(co2, temp)
    map.createGraph()
