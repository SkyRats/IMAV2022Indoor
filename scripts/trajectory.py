#!/usr/bin/env python3
import numpy as np
import rospy
import time
import sys
import cv2
import os
from mavbase2.MAV2 import MAV2
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from std_msgs.msg import Float64


   
HEIGHT = 1
FIRST_GOING =2
ROW_DISTANCE = 2.8
ROW_WIDTH = 6
DISTANCE_DRONES = 0 
VEL = 0.7
ROWS = 12
TOL = 0.5

FRENTE = np.pi/2
ESQUERDA = np.pi
TRAZ = -np.pi/2

class trajectory():
    def __init__(self, mav):
        self.mav = mav

        self.laser_sub = rospy.Subscriber('/laser/scan', LaserScan, self.laser_callback)
        self.compass_sub = rospy.Subscriber( '/mavros/global_position/compass_hdg',Float64, self.compass_callback)

    def laser_callback(self, data):
        self.sonar = data.ranges[0]
        print(self.sonar)
        
    def compass_callback(self, data):
        self.compass = data.data

    def fix_trajectory(self):
        self.mav.takeoff(2)
        self.mav.go_to_local(0, 0, HEIGHT, yaw = np.pi/2)
        self.mav.go_to_local(0, FIRST_GOING, HEIGHT) 

        going = ROWS/2
        for i in range(int(2*going)):
            if (i < going):
                self.mav.go_to_local(-ROW_WIDTH, FIRST_GOING + i*ROW_DISTANCE, HEIGHT)
                self.mav.go_to_local(-ROW_WIDTH, FIRST_GOING + i*ROW_DISTANCE, HEIGHT, yaw = -np.pi/2)

                self.mav.go_to_local(0, FIRST_GOING + i*ROW_DISTANCE, HEIGHT)

                if(i != going-1):
                    self.mav.go_to_local(0, FIRST_GOING + i*ROW_DISTANCE, HEIGHT, yaw = np.pi/2)
                    self.mav.go_to_local(0, FIRST_GOING + (i+1)*ROW_DISTANCE, HEIGHT)
                else:
                    self.mav.go_to_local(DISTANCE_DRONES, FIRST_GOING + i*ROW_DISTANCE, HEIGHT)

            else:
                self.mav.go_to_local(DISTANCE_DRONES + ROW_WIDTH, FIRST_GOING + (2*going-(i+1))*ROW_DISTANCE, HEIGHT)
                self.mav.go_to_local(DISTANCE_DRONES + ROW_WIDTH, FIRST_GOING + (2*going-(i+1))*ROW_DISTANCE, HEIGHT, yaw = np.pi/2)
                
                self.mav.go_to_local(DISTANCE_DRONES, FIRST_GOING + (2*going-(i+1))*ROW_DISTANCE, HEIGHT)
                self.mav.go_to_local(DISTANCE_DRONES, FIRST_GOING + (2*going-(i+1))*ROW_DISTANCE, HEIGHT, yaw = -np.pi/2)

                if(i != 2*going-1):
                    self.mav.go_to_local(DISTANCE_DRONES, FIRST_GOING + ((2*going-(i+2))*ROW_DISTANCE), HEIGHT)
                else:
                    self.mav.go_to_local(DISTANCE_DRONES, 0, HEIGHT)
            
        self.mav.land()


    def sonar_trajectory(self):
        print("Iniciando trajetoria com sonar e centralizacao nos vermelhos!")

        self.mav.takeoff(HEIGHT)
        self.mav.go_to_local(0, 0, HEIGHT, TOL = 0.05)
        self.mav.go_to_local(0, FIRST_GOING, HEIGHT) 

        self.going = ROWS/2
        
        for i in range(int(2 * self.going)):
            current_y = self.mav.drone_pose.pose.position.y

            if(i < self.going):
                print("Virando para o corredor da esquerda")
                self.mav.go_to_local(0, current_y, HEIGHT, yaw = TRAZ, TOL = 0.05) 
            else:
                print("Virando para o corredor da direita")
                self.mav.go_to_local(0, current_y, HEIGHT, yaw = FRENTE, TOL = 0.05) 
            self.align = 0
            while(self.align == 0) and not rospy.is_shutdown():
                print("Centralizando no vermelho")
                x_goal = self.center_red() 
                print("PID")
                if i < self.going:
                    signal = 1
                else:
                    signal = -1
                self.camera_pid(x_goal, signal)

            current_y = self.mav.drone_pose.pose.position.y            
            if(i < self.going):
                self.mav.go_to_local(0, current_y, HEIGHT, yaw = ESQUERDA) 
            else:
                self.mav.go_to_local(0, current_y, HEIGHT, yaw = -ESQUERDA) 

            while not rospy.is_shutdown() and abs(self.sonar - 6) > TOL :
                if(i < self.going):
                    self.mav.set_vel(-VEL, 0,0)
                else:
                    self.mav.set_vel(VEL, 0,0)

            self.mav.set_vel(0, 0,0)
            current_x = self.mav.drone_pose.pose.position.x
            current_y = self.mav.drone_pose.pose.position.y
            if(i<self.going):
                self.mav.go_to_local(current_x, current_y, HEIGHT, yaw = -ESQUERDA)

            else:
                self.mav.go_to_local(current_x, current_y, HEIGHT, yaw = ESQUERDA)

        
           
               
            while not rospy.is_shutdown() and abs(current_x - 0) > TOL :
                print(current_x)
                if(i<self.going):
                    self.mav.set_vel(VEL, 0,0)

                else:
                    self.mav.set_vel(-VEL, 0,0)
            self.mav.set_vel(0, 0,0)
            
            if (i < self.going-1):
                self.mav.go_to_local(0, FIRST_GOING + (i+1)*ROW_DISTANCE, HEIGHT, yaw = ESQUERDA)
            elif (i == (2*self.going)-1):
                self.mav.go_to_local(DISTANCE_DRONES, 0, HEIGHT, TOL = 0.1)
            elif (i > self.going-1):
                self.mav.go_to_local(0, FIRST_GOING + (2*self.going-(i+2))*ROW_DISTANCE, HEIGHT, yaw = -ESQUERDA)
            elif (i == self.going - 1):
                current_y = self.mav.drone_pose.pose.position.y
                self.mav.go_to_local(0, current_y, HEIGHT, yaw = -ESQUERDA)


        self.mav.land()
        
    def center_red(self):
        img = self.mav.cam
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(img , (0, 230, 200), (20, 255, 255))

        self.height = mask.shape[0]
        self.width = mask.shape[1]
        

        contours, heirarchies = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        mask = np.zeros(mask.shape[:2], dtype='uint8')
        
        cv2.drawContours(mask, contours, -1, (255, 0, 0), 1)
        
        
        cv2.imwrite("Contours.png", mask)

        centerx = np.empty(2)
        cont = 0
        for i in contours:
            M = cv2.moments(i)
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                area = cv2.contourArea(i)
                if area > 200:
                    
                    centerx[cont] = cx
                    cont = cont + 1

                    print(f"x: {centerx[0]}")
                    print(f"x: {centerx[1]}")


        x_goal = ( centerx[1] - centerx[0] ) / 2 + centerx[0]


        return x_goal

    def camera_pid(self, x_goal, signal):
        self.TARGET = int(self.width/2)
        delta_x = self.TARGET - x_goal

        self.TOL = 0.07
        self.PID = 2/2000

        # Centralization PID
        vel_y = delta_x * self.PID

        self.align = 0

        if abs(vel_y) < self.TOL:
            vel_y = 0.0
            self.row_y = self.mav.drone_pose.pose.position.y
            self.align = 1
        
      
        self.mav.set_vel(0, signal *vel_y, 0)
  
              
        #self.get_logger().info(" erro: " + str(delta_x))  
        #self.get_logger().info(" vely: " + str(vel_y))

    def rotate_to(self, degree):
        current_x = self.mav.drone_pose.pose.position.x
        current_y = self.mav.drone_pose.pose.position.y
        while not rospy.is_shutdown() and abs((self.compass - degree))>0.2:
            print(self.compass)


            self.mav.set_vel(0,0,0,yaw=0.1)
        self.mav.set_vel(0,0,0,0)
        self.mav.go_to_local(current_x, current_y, HEIGHT)



if __name__ == "__main__":
    rospy.init_node('trajectory')
    mav = MAV2()
    missao = trajectory(mav)

    missao.sonar_trajectory()
