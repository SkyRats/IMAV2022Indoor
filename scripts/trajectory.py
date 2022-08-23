#!/usr/bin/env python3
import numpy as np
import rclpy
import time
import sys
import cv2
import os
sys.path.insert(0,'/home/' + os.environ["USER"] + '/skyrats_ws2/src/mavbase2')
from MAV2 import MAV2
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from rclpy.node import Node
from rclpy import qos

   
HEIGHT = 1.2
FIRST_GOING =1.9
ROW_DISTANCE = 2.8
ROW_WIDTH = 1.5
DISTANCE_DRONES = 0 
VEL = 0.7
ROWS = 12
TOL = 0.5

class trajectory(Node):
    def __init__(self, mav):
        super().__init__('mav')
        self.mav = mav
        self.laser_sub = self.create_subscription(LaserScan, '/laser/scan', self.laser_callback, qos.qos_profile_sensor_data)
        self.compass_sub = self.create_subscription(Float64, '/mavros/global_position/compass_hdg', self.compass_callback, qos.qos_profile_sensor_data)

    def laser_callback(self, data):
        self.sonar = data.ranges[0]

    def compass_callback(self, data):
        self.compass = data.data

    def fix_trajectory(self):
        self.mav.takeoff(2)
        self.mav.go_to_local(0, 0, HEIGHT, yaw = np.pi/2, vel_xy = VEL)
        self.mav.go_to_local(0, FIRST_GOING, HEIGHT, vel_xy = VEL) 

        going = ROWS/2
        for i in range(int(2*going)):
            if (i < going):
                self.mav.go_to_local(-ROW_WIDTH, FIRST_GOING + i*ROW_DISTANCE, HEIGHT, vel_xy = VEL)
                self.mav.go_to_local(-ROW_WIDTH, FIRST_GOING + i*ROW_DISTANCE, HEIGHT, yaw = -np.pi/2, vel_xy = VEL)

                self.mav.go_to_local(0, FIRST_GOING + i*ROW_DISTANCE, HEIGHT, vel_xy = VEL)

                if(i != going-1):
                    self.mav.go_to_local(0, FIRST_GOING + i*ROW_DISTANCE, HEIGHT, yaw = np.pi/2, vel_xy = VEL)
                    self.mav.go_to_local(0, FIRST_GOING + (i+1)*ROW_DISTANCE, HEIGHT, vel_xy = VEL)
                else:
                    self.mav.go_to_local(DISTANCE_DRONES, FIRST_GOING + i*ROW_DISTANCE, HEIGHT, vel_xy = VEL)

            else:
                self.mav.go_to_local(DISTANCE_DRONES + ROW_WIDTH, FIRST_GOING + (2*going-(i+1))*ROW_DISTANCE, HEIGHT, vel_xy = VEL)
                self.mav.go_to_local(DISTANCE_DRONES + ROW_WIDTH, FIRST_GOING + (2*going-(i+1))*ROW_DISTANCE, HEIGHT, yaw = np.pi/2, vel_xy = VEL)
                
                self.mav.go_to_local(DISTANCE_DRONES, FIRST_GOING + (2*going-(i+1))*ROW_DISTANCE, HEIGHT, vel_xy = VEL)
                self.mav.go_to_local(DISTANCE_DRONES, FIRST_GOING + (2*going-(i+1))*ROW_DISTANCE, HEIGHT, yaw = -np.pi/2, vel_xy = VEL)

                if(i != 2*going-1):
                    self.mav.go_to_local(DISTANCE_DRONES, FIRST_GOING + ((2*going-(i+2))*ROW_DISTANCE), HEIGHT, vel_xy = VEL)
                else:
                    self.mav.go_to_local(DISTANCE_DRONES, 0, HEIGHT, vel_xy = VEL)
            
        self.mav.land()


    def sonar_trajectory(self):
        print("Iniciando trajetoria com sonar e centralizacao nos vermelhos!")
        for k in range(30):
            rclpy.spin_once(self)

        self.mav.takeoff(HEIGHT)
        self.mav.go_to_local(0, 0, HEIGHT, yaw = np.pi/2, vel_xy = VEL, TOL = 0.05)
        self.mav.go_to_local(0, FIRST_GOING, HEIGHT, vel_xy = VEL) 

        self.going = ROWS/2
        
        for i in range(int(2 * self.going)):
            rclpy.spin_once(self.mav)
            current_y = self.mav.drone_pose.pose.position.y

            if(i < self.going):
                print("Virando para o corredor da esquerda")
                self.mav.go_to_local(0, current_y, HEIGHT, vel_xy = VEL, yaw = np.pi, TOL = 0.05) 
            else:
                print("Virando para o corredor da direita")
                self.mav.go_to_local(0, current_y, HEIGHT, vel_xy = VEL, yaw = 0, TOL = 0.05) 
            self.align = 0
            while(self.align == 0):
                rclpy.spin_once(self)
                print("Centralizando no vermelho")
                x_goal = self.center_red() 
                print("PID")
                if i < self.going:
                    signal = -1
                else:
                    signal = 1
                self.camera_pid(x_goal, signal)

            current_y = self.mav.drone_pose.pose.position.y            
            if(i < self.going):
                self.mav.go_to_local(0, current_y, HEIGHT, vel_xy = VEL, yaw = np.pi/2) 
            else:
                self.mav.go_to_local(0, current_y, HEIGHT, vel_xy = VEL, yaw = -np.pi/2) 

            for j in range(15):
                rclpy.spin_once(self)   

            while abs(self.sonar - 3) > TOL :
                rclpy.spin_once(self)
                if(i < self.going):
                    self.mav.set_vel(-VEL, 0,0)
                else:
                    self.mav.set_vel(VEL, 0,0)

            self.mav.set_vel(0, 0,0)
            rclpy.spin_once(self.mav)
            current_x = self.mav.drone_pose.pose.position.x
            current_y = self.mav.drone_pose.pose.position.y
            if(i<self.going):
                self.mav.go_to_local(current_x, current_y, HEIGHT, yaw = -np.pi/2, vel_xy = VEL)

            else:
                self.mav.go_to_local(current_x, current_y, HEIGHT, yaw = np.pi/2, vel_xy = VEL)

        
           
               
            while abs(current_x - 0) > TOL :
                print(current_x)
                rclpy.spin_once(self.mav)
                if(i<self.going):
                    self.mav.set_vel(VEL, 0,0)

                else:
                    self.mav.set_vel(-VEL, 0,0)
                for i in range(15):
                    rclpy.spin_once(self.mav)
            self.mav.set_vel(0, 0,0)
            
            if (i < self.going-1):
                self.mav.go_to_local(0, FIRST_GOING + (i+1)*ROW_DISTANCE, HEIGHT, vel_xy = VEL, yaw = np.pi/2)
            elif (i == (2*self.going)-1):
                self.mav.go_to_local(DISTANCE_DRONES, 0, HEIGHT, vel_xy = VEL, TOL = 0.1)
            elif (i > self.going-1):
                self.mav.go_to_local(0, FIRST_GOING + (2*self.going-(i+2))*ROW_DISTANCE, HEIGHT, vel_xy = VEL, yaw = -np.pi/2)
            elif (i == self.going - 1):
                rclpy.spin_once(self.mav)
                current_y = self.mav.drone_pose.pose.position.y
                self.mav.go_to_local(0, current_y, HEIGHT, yaw = -np.pi/2, vel_xy = VEL)
           

            
            rclpy.spin_once(self)



        self.mav.land()
        
    def center_red(self):
        for j in range(100):
            rclpy.spin_once(self.mav)
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
        while abs((self.compass - degree))>0.2:
            print(self.compass)
            rclpy.spin_once(self)

            self.mav.set_vel(0,0,0,yaw=0.1)
        self.mav.set_vel(0,0,0,0)
        rclpy.spin_once(self)
        self.mav.go_to_local(current_x, current_y, HEIGHT, vel_xy = VEL)
        rclpy.spin_once(self)



if __name__ == "__main__":
    rclpy.init()
    mav = MAV2()
    missao = trajectory(mav)
    missao.sonar_trajectory()
