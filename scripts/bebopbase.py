#!/usr/bin/env python2
import rospy
import math 
from std_msgs.msg import Empty
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import tf
import numpy as np
import cv2
from cv_bridge import CvBridge

PI = math.pi
FRENTE =-2.9
TRAZ = ((FRENTE + 2*PI) % (2*PI) ) - PI 
ESQUERDA = ((FRENTE + 1.5 * PI) % (2 * PI)) - PI  
DIREITA =  ((FRENTE + 0.5 * PI) % (2 * PI)) - PI
HEIGHT = 1
FIRST_GOING =2
ROW_DISTANCE = 2.8
ROW_WIDTH = 6
ROWS = 12
#Rotacao = (self.yaw + PI + GOAL) % (2*PI) - PI

class Bebopbase():

    def __init__(self, namespace="bebop"):

        self.Hz = 60
        self.rate = rospy.Rate(self.Hz)
        self.empty = Empty()
        self.bridge_object = CvBridge()
        self.takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size = 5, latch=True)
        self.land_pub = rospy.Publisher( '/' + namespace + '/land', Empty, queue_size = 50, latch=True)
        self.kill_pub = rospy.Publisher( '/' + namespace + '/reset', Empty, queue_size = 50,  latch=True)
        self.vel_pub = rospy.Publisher( '/' + namespace + '/cmd_vel', Twist, queue_size = 50, latch=True)
        self.camera_pub = rospy.Publisher( '/' + namespace + '/camera_control', Twist, queue_size = 50,  latch=True)

        self.local_sub = rospy.Subscriber('/bebop/odom', Odometry, self.local_callback)
        self.camera_sub = rospy.Subscriber('/bebop/image_raw', Image, self.image_callback)

        self.listener = tf.TransformListener() # position tf listener

        rospy.sleep(3.)
        rospy.wait_for_message('/bebop/odom', Odometry)
        rospy.wait_for_message("/bebop/image_raw", Image)
        self.image_width = 856
        self.image_height = 480

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
        

    def image_callback(self, data):
        self.image = data
        self.cv_image = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8") 


    def takeoff(self):
        self.initial_yaw = self.yaw
        #print("-----------Initial: " + str(self.initial_yaw))
        rospy.loginfo("Takeoff")
        self.takeoff_pub.publish(self.empty)
        
        rospy.sleep(5.)


    def land(self):
        rospy.loginfo("Land")
        self.land_pub.publish(self.empty)


    def kill(self):
        self.kill_pub.publish(self.empty)


    def set_vel(self, x, y, z, yaw):
        vel = Twist()
        vel.linear.x = x
        vel.linear.y = y
        vel.linear.z = z
        vel.angular.z = yaw
        self.vel_pub.publish(vel)

    def camera_control(self, horizontal, vertical):
        vel_cam = Twist()
        vel_cam.angular.y = vertical
        vel_cam.angular.z = horizontal
        self.camera_pub.publish(vel_cam)

    

    def set_position(self, x, y, z, tolerance= 0.2):
        VEL_MAX_Z = 0.2
        VEL_MAX = 0.08

        self.P = 0.05 
        self.I = 0.0035
        self.D = 0.09

        integral_prior_x = 0
        error_prior_x = 0
        
        integral_prior_y = 0
        error_prior_y = 0     
        
        integral_prior_z = 0
        error_prior_z = 0
        self.iteration_time = 1.0/60
        while z - self.pose[2] > tolerance*2 and not rospy.is_shutdown():
            delta_z =  float(z - self.pose[2] )
            integral_z = integral_prior_z + delta_z * self.iteration_time
            derivative_z = (delta_z - error_prior_z) / self.iteration_time
            vel_z = self.P * delta_z + self.I * integral_z + self.D * derivative_z 
            error_prior_z = delta_z
            integral_prior_z = integral_z

            if abs(vel_z) > VEL_MAX_Z:
                if vel_z > VEL_MAX_Z:
                    vel_z = VEL_MAX_Z
                else: 
                    vel_z = -VEL_MAX_Z

            self.set_vel(0, 0, vel_z, 0)
            self.rate.sleep()


        self.set_vel(0, 0, 0, 0)
        cont_loops = 0
        cont_loops_ativos = 0
        ultimo_contador = 0
        self.iteration_time = 1.0/60

        print(np.sqrt((x - self.correct_pose_x)**2 + (y -self.correct_pose_y)**2))
        while np.sqrt((x - self.correct_pose_x)**2 + (y -self.correct_pose_y)**2)  > tolerance and not rospy.is_shutdown():

        #while y - self.correct_pose_y > tolerance and not rospy.is_shutdown():
           
            cont_loops = cont_loops + 1
            delta_x =  float(x - self.correct_pose_x)
            delta_y =  float(y - self.correct_pose_y )

            integral_x = integral_prior_x + delta_x * self.iteration_time
            derivative_x = (delta_x - error_prior_x) / self.iteration_time

            integral_y = integral_prior_y + delta_y * self.iteration_time
            derivative_y = (delta_y - error_prior_y) / self.iteration_time

            if(derivative_y != 0.0): 
                self.iteration_time = (cont_loops - ultimo_contador)* (1.0/self.Hz)
                ultimo_contador = cont_loops

                # IMPLEMENTAR CONTROLE


                print("P= " + str(self.P * delta_y))
                print("I = " +str( self.I * integral_y))
                print("D = " + str( self.D * derivative_y ))

                # Centralization PID
                vel_x = self.P * delta_x + self.I * integral_x + self.D * derivative_x 
                vel_y = self.P * delta_y + self.I * integral_y + self.D * derivative_y 
                

                error_prior_x = delta_x
                error_prior_y = delta_y

                integral_prior_x = integral_x
                integral_prior_y = integral_y


                if abs(vel_x) > VEL_MAX:
                    if vel_x > VEL_MAX:
                        vel_x = VEL_MAX
                    else: 
                        vel_x = -VEL_MAX
            
                if abs(vel_y) > VEL_MAX:
                    if vel_y > VEL_MAX:
                        vel_y = VEL_MAX
                    else: 
                        vel_y = -VEL_MAX
                        

                print("position_atual = " + str(self.correct_pose_x) + " , " +  str(self.correct_pose_y) )
                print("vel_x = " + str(vel_x))
                print("vel_y = " + str(vel_y))
            self.rate.sleep()

            self.set_vel(vel_x, vel_y, 0,0)
        print("position_atual = " + str(self.correct_pose_x) + " , " +  str(self.correct_pose_y) )
        self.set_vel(0, 0, 0, 0)


    def rotate(self,desired_yaw, tolerance=0.01):
        yaw_diff = tolerance + 1
        while abs(yaw_diff) >= tolerance and not rospy.is_shutdown():
            if self.yaw*desired_yaw >= 0:
                yaw_diff = self.yaw - desired_yaw
            elif self.yaw >= 0:
                yaw_diff = (abs(self.yaw) + abs(desired_yaw)) - 2*np.pi
            elif self.yaw <= 0:
                yaw_diff = 2*np.pi - (abs(self.yaw) + abs(desired_yaw))

            # IMPLEMENTAR CONTROLE
            bebop.set_vel(0.0,0.0,0.0, -0.7*yaw_diff)

    def set_yaw(self, desired_yaw, tolerance=0.01):
        P = desired_yaw - self.yaw
        if(P  > PI):
            P = -2*PI + P
        elif (P < -PI):
            P = 2*PI + P
        while abs(P) >= tolerance and not rospy.is_shutdown():
            bebop.set_vel(0.0,0.0,0.0,0.7*P)
            P = desired_yaw - self.yaw
            if(P  > PI):
                P = -2*PI + P
            elif (P < -PI):
                P = 2*PI + P
            print("desired: " + str(desired_yaw))
            print("Yaw: " + str(self.yaw))
            print("P: " + str(P))


    def fix_trajectory(self):
        self.set_yaw(FRENTE)
        self.set_position(0, 0, HEIGHT)
        self.set_position(0, FIRST_GOING, HEIGHT) 

        going = ROWS/2
        for i in range(int(2*going)):
            if (i < going):
                self.set_position(-ROW_WIDTH, FIRST_GOING + i*ROW_DISTANCE, HEIGHT)
                set_yaw(TRAZ)
                self.set_position(-ROW_WIDTH, FIRST_GOING + i*ROW_DISTANCE, HEIGHT)

                self.set_position(0, FIRST_GOING + i*ROW_DISTANCE, HEIGHT)

                if(i != going-1):
                    self.set_yaw(FRENTE)
                    self.set_position(0, FIRST_GOING + i*ROW_DISTANCE, HEIGHT)
                    self.set_position(0, FIRST_GOING + (i+1)*ROW_DISTANCE, HEIGHT)
                else:
                    self.set_position(0, FIRST_GOING + i*ROW_DISTANCE, HEIGHT)

            else:
                self.set_position(ROW_WIDTH, FIRST_GOING + (2*going-(i+1))*ROW_DISTANCE, HEIGHT)
                self.set_yaw(TRAZ)
                
                self.set_position(0, FIRST_GOING + (2*going-(i+1))*ROW_DISTANCE, HEIGHT)
                self.set_yaw(FRENTE)

                if(i != 2*going-1):
                    self.set_position(0, FIRST_GOING + ((2*going-(i+2))*ROW_DISTANCE), HEIGHT)
                else:
                    self.set_position(0, 0, HEIGHT)
            
        self.land()

    def save_image(self):
        cv2.imwrite('image5.png', self.cv_image)

    def save_image1(self):
        cv2.imwrite('image6.png', self.cv_image)

if __name__ == "__main__":
    rospy.init_node('bebopbase')
    bebop = Bebopbase()
    bebop.takeoff()
    #bebop.set_yaw(DIREITA)
    #bebop.set_yaw(ESQUERDA)
    #bebop.set_yaw(FRENTE)

    #bebop.set_position(0.5,0,1)
    #bebop.set_yaw(ESQUERDA)
    #bebop.set_position(0.5,-0.5,1)
    bebop.camera_control(0, -90)
    rospy.sleep(5)
    bebop.save_image()
    bebop.set_position(1.5,0,1)
    bebop.save_image1()
    '''print(FRENTE)
    print(ESQUERDA)
    print(TRAZ)
    print(DIREITA)

    bebop.camera_control(0, -90)
    #print("a")

    #bebop.set_yaw(FRENTE)
    #bebop.set_yaw(ESQUERDA)
    #bebop.set_yaw(DIREITA)'''

    bebop.land()

