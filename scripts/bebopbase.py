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


PI = math.pi
FRENTE =2.3 #2.73
class Bebopbase():

    def __init__(self, namespace="bebop"):

        self.Hz = 60
        self.rate = rospy.Rate(self.Hz)
        self.empty = Empty()

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

    '''def set_position(self, x, y, z, tolerance= 0.1):
        print(x,y,z)
        while np.sqrt((x - self.pose[0])**2 + (y - self.pose[1])**2 + (z - self.pose[2])**2) > tolerance:
            # IMPLEMENTAR CONTROLE
            self.set_vel(x - self.pose[0], y - self.pose[1], z - self.pose[2], 0)'''
    

    def set_position(self, x, y, z, yaw=FRENTE, tolerance= 0.2):
        VEL_MAX_Z = 0.2
        VEL_MAX = 0.05

        self.P = 0.025 
        self.I = 0.004
        self.D = 0.085

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

        yaw_diff = tolerance + 1
        while abs(yaw_diff) >= tolerance/10 and not rospy.is_shutdown():
            if self.yaw*yaw >= 0:
                yaw_diff = self.yaw - yaw
            elif self.yaw >= 0:
                yaw_diff = (abs(self.yaw) + abs(yaw)) - 2*np.pi
            elif self.yaw <= 0:
                yaw_diff = 2*np.pi - (abs(self.yaw) + abs(yaw))

            # IMPLEMENTAR CONTROLE
            bebop.set_vel(0.0,0.0,0.0, -0.7*yaw_diff)
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


    def set_yaw(self,desired_yaw, tolerance=0.01):
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
        

if __name__ == "__main__":
    rospy.init_node('bebopbase')
    bebop = Bebopbase()
    bebop.takeoff()
    #bebop.set_position(1,bebop.pose[1],bebop.pose[2])
    #bebop.set_yaw(np.pi, tolerance=0.01)
    #bebop.set_yaw(0, tolerance=0.01)
    ''' bebop.set_yaw(0, tolerance=0.01)

    rospy.sleep(2.)
    bebop.set_yaw(np.pi/2, tolerance=0.01)
    rospy.sleep(2.)
    bebop.set_yaw(np.pi, tolerance=0.01)
    rospy.sleep(2.)
    bebop.set_yaw(-np.pi/2, tolerance=0.01)
    rospy.sleep(2.)'''
    '''bebop.set_yaw(np.pi)
    rospy.sleep(3.)
    bebop.set_yaw(bebop.initial_yaw)
    rospy.sleep(3.)
    bebop.set_yaw(np.pi)
    rospy.sleep(3.)
    bebop.set_yaw(bebop.initial_yaw)
    rospy.sleep(3.)'''
    rospy.sleep(2.)
    bebop.set_position(0,3,1)
    rospy.sleep(2.)
    #bebop.set_position(0,0,1)
    #rospy.sleep(2.)
    #bebop.set_position(0,2,1)     
    #rospy.sleep(2.)
    #bebop.set_position(0,0,1)
    #rospy.sleep(2.)
    
    print(bebop.yaw) 




    #bebop.set_yaw(0, tolerance=0.005)

    bebop.land()

