#!/usr/bin/env python2
import rospy
import math 
from std_msgs.msg import Empty
from std_msgs.msg import Float32
from std_msgs.msg import Int64

from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped

from nav_msgs.msg import Odometry
import tf
import numpy as np
import cv2
from cv_bridge import CvBridge
RED_MISSION = 0

PI = math.pi
FRENTE = -2.56497427607
TRAZ = ((FRENTE + 2*PI) % (2*PI) ) - PI 
ESQUERDA = ((FRENTE + 1.5 * PI) % (2 * PI)) - PI  
DIREITA =  ((FRENTE + 0.5 * PI) % (2 * PI)) - PI
HEIGHT = 1
FIRST_GOING =2
ROW_DISTANCE = 2.8
ROW_WIDTH = 6
ROWS = 12

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
        self.quantidade_fotos = 0

        self.local_sub = rospy.Subscriber('/bebop/odom', Odometry, self.local_callback)
        self.camera_sub = rospy.Subscriber('/bebop/image_raw', Image, self.image_callback)
        self.sonar_sub = rospy.Subscriber('/jonas/sonar', Float32, self.sonar_callback)
        self.temperature_sub = rospy.Subscriber('/jonas/temperature', Float32, self.temperature_callback)
        self.co2_sub = rospy.Subscriber('/jonas/CO2', Int64, self.co2_callback)

        self.local_cam_sub = rospy.Subscriber('/orb_sl2_mono/pose', PoseStamped, self.local_callback)

        self.listener = tf.TransformListener() # position tf listener

        rospy.sleep(3.)
        rospy.wait_for_message('/bebop/odom', Odometry)
        rospy.wait_for_message("/bebop/image_raw", Image)
        self.width = 856
        self.height = 480

        self.direction = "FRENTE"

    def sonar_callback(self, data):
        self.sonar = data

    def local_cam_sub(self, data):
        self.correct_pose_x = self.Pose.Point.x
        self.correct_pose_y = self.Pose.Point.y
        self.correct_pose_z = self.Pose.Point.z


    def co2_callback(self, data):
        self.co2 = data  

    def temperature_callback(self, data):
        self.temperature = data

    def local_callback(self, data):
        pass
        '''
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
        '''
    def image_callback(self, data):
        self.image = data
        self.cv_image = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8") 

    def takeoff(self):
        self.initial_yaw = self.yaw
        #print("-----------Initial: " + str(self.initial_yaw))
        rospy.loginfo("Takeoff")
        self.takeoff_pub.publish(self.empty)
        
        rospy.sleep(5.)
        print(self.yaw)

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

        while np.sqrt((x - self.correct_pose_x)**2 + (y -self.correct_pose_y)**2)  > tolerance and not rospy.is_shutdown():

           
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


                #print("P= " + str(self.P * delta_y))
                #print("I = " +str( self.I * integral_y))
                #print("D = " + str( self.D * derivative_y ))

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
                        

                print("( " + str(self.correct_pose_x) + " , " +  str(self.correct_pose_y) + " )")
                #print("vel_x = " + str(vel_x))
                #print("vel_y = " + str(vel_y))
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

    def set_yaw(self, desired_yaw, tolerance=0.03):
        P = desired_yaw - self.yaw
        if(P  > PI):
            P = -2*PI + P
        elif (P < -PI):
            P = 2*PI + P
        self.yaw_antigo = 0
        while abs(P) >= tolerance and not rospy.is_shutdown():
            self.yaw_atual = self.yaw
            if self.yaw_antigo - self.yaw_atual == 0:
                bebop.set_vel(0.0,0.0,0.0,0.3*P)
                P = desired_yaw - self.yaw
                if(P  > PI):
                    P = -2*PI + P
                elif (P < -PI):
                    P = 2*PI + P
                #print("desired: " + str(desired_yaw))
                #print("Yaw: " + str(self.yaw))
                #print("P: " + str(P))
            self.yaw_antigo = self.yaw

    def fix_trajectory(self):
        self.set_yaw(FRENTE)
        self.set_position(0, 0, HEIGHT)
        self.set_position(0, FIRST_GOING, HEIGHT) 

        retas_completas = ROWS/2
        for i in range(int( (2 * retas_completas ) - 2)):
            if (i < retas_completas):
                self.set_position(-ROW_WIDTH, FIRST_GOING + i*ROW_DISTANCE, HEIGHT)
                set_yaw(TRAZ)
                self.set_position(-ROW_WIDTH, FIRST_GOING + i*ROW_DISTANCE, HEIGHT)

                self.set_position(0, FIRST_GOING + i*ROW_DISTANCE, HEIGHT)

                if(i != retas_completas - 1):
                    self.set_yaw(FRENTE)
                    self.set_position(0, FIRST_GOING + i*ROW_DISTANCE, HEIGHT)
                    self.set_position(0, FIRST_GOING + (i+1)*ROW_DISTANCE, HEIGHT)
                else:
                    self.set_position(0, FIRST_GOING + i*ROW_DISTANCE, HEIGHT)

            else:
                self.set_position(ROW_WIDTH, FIRST_GOING + (2*retas_completas-(i+1))*ROW_DISTANCE, HEIGHT)
                self.set_yaw(TRAZ)
                
                self.set_position(0, FIRST_GOING + (2*retas_completas-(i+1))*ROW_DISTANCE, HEIGHT)
                self.set_yaw(FRENTE)

                if(i != 2*retas_completas-1):
                    self.set_position(0, FIRST_GOING + ((2*retas_completas-(i+2))*ROW_DISTANCE), HEIGHT)
                else:
                    self.set_position(0, 0, HEIGHT)
            
        self.land()

    def trajectory(self):
        #SOBE A 1 METRO DE ALTURA
        self.takeoff() 
        #VIRA A CAMERA PARA BAIXO
        self.camera_control(0,-45)
        rospy.sleep(5)
        #RODA O DRONE PARA ESQUERDA
        self.set_yaw(FRENTE)
        #SOBE PARA A ALTURA DESEJADA
        self.set_position(0, 0, HEIGHT)

        #VAI PARA FRENTE DE LADO
        self.set_position(0, 0.85, HEIGHT)

        #DEFINE O NUMERO DE RETAS QUE VAO DE ESQUERDA A DIREITA
        self.retas_completas = (ROWS - 2)/2 #O "-2" ESTA AI PQ DUAS COLUNAS SAO INDIVIDUAIS
        
        for i in range(int(self.retas_completas)):
            #RED MISSION ACIONA A CENTRALIZACAO NO VERMELHO
            if (RED_MISSION == 1):

                self.align = 0 #DEFINE COMO DESCENTRALIZADO
                #self.camera_control(0, 0) #VIRA A CAMERA PARA FRENTE
                #rospy.sleep(5)

                while(self.align == 0) and not rospy.is_shutdown(): #ENQUANTO NAO ESTIVER CENTRALIZADO
                    print("Centralizando no vermelho")
                    x_goal = self.center_red() #ENCONTRA O CENTRO DO VERMELHO

                    print("PID")
                    self.camera_pid(x_goal, 1)# 1 POIS O DRONE ESTA OLHANDO PARA A ESQUERDA

                self.set_vel(0, 0, 0, 0)

                #self.camera_control(0, -90) #VIRA A CAMERA PARA BAIXO
                #rospy.sleep(5)

            current_y = self.correct_pose_y

            while not rospy.is_shutdown() and abs(self.sonar - 6) > 0.5: 
                self.set_vel(0.03, 0, 0, 0)
            
            self.set_vel(0, 0, 0, 0)

            current_x = self.correct_pose_x
            current_y = self.correct_pose_y
         
            self.set_yaw(TRAZ)
            self.set_position(current_x, current_y, HEIGHT)
         
            while not rospy.is_shutdown() and abs(self.correct_pose_x - 0) > TOL :
                print(self.correct_pose_x)
                self.set_vel(0.03, 0, 0, 0)

            #PERCORRER O LADO DIREITO
            if (RED_MISSION == 1):

                self.align = 0 #DEFINE COMO DESCENTRALIZADO
                #self.camera_control(0, 0) 
                #rospy.sleep(5)

                while(self.align == 0) and not rospy.is_shutdown():
                    print("Centralizando no vermelho")
                    x_goal = self.center_red()

                    print("PID")
                    self.camera_pid(x_goal, -1) # -1 POIS O DRONE ESTA OLHANDO PARA A DIREITA

                self.set_vel(0, 0, 0, 0)

                #self.camera_control(0, -90) #VIRA A CAMERA PARA BAIXO
                #rospy.sleep(5)
            
            while not rospy.is_shutdown() and abs(self.sonar - 6) > 0.5 :
                self.set_vel(0.03, 0, 0, 0)
            
            self.set_vel(0, 0, 0, 0)

            current_x = self.correct_pose_x
            current_y = self.correct_pose_y
         
            self.set_yaw(FRENTE)
            self.set_position(current_x, current_y, HEIGHT)

            while not rospy.is_shutdown() and abs(self.correct_pose_x - 0) > TOL :
                print(self.correct_pose_x)
                self.mav.set_vel(0.03, 0, 0, 0)
         
            if (i < self.retas_completas-1):
                self.set_position(0, -(self.correct_pose_y + ROW_DISTANCE), HEIGHT)  #LOGICA: (x, y) --> (y, -x)

            elif (i == self.retas_completas-1): #SE TIVER FEITO A ULTIMA COLUNA 
                self.land()
            else: #NAO DEVERIA CONSEGUIR ENTRAR AQUI
                print("???")
        
    def center_red(self):
        img = self.cv_image
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(img , (0, 176, 144), (17, 255, 255))
        kernel = np.ones((15, 15), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=6)
        ksize = (5, 5)
        mask = cv2.blur(mask, ksize) 
        mask = cv2.blur(mask, ksize) 
    
        height = mask.shape[0]
        width = mask.shape[1]

        _, contours, _= cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
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

                    #print(f"x: {centerx[0]}")
                    #print(f"x: {centerx[1]}")

        x_goal =( ( centerx[1] - centerx[0] ) / 2 ) + centerx[0]

        print(x_goal)
       

        return x_goal

    def camera_pid(self, x_goal, signal):
        self.TARGET = int(self.width/2)
        delta_x = self.TARGET - x_goal

        self.TOL = 0.02
        self.PID = 0.0002
        # Centralization PID
        vel_y = delta_x * self.PID

        self.align = 0

        if abs(vel_y) < self.TOL:
            vel_y = 0.0
            self.row_y = self.mav.drone_pose.pose.position.y
            self.align = 1
        
        self.mav.set_vel(0, signal * vel_y, 0)
              
        #self.get_logger().info(" erro: " + str(delta_x))  
        #self.get_logger().info(" vely: " + str(vel_y))

    def save_image(self):
        name = "image%d.jpg"%self.quantidade_fotos

        cv2.imwrite(name, self.cam_frame)
        self.quantidade_fotos += 1

    def ultrateste(self):
        self.takeoff()
        while not rospy.is_shutdown() and abs(self.sonar - 2000) > 0.5 :
            print("Sonar: " + str(self.sonar))
            self.set_vel(0.03 , 0,0)
        self.set_vel(0, 0, 0)
        self.land()

if __name__ == "__main__":
    rospy.init_node('bebopbase')
    bebop = Bebopbase()
    #bebop.ultrateste()

    bebop.takeoff()
    bebop.set_position(1,0,1)
    bebop.set_yaw(TRAZ)
    bebop.set_position(0,0,1)
    bebop.set_position(-1,0)    
    bebop.land()
