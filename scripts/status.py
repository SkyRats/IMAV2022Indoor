#!/usr/bin/env python2

from pty import slave_open
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
import tf

from genericpath import sameopenfile
from random import sample
import cv2
import os

MIN_GREEN = (35, 64, 70)
MAX_GREEN = (80, 255, 231)
MIN_BROWN = (11, 4, 133)
MAX_BROWN = (76, 52, 255)


class Status():

    def __init__(self):

        self.co2_sub = rospy.Subscriber('/jonas/CO2', Int64, self.co2_callback)
        self.sonar_sub = rospy.Subscriber('/jonas/sonar', Float32, self.sonar_callback)
        self.temp_sub = rospy.Subscriber('/jonas/temperature', Float32, self.temp_callback)
        self.local_sub = rospy.Subscriber('/bebop/odom', Odometry, self.local_callback)
        self.trajectory_sub = rospy.Subscriber('/trajectory/finish', Bool, self.trajectory_callback)

        self.trajectory = 0
        self.video = cv2.VideoCapture("/dev/video1")
        self.image = cv2.imread("/home/gabs/skyrats_ws/src/IMAV2022Indoor/scripts/planta.png")

        self.photos = 0
        self.disease = False

        rospy.sleep(5)
        rospy.loginfo("Object Map() created!")


    ########################################## Callback ############################################# 

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


    #################################### CO2 and Temperature ######################################## 

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
        
    def save_graph(self):
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

        plt.savefig('co2_temp.png')
        rospy.loginfo("CO2 and Temperature graph saved!")
        return("1")


    ###################################### Disease Detection ######################################## 

    def cropImage(self, image):
        #Split an image 1920x1080 in 6 pieces  960x540
        croppedImage = []
        for i in range(0,6):
            if i < 3:
                piece = image[0:270, i*320:(i+1)*320]
                croppedImage.append(piece)
                
            if i >= 3:  
                piece = image[270:540, (i-3)*320:(i-2)*320]
                croppedImage.append(piece)

        return croppedImage

    def recomposeImage(self, images):
        #receives cropped image array and returns the original image
        result = np.zeros(self.image.shape, np.uint8)
        for i in range(0, 6):
            if i < 3:
                result[0:270, i*320:(i+1)*320] = images[i]

            if i >=3:
                result[270:540, (i-3)*320:(i-2)*320] = images[i]

        return result

    def imageProcessing(self, image, imageCopy):
        #apply filter and transformations

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray, 110, 255, cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(image= thresh, mode= cv2.RETR_LIST, method= cv2.CHAIN_APPROX_NONE)
        sorted_contours = sorted(contours, key= cv2.contourArea, reverse= True)

        #find the biggest leaf in the green image
        largestContour = []
        if sorted_contours:
            largestContour = [sorted_contours[0]]
            contour_area = cv2.contourArea(sorted_contours[0])

        mask = np.zeros(image.shape, np.uint8)
        mask.fill(255)
        if largestContour != 1:
            cv2.fillPoly(mask, largestContour, (0,0,0))
        
        mask = cv2.bitwise_not(mask)
        leaf = cv2.bitwise_and(mask, imageCopy)

        #use brown mask to verify disease
        hsv = cv2.cvtColor(leaf, cv2.COLOR_BGR2HSV)
        brownMask = cv2.inRange(hsv, MIN_BROWN, MAX_BROWN)
        brownMask = brownMask>0
        imageBrown = np.zeros(image.shape, np.uint8)
        imageBrown[brownMask] = leaf[brownMask]

        gray = cv2.cvtColor(imageBrown, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray, 40, 255, cv2.THRESH_BINARY)

        contours, hierarchy = cv2.findContours(image=thresh,mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
        result = cv2.drawContours(image=imageCopy, contours=contours, contourIdx=-1, color=(255,0,0), thickness=2, lineType=cv2.LINE_AA)

        max_contours = len(contours)
        i = 0
        for contour in contours:
            if cv2.contourArea(contour)/contour_area > 0.05:
                i += 1
    
        if i > max_contours/2:
            self.disease = True

        return result

    def detectionLoop(self):
        imageHSV = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        #saturation = imageHSV[..., 1]
        #saturation = cv2.add(saturation, 150)

        greenMask = cv2.inRange(imageHSV, MIN_GREEN, MAX_GREEN)
        imask = greenMask>0
        imageGreen = np.zeros_like(self.image, np.uint8)
        imageGreen[imask] = self.image[imask]
        
        croppedImage = self.cropImage(imageGreen)
        imageCopy = self.cropImage(self.image)

        processedImage = []
        for i in range(0, 6):
            processedImage.append(self.imageProcessing(croppedImage[i], imageCopy[i]))

        result = self.recomposeImage(processedImage)

        return result

    def save_disease(frame, x, y):

        plt.imshow(frame)
        title = "Disease Plant in (" + str(x) + ", " + str(y) + ")"
        plt.title(title)

        plt.savefig('disease.png')
        rospy.loginfo("Disease spoted and saved!")
        return("1")


    ###################################### Photo Mapping ######################################## 

    def save_image(self, frame):
        self.photos += 1

        name = "image%d.jpg"%self.photos
        path = "/home/gabs/skyrats_ws/src/IMAV2022Indoor/images"

        cv2.imwrite(os.path.join(path, name), frame)
        return("1")


    #################################### Analize periodicaly ###################################### 

    def run(self):
        self.createCSV()
        while not self.trajectory:
            self.addtoCSV()
            success, self.image = video.read()
            self.save_image(self.image)

            if self.disease == True:
                self.save_disease(self.image, self.correct_pose_x, self.correct_pose_y)
                self.disease == False

            rospy.sleep(3)

        rospy.loginfo("All images saved!")

if __name__ == '__main__':
    rospy.init_node("status")
    status = Status()
    status.run()