from genericpath import sameopenfile
from random import sample
import cv2
import numpy as np

class diseaseDetection:

    def __init__(self):
        video_path = "/home/renato/skyrats_ws2/src/IMAV2022Indoor/images/video1.mp4"
        #self.cap = cv2.VideoCapture("/dev/video0")
        self.cap = cv2.VideoCapture(video_path)

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

        largestContour = []
        if sorted_contours:
            largestContour = [sorted_contours[0]]

        mask = np.zeros(image.shape, np.uint8)
        mask.fill(255)
        if largestContour != 1:
            cv2.fillPoly(mask, largestContour, (0,0,0))
        
        mask = cv2.bitwise_not(mask)
        result = cv2.bitwise_and(mask, imageCopy)
        hsv = cv2.cvtColor(result, cv2.COLOR_BGR2HSV)
        brownMask = cv2.inRange(hsv, (0, 0, 0), (14, 255, 255))
        brownMask = brownMask>0
        imageBrown = np.zeros(image.shape, np.uint8)
        imageBrown[brownMask] = result[brownMask]

        gray = cv2.cvtColor(imageBrown, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray, 40, 255, cv2.THRESH_BINARY)

        contours, hierarchy = cv2.findContours(image=thresh,mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
        result = cv2.drawContours(image=imageCopy, contours=contours, contourIdx=-1, color=(255,0,0), thickness=2, lineType=cv2.LINE_AA)

        return result

    def detectionLoop(self):
        while self.cap.isOpened():

            success,self.image = self.cap.read()
            
            self.image = cv2.resize(self.image, (960,540))


            imageHSV = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
            #saturation = imageHSV[..., 1]
            #saturation = cv2.add(saturation, 150)

            greenMask = cv2.inRange(imageHSV, (29, 29, 0), (101, 255, 210))           
            #greenMask = cv2.inRange(imageHSV, (31, 34, 0), (179, 255, 255))
            #greenMask = cv2.inRange(imageHSV, (59, 8, 138), (98, 24, 255))
            #greenMask = cv2.inRange(imageHSV, (75, 145, 130), (149, 250, 66))
            #greenMask = cv2.inRange(imageHSV, (36, 25, 25), (86, 255, 255))
            #greenMask = cv2.inRange(imageHSV, (40, 40, 40), (70, 255, 255))
            #greenMask = cv2.inRange(imageHSV, (26, 10, 20), (97, 255, 255))

            #greenMask = cv2.inRange(imageHSV, (0,0,0), (87,255,255))
            imask = greenMask>0

            imageGreen = np.zeros_like(self.image, np.uint8)
            imageGreen[imask] = self.image[imask]
            croppedImage = self.cropImage(imageGreen)
            imageCopy = self.cropImage(self.image)
            processedImage = []


            for i in range(0, 6):

                processedImage.append(self.imageProcessing(croppedImage[i], imageCopy[i]))            

            result = self.recomposeImage(processedImage)
        
            cv2.namedWindow("result", cv2.WINDOW_FULLSCREEN)
            cv2.imshow("result", result)
            cv2.namedWindow("imageGreen", cv2.WINDOW_FULLSCREEN)
            cv2.imshow("imageGreen", imageGreen)

            if cv2.waitKey(5) & 0xFF == 27:
                
                break
    
    def main_interface(self):
        self.detectionLoop()

detection = diseaseDetection()
detection.main_interface()
