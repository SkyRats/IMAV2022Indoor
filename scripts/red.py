import numpy as np
import cv2

img_raw = cv2.imread('placa3.png')
img = cv2.cvtColor(img_raw, cv2.COLOR_BGR2HSV)
mask = cv2.inRange(img , (0, 176, 144), (17, 255, 255))
kernel = np.ones((15, 15), np.uint8)
mask = cv2.dilate(mask, kernel, iterations=6)
ksize = (5, 5)
mask = cv2.blur(mask, ksize) 
mask = cv2.blur(mask, ksize) 
cv2.imshow("a", img_raw)
cv2.waitKey(0)
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
        mask = cv2.circle(mask, (int(centerx[0]), int(height/2)), radius=2, color=(255,255,255), thickness=-1)
        mask = cv2.circle(mask, (int(centerx[1]), int(height/2)), radius=2, color=(255,255,255), thickness=-1)

            #print(f"x: {centerx[0]}")
            #print(f"x: {centerx[1]}")


x_goal =( centerx[1] - centerx[0] ) / 2 + centerx[0]


print(x_goal)
mask = cv2.circle(mask, (int(x_goal), int(height/2)), radius=2, color=(255,255,255), thickness=-1)
cv2.imshow("a", mask)
cv2.waitKey(0)