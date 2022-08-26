import cv2
import numpy as np

img = cv2.imread('foto.png')
img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
mask = cv2.inRange(img , (0, 230, 200), (20, 255, 255))
cv2.imshow("img", mask)
print(img)
height = mask.shape[0]
width = mask.shape[1]
print("height: " + str(height))
print("width: " + str(width))

contours, heirarchies = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
centerx = np.empty(2)
cont = 0
for i in contours:
    M = cv2.moments(i)
    if M['m00'] != 0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        centerx[cont] = cx
        cont = cont + 1
#print(f"x: {centerx[0]}")
#print(f"x: {centerx[1]}")


x_goal = ( centerx[1] - centerx[0] ) / 2 + centerx[0]
print(f"x_goal: {x_goal}")











