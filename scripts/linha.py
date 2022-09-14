import numpy as np
import cv2
AreaContornoLimiteMin = 5000  #este val

img = cv2.imread('fita.jpg')
img = cv2.resize(img, (960, 540))                # Resize image

height = img.shape[0]
width = img.shape[1]
QtdeContornos = 0
DirecaoASerTomada = 0
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
gray = cv2.GaussianBlur(gray, (21, 21), 0)
FrameBinarizado = cv2.threshold(gray,50,255,cv2.THRESH_BINARY)[1]
FrameBinarizado = cv2.dilate(FrameBinarizado,None,iterations=2)
FrameBinarizado = cv2.bitwise_not(FrameBinarizado)
     
cv2.imshow("a", FrameBinarizado)
cv2.waitKey(0)



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