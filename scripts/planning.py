import cv2
import numpy as np

def white_mask(img):
    #apply filter and transformations
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #ret, thresh = cv2.threshold(gray, 110, 255, cv2.THRESH_BINARY)
    #contours, hierarchy = cv2.findContours(image= thresh, mode= cv2.RETR_LIST, method= cv2.CHAIN_APPROX_NONE)
    #sorted_contours = sorted(contours, key= cv2.contourArea, reverse= True)
    #largestContour = []
    #if sorted_contours:
    #    largestContour = [sorted_contours[0]]
    #mask = np.zeros(image.shape, np.uint8)
    #mask.fill(255)
    #if largestContour != 1:
    #    cv2.fillPoly(mask, largestContour, (0,0,0))
    #
    #mask = cv2.bitwise_not(mask)
    #result = cv2.bitwise_and(mask, imageCopy)
    #hsv = cv2.cvtColor(result, cv2.COLOR_BGR2HSV)
    #brownMask = cv2.inRange(hsv, (0, 0, 0), (14, 255, 255))
    #brownMask = brownMask>0
    #imageBrown = np.zeros(image.shape, np.uint8)
    #imageBrown[brownMask] = result[brownMask]

    #gray = cv2.cvtColor(imageBrown, cv2.COLOR_BGR2GRAY)
    #ret, thresh = cv2.threshold(gray, 40, 255, cv2.THRESH_BINARY)
    #contours, hierarchy = cv2.findContours(image=thresh,mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
    #result = cv2.drawContours(image=imageCopy, contours=contours, contourIdx=-1, color=(255,0,0), thickness=2, lineType=cv2.LINE_AA)

    pass 

def correct_direct():
    abs = 10

    #pega imagens 
    img_atual = imread("caminho1.png")
    img_next = imread("proxima.png")

    #coloca a mascara
    #img_comp = white_mask(img_atual)
    #next_comp = white_mask(img_next)

    #width, length = img_comp.size()
    #tam = width*length

    #x = 0
    #y = 0

    #for i in range(width):
    #    if img_comp[i,0] != 0: 
    #        y += i
    #        x += 1

    #meio_padrao = y/x 

    #for i in range(width):
    #    if img_next[i,0] != 0: 
    #        y += i
    #        x += 1

    #meio_errado = y/x 

    img = cv2.imread("caminho1.jpeg")
    img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(img , (0, 0, 100), (213, 0, 67))
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
    #x_goal = ( centerx[1] - centerx[0] ) / 2 + centerx[0]


