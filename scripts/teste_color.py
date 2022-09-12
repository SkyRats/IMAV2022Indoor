# IMPORTANT
# INSTALL VERSION 4.2.0 OF OPENCV: pip install opencv-python==4.2.0.34
# sudo apt-get install scrot

from turtle import color, onclick
from warnings import resetwarnings
import cv2
from cv2 import erode
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import pyautogui
from pynput import mouse, keyboard
import colorsys
import math

hMin = 0
sMin = 0
vMin = 0
hMax = 179
sMax = 255
vMax = 255

hMinTest = 0
sMinTest = 0
vMinTest = 0
hMaxTest = 179
sMaxTest = 255
vMaxTest = 255

cliques = 0
cliquesMax = 3
recebendoCliques = False

key = 'a'

def on_click(x, y, button, pressed):
    
    # add
    global cliques
    global cliquesMax
    global hMax
    global hMaxTest
    global sMax
    global sMaxTest
    global vMax
    global vMaxTest
    global hMin
    global hMinTest
    global sMin
    global sMinTest
    global vMin
    global vMinTest

    if pressed and recebendoCliques and button == mouse.Button.left:

        

        cliques = cliques + 1

        mousePosition = pyautogui.position()
        RGB_color = pyautogui.pixel(mousePosition.x, mousePosition.y)
        (r, g, b) = (RGB_color.red / 255, RGB_color.green / 255, RGB_color.blue / 255)
        (h, s, v) = colorsys.rgb_to_hsv(r, g, b)
        (h, s, v) = (int(h * 179), int(s * 255), int(v * 255))
        
        
        if cliques == 1:
            hMaxTest = h
            hMinTest = h
            sMaxTest = s
            sMinTest = s
            vMaxTest = v
            vMinTest = v
        else: 
            if hMaxTest < h:
                hMaxTest = h
            if hMinTest > h:
                hMinTest = h            
            if sMaxTest < s:
                sMaxTest = s
            if sMinTest > s:
                sMinTest = s
            if vMaxTest < v:
                vMaxTest = v
            if vMinTest > v:
                vMinTest = v
        if cliques >= cliquesMax:
            cv2.setTrackbarPos('HMin', 'Parametros', hMinTest)
            cv2.setTrackbarPos('SMin', 'Parametros', sMinTest)
            cv2.setTrackbarPos('VMin', 'Parametros', vMinTest)
            cv2.setTrackbarPos('HMax', 'Parametros', hMaxTest)
            cv2.setTrackbarPos('SMax', 'Parametros', sMaxTest)
            cv2.setTrackbarPos('VMax', 'Parametros', vMaxTest)

    # subtract
    if pressed and recebendoCliques and button == mouse.Button.right:
        cliques = cliques + 1

        mousePosition = pyautogui.position()
        RGB_color = pyautogui.pixel(mousePosition.x, mousePosition.y)
        (r, g, b) = (RGB_color.red / 255, RGB_color.green / 255, RGB_color.blue / 255)
        (h, s, v) = colorsys.rgb_to_hsv(r, g, b)
        (h, s, v) = (int(h * 179), int(s * 255), int(v * 255))
        
        
        if cliques == 1:
            hMaxTest = 179
            hMinTest = 0
            sMaxTest = 255
            sMinTest = 0
            vMaxTest = 255
            vMinTest = 0
        else: 
            if hMaxTest > h:
                hMaxTest = h
            if hMinTest < h:
                hMinTest = h            
            if sMaxTest > s:
                sMaxTest = s
            if sMinTest < s:
                sMinTest = s
            if vMaxTest > v:
                vMaxTest = v
            if vMinTest < v:
                vMinTest = v
        if cliques >= cliquesMax:
            # divide por 100 para deixar mais preciso, esse numero pode ser calibrado fuuturamente
            cv2.setTrackbarPos('HMin', 'Parametros', math.floor(hMinTest / 100))
            cv2.setTrackbarPos('SMin', 'Parametros', math.floor(sMinTest / 100))
            cv2.setTrackbarPos('VMin', 'Parametros', math.floor(vMinTest / 100))
            cv2.setTrackbarPos('HMax', 'Parametros', hMaxTest)
            cv2.setTrackbarPos('SMax', 'Parametros', sMaxTest)
            cv2.setTrackbarPos('VMax', 'Parametros', vMaxTest)

def on_press_keyboard(key):
    try:
        if key.char == 'q':
            capture.release()
            cv2.destroyAllWindows()
            cv2.waitKey(0)
        if key.char == 'r':
            reset()
        if key.char == 'e':
            setandoCliques()
        if key.char == 'f':
            set_red()
        if key.char == 'g':
            set_green()
            
    except AttributeError:
        pass

def reset (): 
    cv2.setTrackbarPos('HMin', 'Parametros', 0)
    cv2.setTrackbarPos('SMin', 'Parametros', 0)
    cv2.setTrackbarPos('VMin', 'Parametros', 0)
    cv2.setTrackbarPos('HMax', 'Parametros', 179)
    cv2.setTrackbarPos('SMax', 'Parametros', 255)
    cv2.setTrackbarPos('VMax', 'Parametros', 255)

    print("Mascara resetada!")
    global cliques
    cliques = 0
    
def set_red(): 
    cv2.setTrackbarPos('HMin', 'Parametros', 0)
    cv2.setTrackbarPos('SMin', 'Parametros', 118)
    cv2.setTrackbarPos('VMin', 'Parametros', 144)
    cv2.setTrackbarPos('HMax', 'Parametros', 4)
    cv2.setTrackbarPos('SMax', 'Parametros', 227)
    cv2.setTrackbarPos('VMax', 'Parametros', 255)

    print("Mascara calibrada em vermelho")
    global cliques
    cliques = 0

def set_green(): 
    cv2.setTrackbarPos('HMin', 'Parametros', 60)
    cv2.setTrackbarPos('SMin', 'Parametros', 82)
    cv2.setTrackbarPos('VMin', 'Parametros', 83)
    cv2.setTrackbarPos('HMax', 'Parametros', 97)
    cv2.setTrackbarPos('SMax', 'Parametros', 184)
    cv2.setTrackbarPos('VMax', 'Parametros', 211)

    print("Mascara calibrada em verde")
    global cliques
    cliques = 0

listener = mouse.Listener(on_click=on_click)
listener.start()

listener = keyboard.Listener(
    on_press=on_press_keyboard
    )
listener.start()

def setandoCliques ():
    global recebendoCliques
    if recebendoCliques == True:
        recebendoCliques = False
        print("Cliques desativados!")
    else:
        recebendoCliques = True
        print("Cliques ativados!")

def get_mask(hsv , lower_color , upper_color):
    lower = np.array(lower_color)
    upper = np.array(upper_color)
    
    mask = cv2.inRange(hsv , lower, upper)

    return mask

def nothing(x):
    pass

capture = cv2.VideoCapture(0)
skyratsImg = cv2.imread("placa2.png")
skyratsImgResize = cv2.resize(skyratsImg, (300, 300))

cv2.namedWindow('Camera - R: resetar mascara, E: ativar mouse e Q: sair')
cv2.namedWindow('Parametros')

cv2.createTrackbar('HMin', 'Parametros', 0, 179, nothing)
cv2.createTrackbar('SMin', 'Parametros', 0, 255, nothing)
cv2.createTrackbar('VMin', 'Parametros', 0, 255, nothing)
cv2.createTrackbar('HMax', 'Parametros', 179, 179, nothing)
cv2.createTrackbar('SMax', 'Parametros', 255, 255, nothing)
cv2.createTrackbar('VMax', 'Parametros', 255, 255, nothing)
cv2.createTrackbar('Erode', 'Parametros', 0, 100, nothing)
cv2.createTrackbar('Dilate', 'Parametros', 0, 100, nothing)
cv2.createTrackbar('Blur', 'Parametros', 1, 100, nothing)
cv2.createTrackbar('CliquesMaximos', 'Parametros', 1, 100, nothing)
cv2.createTrackbar('Cliques', 'Parametros', 1, cliquesMax, nothing)

cv2.setTrackbarPos('CliquesMaximos', 'Parametros', 10)

while True: 
    frame = cv2.imread("caminho1.jpeg")

    hMin = cv2.getTrackbarPos('HMin', 'Parametros')
    sMin = cv2.getTrackbarPos('SMin', 'Parametros')
    vMin = cv2.getTrackbarPos('VMin', 'Parametros')
    hMax = cv2.getTrackbarPos('HMax', 'Parametros')
    sMax = cv2.getTrackbarPos('SMax', 'Parametros')
    vMax = cv2.getTrackbarPos('VMax', 'Parametros')
    cv2.setTrackbarMax('Cliques', 'Parametros', cliquesMax)

    cliquesMax = cv2.getTrackbarPos('CliquesMaximos', 'Parametros')

    cv2.setTrackbarPos('Cliques', 'Parametros', cliques)
    
    
    lower = [hMin, sMin, vMin]
    upper = [hMax, sMax, vMax]
    # get_mask(hsv, [160, 100, 20], [179, 255, 255])

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = get_mask(hsv, lower, upper)

    
    result = cv2.bitwise_and(frame, frame, mask= mask)
    
    #plotting
    erode_size = cv2.getTrackbarPos('Erode', 'Parametros')
    dilate_size = cv2.getTrackbarPos('Dilate', 'Parametros')
    blur_size = cv2.getTrackbarPos('Blur', 'Parametros')

    erode_kernel = np.ones((erode_size, erode_size), np.float32)
    
    dilate_kernel = np.ones((dilate_size, dilate_size), np.float32)
    

    if blur_size == 0: 
        blur_size = 1
    blurredFrame = cv2.blur(result, (blur_size, blur_size))

    result = cv2.dilate(blurredFrame, dilate_kernel)
    result = cv2.erode(result, erode_kernel)
    
    cv2.imshow('Parametros', skyratsImgResize)
    cv2.imshow('Camera - R: resetar mascara, E: ativar mouse e Q: sair', result)
    
    if cv2.waitKey(20): key = cv2.waitKey(20)  