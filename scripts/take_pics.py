import cv2

cam = cv2.VideoCapture("/dev/video2")


i = 0
while i < 20:
    v, image = cam.read()
    cv2.imwrite('img'+str(i)+'.jpg', image)
    cv2.waitKey(300)
    i += 1
