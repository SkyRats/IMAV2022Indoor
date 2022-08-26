from imutils.perspective import four_point_transform
from skimage import exposure
import numpy as np
import imutils
import cv2
import sys
from cv2 import aruco
#generate ArUco 
#https://pyimagesearch.com/2020/12/14/generating-aruco-markers-with-opencv-and-python/

#https://jephraim-manansala.medium.com/image-processing-with-python-color-correction-using-white-balancing-6c6c749886de

def find_color(image):
	return None 

def main():

	#imagens teste
	ref = cv2.imread('reference.png')
	img = cv2.imread('input.png')

	ref = cv2.cvtColor(ref, cv2.COLOR_BGR2GRAY)
	ref_s = cv2.resize(ref, (500, 500))
	#cv2.imshow("ref", ref_s)

	#img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	img_s = cv2.resize(img, (500, 500))
	#cv2.imshow("img", img_s)

	#cv2.waitKey(1000000000)

	mask = cv2.inRange(ref,230, 255)

	#s = 70
	#mask = cv2.inRange(ref , (0, 0, 255-s), (255, s/2, 255))
	#mask2 = np.array(mask, dtype=np.uint8)
	height = mask.shape[0]
	width = mask.shape[1]
	conv = np.zeros((width ,height),np.uint8)
	#mask2 = cv2.cvtColor(mask2, cv2.COLOR_HSV2BGR) 
	#mask2 = cv2.cvtColor(mask2, cv2.COLOR_BGR2GRAY) 

	for i in range(height):
		for j in range(width-1):
			if mask[i,j] != 0:
				x = img[i,j] - mask[i,j]
				conv[i,j] = x

			img[i,j] = img[i,j] - conv[i,j]	

	img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
	cv2.namedWindow("img", cv2.WINDOW_AUTOSIZE)
	img_s = cv2.resize(img, (500, 500))
	cv2.imshow("img", img_s)
	cv2.waitKey(1000000000)

if __name__ == "__main__":
	main()