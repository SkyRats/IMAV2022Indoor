from __future__ import (
    division, absolute_import, print_function, unicode_literals)

from pickle import PickleError
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

#https://stackoverflow.com/questions/46390779/automatic-white-balancing-with-grayworld-assumption

def find_color(image):
	return None 

#	#imagens teste
#	ref = cv2.imread('reference.png')
#	img = cv2.imread('input.png')
#
#	ref = cv2.cvtColor(ref, cv2.COLOR_BGR2GRAY)
#	ref_s = cv2.resize(ref, (500, 500))
#	#cv2.imshow("ref", ref_s)
#
#	#img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
#	#img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#	#img_s = cv2.resize(img, (500, 500))
#	#cv2.imshow("img", img_s)
#
#	#cv2.waitKey(1000000000)
#
#	mask = cv2.inRange(ref,230, 255)
#
#	#s = 70
#	#mask = cv2.inRange(ref , (0, 0, 255-s), (255, s/2, 255))
#	#mask2 = np.array(mask, dtype=np.uint8)
#	height = mask.shape[0]
#	width = mask.shape[1]
#	conv = np.zeros((height ,width),np.uint8)
#	#mask2 = cv2.cvtColor(mask2, cv2.COLOR_HSV2BGR) 
#	#mask2 = cv2.cvtColor(mask2, cv2.COLOR_BGR2GRAY) 
#
#	for i in range(height-2):
#		for j in range(width-2):
#			if mask[i,j] != 0:
#				[b, g, r] = img[i,j] - mask[i,j]
#				conv[i,j] = [b, g, r]
#
#			img[i,j] = img[i,j] - conv[i,j]	
#
#	img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
#	cv2.namedWindow("img", cv2.WINDOW_AUTOSIZE)
#	img = cv2.resize(img, (500, 500))
#	cv2.imshow("img", img)
#	cv2.waitKey(1000000000)

def compare(imgW, imgP):
	#imgW_g = cv2.cvtColor(imgW, cv2.COLOR_BGR2GRAY)
	#imgP_g = cv2.cvtColor(imgP, cv2.COLOR_BGR2GRAY) 

	img_comp = imgW[15:55, 500:540] - imgP[15:55, 500:540]
	
	conv = np.zeros((1 ,1, 3),np.uint8)
	for i in range(40):
		for j in range(40):
			conv[:,:] = conv[:,:] + img_comp[i][j]/1600
	
	#conv[:,:] = img_comp
	#img_comp_bgr = cv2.cvtColor(conv, cv2.COLOR_GRAY2BGR)
	
	return conv

def correction(imgP, img_comp_bgr):
	
	#final = imgP + img_comp_bgr
	height = imgP.shape[0]
	width = imgP.shape[1]
	final = np.zeros((height ,width),np.uint8)

	final = imgP + img_comp_bgr

	return final 

def findWhite(filepath):
	img = cv2.imread(filepath)
	piece = img[0:55, :]

	return piece

def main():
	imgP = cv2.imread("/home/gabiyuri/frame1.jpg")
	imgW = cv2.imread('branco.png')

	imgP = cv2.resize(imgP, (1080, 1920))
	imgW = cv2.resize(imgW, (1080, 1920))

	final = correction(imgP, compare(imgW, imgP))


	imgP = cv2.resize(imgP, (200, 200))
	imgW = cv2.resize(imgW, (200, 200))
	final = cv2.resize(final, (200, 200))

	cv2.imshow("imgW", imgW)
	cv2.imshow("imgP", imgP)
	cv2.imshow("final", final)
	cv2.waitKey(10000000)

def show(final):
    print('display')
    cv2.imshow('Temple', final)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def white_balance_loops(img):
    result = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    avg_a = np.average(result[:, :, 1])
    avg_b = np.average(result[:, :, 2])
    result[:, :, 1] = result[:, :, 1] - ((avg_a - 128) * (result[:, :, 0] / 255.0) * 1.1)
    result[:, :, 2] = result[:, :, 2] - ((avg_b - 128) * (result[:, :, 0] / 255.0) * 1.1)
    result = cv2.cvtColor(result, cv2.COLOR_LAB2BGR)
    return result


if __name__ == "__main__":
	img = cv2.imread("/home/gabiyuri/frame1.jpg")

	final = np.hstack((img, white_balance_loops(img)))
	show(final)
	cv2.imwrite('result.jpg', final) 

	