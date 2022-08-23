from imutils.perspective import four_point_transform
from skimage import exposure
import numpy as np
import argparse
import imutils
import cv2
import sys
from cv2 import aruco
#generate ArUco 
#https://pyimagesearch.com/2020/12/14/generating-aruco-markers-with-opencv-and-python/

#https://jephraim-manansala.medium.com/image-processing-with-python-color-correction-using-white-balancing-6c6c749886de
def find_color_card(image):
	arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
	arucoParams = cv2.aruco.DetectorParameters_create()
	(corners, ids, rejected) = cv2.aruco.detectMarkers(image,
		arucoDict, parameters=arucoParams)

	# try to extract the coordinates of the color correction card
	try:
		# otherwise, we've found the four ArUco markers, so we can
		# continue by flattening the ArUco IDs list
		ids = ids.flatten()
		# extract the top-left marker
		i = np.squeeze(np.where(ids == 923))
		topLeft = np.squeeze(corners[i])[0]
		# extract the top-right marker
		i = np.squeeze(np.where(ids == 1001))
		topRight = np.squeeze(corners[i])[1]
		# extract the bottom-right marker
		i = np.squeeze(np.where(ids == 241))
		bottomRight = np.squeeze(corners[i])[2]
		# extract the bottom-left marker
		i = np.squeeze(np.where(ids == 1007))
		bottomLeft = np.squeeze(corners[i])[3]
	# we could not find color correction card, so gracefully return
	except:
		return None

		# build our list of reference points and apply a perspective
	# transform to obtain a top-down, bird’s-eye view of the color
	# matching card
	cardCoords = np.array([topLeft, topRight,
		bottomRight, bottomLeft])
	card = four_point_transform(image, cardCoords)
	# return the color matching card to the calling function
	return card

def main():
	# load the reference image and input images from disk
	print("[INFO] loading images...")
	ref = cv2.imread('reference.jpeg')
	image = cv2.imread('input.jpeg')
	# resize the reference and input images
	ref = imutils.resize(ref, width=600)
	image = imutils.resize(image, width=600)
	# display the reference and input images to our screen
	cv2.imshow("Reference", ref)
	cv2.imshow("Input", image)

	# find the color matching card in each image
	print("[INFO] finding color matching cards...")
	refCard = find_color_card(ref)
	imageCard = find_color_card(image)
	# if the color matching card is not found in either the reference
	# image or the input image, gracefully exit
	if refCard is None or imageCard is None:
		print("[INFO] could not find color matching card in both images")
		sys.exit(0)	

	# show the color matching card in the reference image and input image,
	# respectively
	cv2.imshow("Reference Color Card", refCard)
	cv2.imshow("Input Color Card", imageCard)
	# apply histogram matching from the color matching card in the
	# reference image to the color matching card in the input image
	print("[INFO] matching images...")
	imageCard = exposure.match_histograms(imageCard, refCard,
		multichannel=True)
	# show our input color matching card after histogram matching
	cv2.imshow("Input Color Card After Matching", imageCard)
	cv2.waitKey(0)

if __name__ == "__main__":
	main()