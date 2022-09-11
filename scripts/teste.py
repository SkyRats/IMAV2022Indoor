from curses.ascii import CAN
from nis import match
from pickletools import uint1
import cv2
import numpy as np
import matplotlib.pylab as plt

def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    #channel_count = img.shape[2]
    match_mask_color = 255 #(255,) * channel_count
    cv2.fillPoly(mask, vertices, match_mask_color)
    masked_image = cv2.bitwise_and(img, mask)

    return masked_image

def draw_the_lines(img, lines):
    img = np.copy(img)
    blank_image = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)

    for line in lines:
        for x1, y1, x2, y2 in line:
            cv2.line(blank_image, (x1, y1), (x2, y2), (0, 255, 0), thickness=3)

    img = cv2.addWeighted(img, 0.8, blank_image, 1, 0.0)
    return img

if __name__ == '__main__':

    # get the image
    image = cv2.imread("/home/gabs/skyrats_ws/src/IMAV2022Indoor/scripts/caminho1.jpeg")
    image = cv2.blur(image, (5, 5))

    # define the region of interest - bottom triangle
    height = image.shape[0]
    width = image.shape[1]

    region_of_interest_vertices = [
        (0, height),
        (width/2, height/2),
        (width, height)
    ]

    # find the edges
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    canny_image = cv2.Canny(gray_image, 100, 150)

    # create the image with region of interest only
    cropped_image = region_of_interest(canny_image, np.array([region_of_interest_vertices], np.int32),)

    # lines
    lines = cv2.HoughLinesP(
        cropped_image,
        rho=6,
        theta=np.pi/60,
        threshold=160,
        lines=np.array([]),
        minLineLength=min(image.shape[0]/5, image.shape[1]/5),
        maxLineGap=15
    )

    image_with_lines = draw_the_lines(image, lines)

    plt.imshow(image_with_lines)
    #plt.imshow(image_with_lines)
    plt.show()