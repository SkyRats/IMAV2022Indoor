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

def mask_green(image):
    sensitivity = 15

    imageHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    greenMask = cv2.inRange(imageHSV, (65, 39, 19), (107, 255, 66))

    imask = greenMask>0
    imageGreen = np.zeros_like(image, np.uint8)
    imageGreen[imask] = image[imask]

    imageGreen = cv2.cvtColor(imageGreen, cv2.COLOR_HSV2BGR)

    return imageGreen

if __name__ == '__main__':

    # get the image
    image = cv2.imread("/home/gabs/skyrats_ws/src/IMAV2022Indoor/scripts/caminho4.jpeg")
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
    #image_green = mask_green(image)
    #plt.imshow(image_green)
    #plt.show()
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    canny_image = cv2.Canny(gray_image, 70, 125)

    # create the image with region of interest only
    cropped_image = region_of_interest(canny_image, np.array([region_of_interest_vertices], np.int32),)

    # lines
    lines = cv2.HoughLinesP(
        cropped_image,
        rho=1,
        theta=np.pi/180,
        threshold=100,
        lines=np.array([]),
        minLineLength=70,
        maxLineGap=30
    )

    print(lines)

    l1 = []
    l2 = []

    for i in range(len(lines)):
        for j in range(len(lines)):
            if abs((lines[j+1][0] - lines[i][0]) - 1) < 0.2:
                l1.append(lines[j+1][0])
                l2.append(lines[i][0])

    centro = sum(l2-l1)/len(l1)
    print(centro)

    image_with_lines = draw_the_lines(image, lines)

    plt.imshow(image_with_lines)
    plt.show()