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

def acha_centro(image):

    cv2.imshow(image)

    # define the region of interest - bottom triangle
    height = image.shape[0]
    width = image.shape[1]
    print(image.shape)
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

    l1 = []

    for i in range(len(lines)):
        for j in range(len(lines)):
            a = [a[0] for a in lines[i]]
            b = [b[0] for b in lines[j]]

            if abs((a[0] - b[0])  - 1) < 3:
                l1.append(a[0])
                l1.append(b[0])

    centro = sum(l2)/len(l2)

    return centro

if __name__ == '__main__':
    #coloca o endereo da imagem ali
    acha_centro('caminho1.jpeg')