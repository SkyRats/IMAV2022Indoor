import tensorflow as tf
import numpy as np
import cv2

CATEGORIES = ["_Bacterial_spot", "_Early_blight", "_Late_blight", "_Leaf_Mold", "_Septoria_leaf_spot", "_Spider_mites Two-spotted_spider_mite", "_Target_Spot", "_Tomato_Yellow_Leaf_Curl_Virus", "_Tomato_mosaic_virus", "_healthy"]

def prepare(filepath):
    IMG_SIZE = 256
    img_array = cv2.imread(filepath)
    new_array = cv2.resize(img_array, (IMG_SIZE, IMG_SIZE))

    return new_array.reshape(-1, IMG_SIZE, IMG_SIZE, 1)

def analiseLeaf(filepath):
    healthy = False
    model = tf.keras.models.load_model("tomatoes_96.89.h5")
    prediction = model.predict(prepare(filepath))

    print(prediction)
    a = np.where(prediction[1] == np.max(prediction[1]))

    prob = np.zeros(10)
    beta = 0
    for i in range(3):
        idx = np.where(prediction[i] == np. max(prediction[i]))
        prob[idx] += 1

        alfa = np.where(prediction[i] > 0.2)
        #print(len(alfa[0]))
        if len(alfa[0]) >= 2:
            beta += 1
    #print(beta)
    #print(prob[9])
    if prob[9] >= 2 and not beta >= 2:
        healthy = True

    return healthy

def main():
    filepath = "/home/gabiyuri/Downloads/tomato4.jpeg"
    img = cv2.imread(filepath)
    img = cv2.resize(img, (256, 256))

    if analiseLeaf(filepath):
        cv2.imshow("Healthy", img)
    else: 
        cv2.imshow("Sick", img)

    cv2.waitKey(1000000)

main()

#filepath = "/home/gabiyuri/Pictures/Webcam/folha5.jpg"
#img = cv2.imread(filepath)
#height = img.shape[0]
#width = img.shape[1]
#print("height: ", height)
#print("width: ", width)