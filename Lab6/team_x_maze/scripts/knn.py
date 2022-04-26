#!/usr/bin/env python
# -*- coding: utf-8 -*-
""""
This is the 7785 LAB6 part1 KNN classifier

Author: Yinuo Wang, Praneeth Erwin Eddu
Date: 04/04/2022

"""
import platform
import cv2 as cv
import csv
import numpy as np
import time
##########################################
#            Variables                   #
##########################################
# model
# model_path = "/home/pran/catkin_ws/src/Robotics-Research-Assignments/team_x_maze/model/knnModel"
model_path = "/home/allen/catkin_ws/src/team_x_maze/model/knnModel"
#model_path = "/home/rick/catkin_ws/src/team_x_maze/model/knnModel"
# dir
trainDirectory = './2022imgs/train_images/'
testDirectory = './2022imgs/test_images/'

# knn
k = 5
dimension = 3 # image channel
train = []
test = []

# image processing
rgb = 1 # cv.imread mode

morphKernel = 5  # Closing and Opening Kernel Size
maxObjects = 1  # Max number of object to detect.
minObjectArea = 300  # Min number of pixels for an object to be recognized.

WHITE = [255, 255, 255]
RED = [255, 0, 0]
GREE = [0, 255, 0]
BLUE = [0, 0, 255]

raw_h = 308
raw_w = 410
#resize
resize_h_ratio = 0.1
resize_w_ratio = 0.1
resize_h = int(raw_h * resize_h_ratio)
resize_w = int(raw_w * resize_w_ratio)

# rgb filter
light_thresh = 60
contrast = 30

# other
debug = False
# debug = True
train_flag = False
test_flag = False

# version check
print("Your python version: " + str(platform.python_version()))
print("Minimum opencv version: 4.2.0")
print("Your opencv version: " + str(cv.__version__))
##########################################
#            Image Processing            #
##########################################

def findObjects(image):
    # Finds the location of the desired object in the image.
    contours, hierarchy = cv.findContours(image, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)  # Contours the image to find blobs of the same color
    cont = sorted(contours, key=cv.contourArea, reverse=True)[:maxObjects]  # Sorts the blobs by size (Largest to smallest)
    x = y = w = h = 1
    # Find the center of mass of the blob if there are any
    if len(cont) > 0:
        M = cv.moments(cont[0])
        if M['m00'] > minObjectArea:
            # print(M['m00'])
            x, y, w, h = cv.boundingRect(cont[0])

    return x, y, w, h


"""
Morphological operations
"""

def morphOps(image, kernelSize):
    kernel = np.ones((kernelSize, kernelSize), np.uint8)
    # highlight the edge
    element2 = cv.getStructuringElement(cv.MORPH_RECT, (kernelSize, kernelSize))
    fix = cv.dilate(image, element2, iterations=1)
    # Fill in holes
    fix = cv.morphologyEx(fix, cv.MORPH_CLOSE, kernel)
    return fix


def rgbThreshold(img, upper, fill=None):
    if fill is None:
        fill = WHITE
    if len(img.shape) == 2:
        return img

    h, w, n = img.shape
    for i in range(h):
        for j in range(w):
            if img[i][j][0] > upper and img[i][j][1] > upper and img[i][j][2] > upper:
                img[i][j] = fill
            elif np.var(np.array(img[i][j])) < contrast:
                img[i][j] = fill

            
    return img

"""
preprocess the raw image to focus on the object area
"""
def preprocess(img):
    # downsample to speed up
    img = cv.resize(img, (int(0.3*raw_w), int(0.3*raw_h)))
    # filter the background and noise
    img = rgbThreshold(img, light_thresh)
    img_rgb = img
    # detect edges
    img = cv.Canny(img, 100, 150)
    # morphology
    img = morphOps(img, morphKernel)
    # detect the object
    x, y, w, h = findObjects(img)
    # crop the original image to the object area
    img = img_rgb[y:y + h, x:x + w]
    img = cv.resize(img, (resize_w, resize_h))

    return img

def predict(img, debug=False, k=5):
    tic = time.time()
    # load the model
    knn_test = cv.ml.KNearest_create()
    model = knn_test.load(model_path)

    if debug:
        Title_images = 'Original Image'
        Title_resized = 'Image Resized'
        cv.namedWindow( Title_images, cv.WINDOW_AUTOSIZE )
    #img = cv.resize(img, (raw_w, raw_h))
    #print("test image before pre-process = ", img.shape)    
    test_img = np.array(preprocess(img))
    #print("test image after pre-process = ", test_img.shape) 
    if debug:
        cv.imshow(Title_images, img)
        cv.imshow(Title_resized, test_img)
        key = cv.waitKey()
        if key==27:    # Esc key to stop
            return
    #test_img = test_img.astype(np.float32)
    test_img = test_img.flatten().reshape(1,resize_w*resize_h*dimension)
    test_img = test_img.astype(np.float32)
    #print("test image shape after transformation = ", test_img.shape)
    # test_label = np.int32(lines[i][1])

    ret, results, neighbours, dist = model.findNearest(test_img, k)
    toc = time.time()
    if debug:
        print("\nPrediction Time: " + str(toc-tic))
    return int(ret)

############################################
#              Train                       #
############################################
if train_flag:
    # this line reads in all images listed in the folder, and do preprocessing
    with open(trainDirectory + 'train.txt', 'r') as f:
        reader = csv.reader(f)
        lines = list(reader)
    for i in range(len(lines)):
        img = preprocess(cv.imread(trainDirectory + lines[i][0] + ".jpg", rgb))
        train.append(img)

        if debug:
            cv.imshow("window", img)
            cv.waitKey()
            cv.destroyWindow("window")


    # here we reshape each image into a long vector and ensure the data type is a float (which is what KNN wants)
    train_data = np.array(train).flatten().reshape(len(train),resize_w*resize_h*dimension)
    train_data = train_data.astype(np.float32)

    # read in training labels
    train_labels = np.array([np.int32(lines[i][1]) for i in range(len(lines))])

    ### Train classifier
    knn = cv.ml.KNearest_create()
    knn.train(train_data, cv.ml.ROW_SAMPLE, train_labels)
    # save the model
    knn.save("knnModel")

############################################
#              Test                        #
############################################
if test_flag:
    tic = time.time()
    # load the model
    knn_test = cv.ml.KNearest_create()
    model = knn_test.load("/home/allen/catkin_ws/src/team_x_maze/model/knnModel")
    # load test set
    with open(testDirectory + 'test.txt', 'r') as f:
        reader = csv.reader(f)
        lines = list(reader)

    correct = 0.0
    confusion_matrix = np.zeros((6,6))
    if debug:
        Title_images = 'Original Image'
        Title_resized = 'Image Resized'
        cv.namedWindow( Title_images, cv.WINDOW_AUTOSIZE )
    for i in range(len(lines)):
        original_img = cv.imread(testDirectory+ lines[i][0] + ".jpg", rgb)
        test_img = np.array(preprocess(original_img))

        if debug:
            cv.imshow(Title_images, original_img)
            cv.imshow(Title_resized, test_img)
            key = cv.waitKey()
            if key==27:    # Esc key to stop
                break
        test_img = test_img.flatten().reshape(1,resize_w*resize_h*dimension)
        test_img = test_img.astype(np.float32)

        test_label = np.int32(lines[i][1])

        ret, results, neighbours, dist = model.findNearest(test_img, k)

        if test_label == ret:
            print(str(lines[i][0]) + " Correct, " + str(ret))
            correct += 1
            confusion_matrix[np.int32(ret)][np.int32(ret)] += 1
        else:
            confusion_matrix[test_label][np.int32(ret)] += 1

            print(str(lines[i][0]) + " Wrong, " + str(test_label) + " classified as " + str(ret))
            print("\tneighbours: " + str(neighbours))
            print("\tdistances: " + str(dist))
    toc = time.time()
    print("\nRun Time: " + str(toc-tic))
    print("\n\nTotal accuracy: " + str(correct/len(lines)))
    avg_T = (toc-tic) / len(lines)
    print("Avg. Prediction time spent per img: %2.2f s" % avg_T)
    print(confusion_matrix)
