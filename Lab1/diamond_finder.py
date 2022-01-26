# !/usr/bin/env python
"""Template matching."""

import cv2 as cv
import shutil
import numpy as np
import os
from imutils.object_detection import non_max_suppression
from matplotlib import pyplot as plt


def resize_img(img, interpolation=cv.INTER_AREA, scale=100, show_img=False):
    # if dim[0] > template.shape[0] or dim[1] > template.shape[1]:
    #     print("please make sure to enter dimensions smaller than the original image")
    #     return None
    x = int(img.shape[0] * scale / 100)
    y = int(img.shape[1] * scale / 100)
    img = cv.resize(img, (x, y), interpolation=interpolation)
    print("New Image size = ", img.shape)
    if show_img:
        cv.imshow('Resized Image', img)
        cv.waitKey(0)
        cv.destroyAllWindows()
    return img

def find_diamond(template, src_img_gray, src_img, thresh=0.80, mask=None, show_img=False):
    # 'cv.TM_CCOEFF_NORMED' uses inverse thresh

    if len(template.shape[::-1]) >= 2:
        w, h = template.shape[0], template.shape[1]
    else:
        raise Exception("Please check the dimensions of the template image")

    result = cv.matchTemplate(src_img_gray, template, cv.TM_CCOEFF_NORMED) 

    # threshold = thresh #  0.507
    pick_len = 11
    while (not pick_len <= 10):
        loc = np.where(result >= thresh)
        clone = src_img.copy()

        print("\nTotal matches before applying non-max suppresion = {}"
                .format(len(loc[0])))
        
        # There usually are many duplicated rectangles for the same target
        rects = []
        for pt in zip(*loc[::-1]):
            rects.append((pt[0], pt[1], pt[0] + w, pt[1] + h))

        # merge near rectangles to a single one
        pick = non_max_suppression(np.array(rects))
        pick_len = len(pick)
        print("Total matches after applying non-max suppresion = {}"
                .format(len(pick)))
        for (start_x, start_y, end_x, end_y) in pick:
            cv.rectangle(clone, (start_x, start_y), (end_x, end_y), (0, 0, 255), 1)
        thresh += 0.03
    # Display image if needed
    if show_img:
        cv.imshow('final output with thresh = {}'.format(thresh), clone)
        cv.imshow('result', result)
        cv.waitKey(0)
        

    cv.destroyAllWindows()


def printCVinfo():
    print('OpenCV ' + cv.__version__)

def matchImage(show_img=False):
    # load template
    template = cv.imread('template-3.png', 0)
    # store template width and height
    w, h = template.shape[::-1]
    # cv.imshow('Template', template)

    # create output dir
    if os.path.isfile("output_imgs"):
        shutil.rmtree("output_imgs")
    os.mkdir("output_imgs")

    # load all images from input dir and store them in array
    for file in os.listdir('input_imgs'):

        # load the images and convert to gray0
        img_rgb = cv.imread('input_imgs/' + file)
        if img_rgb is None:
            print('Could not load img.')
        img_gray = cv.cvtColor(img_rgb, cv.COLOR_BGR2GRAY)

        # execute the match function
        res = cv.matchTemplate(img_gray, template, 5)
        # get the match result with a threshold
        threshold = 0.8
        (loc_y, loc_x) = np.where(res >= threshold)

        # There usually are many duplicated rectangles for the same target
        rects = []
        for pt in zip(loc_x, loc_y):
            rects.append((pt[0], pt[1], pt[0] + w, pt[1] + h))
        # merge near rectangles to a single one
        pick = non_max_suppression(np.array(rects))

        # draw the rectangles and output their coordinates
        for (start_x, start_y, end_x, end_y) in pick:
            cv.rectangle(img_rgb, (start_x, start_y), (end_x, end_y), (0, 0, 255), 1)
            print('[' + file + ' ' + str(start_x) + ' ' + str(start_y) + ']')

        # write processed image into the output dir
        cv.imwrite('output_imgs/'+file, img_rgb)

        if show_img:
            # show each image
            cv.imshow('Detected', img_rgb)
            # wait the key 0 to close window
            cv.waitKey(0)
            cv.destroyAllWindows()

def apply_color_mask(image, lower_thresh, upper_thresh, use_HSV=False):
    
    if use_HSV:
        image = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    mask = cv.inRange(image, lower_thresh, upper_thresh)
    image = cv.bitwise_and(image, image, mask=mask)
    return image

if __name__ == '__main__':
    printCVinfo()
    # Load template image
    template = cv.imread('template.jpg', cv.IMREAD_UNCHANGED)
    template = cv.resize(template, (30, 30), interpolation=cv.INTER_AREA)
    template_gray = cv.cvtColor(template, cv.COLOR_BGR2GRAY)
    src_img = cv.imread('input_imgs/frame0001.jpg', cv.IMREAD_UNCHANGED)
    
    src_img_gray = cv.cvtColor(src_img, cv.COLOR_BGR2GRAY)
    
    # Gaussian Blur image
    alpha = 25
    kernel_size = 5
    kernel = np.ones((kernel_size, kernel_size), np.float32) / alpha
    src_img_gray_blur = cv.filter2D(src_img_gray, -1, kernel)

    # Sobel Blur
    sobel = cv.Sobel(src=src_img, ddepth=cv.CV_64F, dx=2, dy=1, ksize=1)
    
    # Edge detector
    src_img_canny = cv.Canny(image=src_img, threshold1=100, threshold2=200)

    src_img_masked = apply_color_mask(src_img, np.array([30, 30, 60]),
                               np.array([80, 80, 130]))
    template_masked = apply_color_mask(template, np.array([40, 40, 80]),
                                np.array([100, 100, 130]))

    # find_diamond(template_masked, src_img_masked, src_img, thresh=0.50,
    #              show_img=True)

    find_diamond(template, src_img, src_img, thresh=0.50,
                 show_img=True)
    # matchImage()
