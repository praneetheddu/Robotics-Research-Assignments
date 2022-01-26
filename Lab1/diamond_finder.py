#!/usr/bin/env python

import shutil
import cv2 as cv
import numpy as np
import os
import imutils
from imutils.object_detection import non_max_suppression


def printCVinfo():
    print('OpenCV ' + cv.__version__)


def matchImage():
    # load template
    template = cv.imread('template.jpg', 0)

    # create output dir
    # shutil.rmtree("output_imgs")
    os.mkdir("output_imgs")

    # load all images from input dir and store them in array
    for file in os.listdir('input_imgs'):

        # load the images and convert to gray0
        img_rgb = cv.imread('input_imgs/' + file)
        if img_rgb is None:
            print('Could not load img.')
        img_gray = cv.cvtColor(img_rgb, cv.COLOR_BGR2GRAY)

        rects = []
        # modify the template size in 0.1-1.5 scale of original size
        for scale in np.linspace(0.1, 1.5, 50):
            # resize the template
            re_temp = imutils.resize(template, width=int(template.shape[1] * scale),
                                     height=int(template.shape[0] * scale))

            # store template width and height
            rw, rh = re_temp.shape[::-1]

            # execute the match function
            res = cv.matchTemplate(img_gray, re_temp, 5)
            # get the match result with a threshold
            threshold = 0.95
            (loc_y, loc_x) = np.where(res >= threshold)

            # There usually are many duplicated rectangles for the same target
            for pt in zip(loc_x, loc_y):
                rects.append((pt[0], pt[1], pt[0] + rw, pt[1] + rh))

        # merge near rectangles to a single one
        pick = non_max_suppression(np.array(rects))

        # draw the rectangles and output their coordinates
        for (start_x, start_y, end_x, end_y) in pick:
            cv.rectangle(img_rgb, (start_x, start_y), (end_x, end_y), (0, 0, 255), 1)
            print('[' + file + ' ' + str(start_x) + ' ' + str(start_y) + ']')

        # write processed image into the output dir
        cv.imwrite('output_imgs/' + file, img_rgb)

        # show each image
        # cv.imshow('Detected', img_rgb)
        # wait the key 0 to close window
        # cv.waitKey(0)
        # cv.destroyAllWindows()


if __name__ == '__main__':
    matchImage()
