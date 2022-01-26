#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Lab-1 Part-2 Option A
# Yinuo Wang, Praneeth Erwin Eddu
# Jan 25, 2022

import shutil
import cv2 as cv
import numpy as np
import os
import imutils
import configparser
from imutils.object_detection import non_max_suppression

# Print iterations progress
def printProgressBar(iteration, total, prefix='',
                     suffix='', decimals=1, length=100,
                     fill='█', printEnd="\r"):
    """
    Call in a loop to create terminal progress bar
    @params:
        iteration   - Required  : current iteration (Int)
        total       - Required  : total iterations (Int)
        prefix      - Optional  : prefix string (Str)
        suffix      - Optional  : suffix string (Str)
        decimals    - Optional  : positive number of decimals in percent complete (Int)
        length      - Optional  : character length of bar (Int)
        fill        - Optional  : bar fill character (Str)
        printEnd    - Optional  : end character (e.g. "\r", "\r\n") (Str)
    """
    percent = ("{0:." + str(decimals) + "f}").format(100 * (iteration / float(total)))
    filledLength = int(length * iteration // total)
    bar = fill * filledLength + '-' * (length - filledLength)
    print(f'\r{prefix} |{bar}| {percent}% {suffix}', end=printEnd)
    # Print New Line on Complete
    if iteration == total:
        print()

# print current openCV version
def printCVinfo():
    print('OpenCV ' + cv.__version__)


# Find all diamonds
# @tmp: template need to match
# @input_dir: input image directory
# @output_dir: output image directory
# @return dictionary
def find_diamond(template, input_dir, output_dir, show_image=False, 
                 threshold=0.95, verbose=False):
    """Find red diamonds in every image using template matching."""
    dictionary = {}
    # load all images from input dir and store them in array
    for i, file in enumerate(os.listdir(input_dir)):
        # load the images and convert to gray0
        img_rgb = cv.imread(input_dir + "/" + file)
        if img_rgb is None:
            print('Could not load img.')
        img_gray = cv.cvtColor(img_rgb, cv.COLOR_BGR2GRAY)

        rects = []
        diamond = []
        dictionary[file] = diamond
        # modify the template size in 0.1-1.5 scale of original size
        for scale in np.linspace(0.1, 1.5, 50):
            # resize the template
            re_template = imutils.resize(template, width=int(template.shape[1] * scale),
                                     height=int(template.shape[0] * scale))

            # store template width and height
            rw, rh = re_template.shape[::-1]

            # execute the match function
            res = cv.matchTemplate(img_gray, re_template, 5)

            # get the match result with a threshold
            
            (loc_y, loc_x) = np.where(res >= threshold)

            # There usually are many duplicated rectangles for the same target
            for pt in zip(loc_x, loc_y):
                rects.append((pt[0], pt[1], pt[0] + rw, pt[1] + rh))

        # merge near rectangles to a single one
        pick = non_max_suppression(np.array(rects))

        # draw the rectangles and output their coordinates
        for (start_x, start_y, end_x, end_y) in pick:
            cv.rectangle(img_rgb, (start_x, start_y), (end_x, end_y), (0, 0, 255), 1)
            if verbose:
                # Print file name and rectangle locations to terminal
                print('[' + file + ' ' + str(start_x) + ' ' + str(start_y) + ']')
            diamond.append([start_x, start_y])

        # write processed image into the output dir
        cv.imwrite(output_dir + "/" + file, img_rgb)

        if not verbose:
            # Show progress bar
            printProgressBar(i + 1, len(os.listdir(input_dir)), prefix='Progress:',
                            suffix='Complete', length=50)
        
        if show_image:
            # show each image
            cv.imshow('Display window', img_rgb)
            # wait the key 0 to close window
            cv.waitKey(0)
            cv.destroyAllWindows()

    return dictionary


if __name__ == '__main__':
    # Load in config params
    params = configparser.ConfigParser()
    params.read('config.ini')
    input_dir = str(params['TEMPLATE-MATCHING']['input_dir'])
    output_dir = str(params['TEMPLATE-MATCHING']['output_dir'])
    threshold = float(params['TEMPLATE-MATCHING']['threshold'])
    verbose = bool(params['TEMPLATE-MATCHING']['threshold'])
    
    # create output dir
    if os.path.isdir(output_dir) is True:
        shutil.rmtree(output_dir)
    os.mkdir(output_dir)

    # load template
    template = cv.imread('template.jpg', 0)

    # find diamonds in all images
    print("Running template matching on input directory: {}"
          .format(input_dir))
    dic = find_diamond(template, input_dir, output_dir,
                       threshold=threshold, verbose=verbose)
    
    print(dic)
    print("Template matching complete. Images are stored in directory name: {}"
          .format(output_dir))
