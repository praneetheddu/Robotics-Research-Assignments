# !/usr/bin/env python
"""Template matching."""

import cv2 as cv
from matplotlib import pyplot as plt

def resize_template(template, interpolation=cv.INTER_AREA, scale=100, show_img=False):
    # if dim[0] > template.shape[0] or dim[1] > template.shape[1]:
    #     print("please make sure to enter dimensions smaller than the original image")
    #     return None
    x = int(template.shape[0] * scale / 100)
    y = int(template.shape[1] * scale / 100)
    resized_img = cv.resize(template, (x, y), interpolation=interpolation)
    print("New Image size = ", resized_img.shape)
    if show_img:
        cv.imshow('Resized Image', resized_img)
        cv.waitKey(0)
        cv.destroyAllWindows()
    return resized_img

def template_matching(template, src_img):
    methods = ['cv.TM_CCOEFF', 'cv.TM_CCOEFF_NORMED', 'cv.TM_CCORR',
            'cv.TM_CCORR_NORMED', 'cv.TM_SQDIFF', 'cv.TM_SQDIFF_NORMED']
    w, h = template.shape[::-1]
    print(w, h)
    print(src_img.shape)
    for method in methods:
        method = eval(method)
        print("method = ", method)
        res = cv.matchTemplate(src_img, template, method)
        min_val, max_val, min_loc, max_loc = cv.minMaxLoc(res)
        if method in [cv.TM_SQDIFF, cv.TM_SQDIFF_NORMED]:
            top_left = min_loc
        else:
            top_left = max_loc
        bottom_right = (top_left[0] + w, top_left[1] + h)
        cv.rectangle(src_img, top_left, bottom_right, 255, 2)
        plt.subplot(121), plt.imshow(res, cmap='gray')
        plt.title('Matching Result'), plt.xticks([]), plt.yticks([])
        plt.subplot(122), plt.imshow(src_img, cmap='gray')
        plt.title('Detected Point'), plt.xticks([]), plt.yticks([])
        plt.suptitle(methods[method])
        plt.show()

def main():
    template = cv.imread('template-2.png', 0)
    test_img = cv.imread('input_imgs/frame0000.jpg', 0)
    template = resize_template(template, interpolation=cv.INTER_AREA, scale=20, show_img=False)
    template_matching(template, test_img)


main()
