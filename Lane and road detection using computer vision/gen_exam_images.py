#importing libraries 
import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mtimg
import os
import pickle
from combined_threshold import combined_threshold
from perspective_transform import perspective_transform
from line_fit import line_fit, viz2, calc_curve, final_viz

#Read camera calibration coefficients
with open('c_calibrate.p', 'rb') as f:
    save_dict = pickle.load(f)
mtx = save_dict['mtx']
dist = save_dict['dist']

#create example pipeline images for test input images(testing images)
image_files = os.listdir('./camera calibration/testing images')
for image_file in image_files:
    out_image_file = image_file.split('.')[0] + '.png' #writing to png format
    img = mtimg.imread('./camera calibration/testing images/' + image_file)

    #undistort image
    img = cv2.undistort(img, mtx, dist, None, mtx)
    plt.imshow(img)
    plt.savefig('./camera calibration/output_images/undisort_' + out_image_file)

    #thrshold binary image
    img, abs_bin, mgtd_bin, dir_bin, hls_bin = combined_threshold(img)
    plt.imshow(img, cmap='gray', vmin=0, vmax=1)
    plt.savefig('./camera calibration/output_images/binary_' + out_image_file)
    
    #perspective tranform
    img, binary_unwarped, m, m_inv = perspective_transform(img)
    plt.imshow(img, cmap='gray', vmin=0, vmax=1)
    plt.savefig('./camera calibration/output_images/binary_' + out_image_file)

    #ploynominal fit
    ret = line_fit(img)
    left_fit = ret['left_fit']
    right_fit = ret['right_fit']
    nonzerox = ret['nonzerox']
    nonzeroy = ret['nonzeroy']
    left_lane_inds = ret['left_lane_inds']
    right_lane_inds = ret['right_lane_inds']
    save_file = './camera calibration/output_images/polyfit_' + out_image_file
    viz2(img, ret, save_file=save_file)


    # doing full annotation on original image
    # code is the same as in 'line_fit_video.py'
    orig = mtimg.imread('./camera calibration/testing images' + image_file)
    undist = cv2.undistort(orig, mtx, dist, None, mtx)
    left_curve, right_curve = calc_curve(left_lane_inds, right_lane_inds, nonzerox, nonzeroy)

    bottom_y = undist.shape[0] - 1
    bottom_x_left = left_fit[0]*(bottom_y**2) + left_fit[1]*bottom_y + left_fit[2]
    bottom_x_right = right_fit[0]*(bottom_y**2) + right_fit[1]*bottom_y + right_fit[2]
    vehicle_offset = undist.shape[1]/2 - (bottom_x_left + bottom_x_right)/2

    #meters per pixels in x dimension
    xm_per_pix = 3.7/700 
    vehicle_offset *= xm_per_pix

    img = final_viz(undist, left_fit, right_fit, m_inv, left_curve, vehicle_offset)
    plt.imshow(img)
    plt.savefig('./camera calibration/output_images/annotated_' + out_image_file)
    