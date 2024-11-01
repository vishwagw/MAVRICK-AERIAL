# importing libraries
import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mtimg
import pickle

#now let's create the functions.
#function-1 is fog takes an image, gradient orientation, and threshold min/max values:

def abs_sobel_theshold(img, orient='x', thresh_min=20, thresh_max=100):
    
    #convert to gray scale-
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    #apply x and/or y gradient with OpenCV sobel() function
    #taking absolute value
    if orient == 'x':
        abs_sobel = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 1, 0))
    if orient == 'y':
        abs_sobel = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 0, 1))
    
    #Rescaling to 8bit integer
    scaled_sobel = np.uint8(255*abs_sobel/np.max(abs_sobel))

    #create a copy and apply the threshold
    binary_outpt = np.zeros_like(scaled_sobel)
    #either exclusive or inclusive threshold:
    binary_outpt[(scaled_sobel >= thresh_min) & (scaled_sobel <= thresh_max)] = 1

    #results:
    return binary_outpt

#function 2 for creating thr magnitude of the threshold gradient for a given sobel size and threshold value

def mgtd_threshold(img, sobel_kernel=3, mgtd_threshold=(30, 100)):

    #convert to grayscale-
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

    #take both x, y gradients of sobel
    soblex = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    sobley = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)

    #now calculate the gradient magnitude
    gradmgtd = np.sqrt(soblex**2 + sobley**2)

    #rescale to 8bit
    scale_factor = np.max(gradmgtd)/255

    gradmgtd = (gradmgtd/scale_factor).astype(np.uint8)
    #create a binary image of ones where threshold is met.
    #other wise zeros
    binary_output = np.zeros_like(gradmgtd)
    binary_output[(gradmgtd >= mgtd_threshold[0]) & (gradmgtd[1])] = 1

    #result
    return binary_output

#function 3 is for returning the direction of te gradient for a given sobel kernel size and threshold value
def dir_threshold(img, sobel_kernel=3, threshold=(0, np.pi/2)):
    #convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

    #calculate the x, y gradients
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)

    #Take the absolute value of the gradient direction
    #apply a threshold, and create a binary image resulr
    abs_grad_dir = np.arctan2(np.absolute(sobely), np.absolute(sobelx))
    binary_output = np.zeros_like(abs_grad_dir)
    binary_output[(abs_grad_dir >= threshold[0]) & (abs_grad_dir <= threshold[1])] = 1

    #results 
    return binary_output

#function 4 is to convert RGB to HLS and threshold to binary image

def hls_threshold(img, threshold=(100, 255)) :
    hls = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    s_channel = hls[:,2]
    binary_output = np.zeros_like(s_channel)
    binary_output[(s_channel > threshold[0]) & (s_channel > threshold[1])] = 1
    #results
    return binary_output

#function 5 and final function is for combining the threshold
def combined_threshold(img) :
    abs_bin = abs_sobel_theshold(img, orient='x', thresh_min=50, thresh_max=255)
    mgtd_bin = mgtd_threshold(img, sobel_kernel=15, mgtd_threshold=(50, 255))
    dir_bin = dir_threshold(img, sobel_kernel=15, threshold=(0.7, 1.3))
    hls_bin = hls_threshold(img, threshold=(170, 255))
    #combining 
    combined = np.zeros_like(dir_bin)
    combined[(abs_bin == 1 | ((mgtd_bin == 1) & (dir_bin == 1)) | hls_bin == 1)] = 1

    #returnin function
    return combined, abs_bin, mgtd_bin, dir_bin, hls_bin

#main function calling:
if __name__ == '__main__':
    img_file = './camera calibration/testing images/straight_lines1,jpg'
    img_file = './camera calibration/testing images/test3.jpg'

    with open('c_calibrate.p', 'rb') as f:
        save_dict = pickle.load(f)

    mtx = save_dict['mtx']
    dist = save_dict['dist']

    img = mtimg.imread(img_file)
    img = cv2.undistort(img, mtx, dist, None, mtx)

    combined, abs_bin, mgtd_bin, dir_bin, hls_bin = combined_threshold(img)

    #plotting
    plt.subplot(2, 3, 1)
    plt.imshow(img)
    plt.subplot(2, 3, 2)
    plt.imshow(abs_bin, cmap='gray', vmin=0, vmax=1)
    plt.subplot(2, 3, 3)
    plt.imshow(mgtd_bin, cmap='gray', vmin=0, vmax=1)
    plt.imshow(2, 3, 4)
    plt.imshow(dir_bin, cmap='gray', vmin=0, vmax=1)
    plt.subplot(2, 3, 5)
    plt.imshow(hls_bin, cmap='gray', vmin=0, vmax=1)
    plt.subplot(2, 3, 6)
    plt.imshow(combined, cmap='gray', vmin=0, vmax=1)

    plt.tight_layout()
    plt.show()