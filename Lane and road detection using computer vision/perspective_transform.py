#importing librariea 
import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mtimg
import pickle 
from combined_threshold import combined_threshold

#function for executing the perspective transform-
def perspective_transform(img):
    img_size = (img.shape[1], img.shape[0])

    src = np.float32(
        [[200, 720],
		[1100, 720],
		[595, 450],
		[685, 450]])
    dst = np.float32(
        [[300, 720],
		[980, 720],
		[300, 0],
		[980, 0]])
    
    m = cv2.getPerspectiveTransform(src, dst)
    m_inv = cv2.getPerspectiveTransform(dst, src)

    warped = cv2.warpPerspective(img, m, img_size, flags=cv2.INTER_LINEAR)
    unwarped = cv2.warpPerspective(warped, m_inv, (warped.shape[1], warped.shape[0]))

    return warped, unwarped, m, m_inv

if __name__ == '__main__':
    img_file = './camera calibration/testing images/test3.jpg'

    with open('c_calibrate.p', 'rb') as f:
        save_dict = pickle.load(f)
    mtx = save_dict['mtx']
    dist = save_dict['dist']

    img = mtimg.imread(img_file)
    img = cv2.undistort(img, mtx, dist, None, mtx)

    img, abs_bin, mgtd_bin, dir_bin, hls_bin = combined_threshold(img)

    warped, unwarped, m , m_inv = perspective_transform(img)

    #plotting and visual output
    plt.imshow(warped, cmap='gray', vmin=0, vmax=1)
    plt.show()

    plt.imshow(unwarped, cmap='gray', vmin=0, vmax=1)
    plt.show()