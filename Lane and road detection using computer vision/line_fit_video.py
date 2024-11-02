#importing libraries
import numpy as np
import cv2
import pickle
import matplotlib.pyplot as plt
import matplotlib.image as mtimg
from moviepy.editor import VideoFileClip
#import scripts
from line import Line
from line_fit import line_fit, final_viz, tune_fit, calc_curve, calc_vehicle_offset
from combined_threshold import combined_threshold
from perspective_transform import perspective_transform

#Global variables 
#(just to make the moviepy video annotation work)
with open('c_calibrate.p', 'rb') as f:
    save_dict = pickle.load(f)
mtx = save_dict['mtx']
dist = save_dict['dist']

window_size = 5  # how many frames for line smoothing
left_line = Line(n=window_size)
right_line = Line(n=window_size)
detected = False  # did the fast line fit detect the lines?
left_curve, right_curve = 0., 0.  # radius of curvature for left and right lanes
left_lane_inds, right_lane_inds = None, None  # for calculating curvature

#function-1
# MoviePy video annotation will call this function
#Annotate the input image with lane line markings
#Returns annotated image
def annotate_image(img_in):

    global mtx, dist, left_line, right_line, detected
    global left_curve, right_curve, left_lane_inds, right_lane_inds

    # Undistort, threshold, perspective transform
    undist = cv2.undistort(img_in, mtx, dist, None, mtx)
    img, abs_bin, mag_bin, dir_bin, hls_bin = combined_threshold(undist)
    binary_warped, binary_unwarped, m, m_inv = perspective_transform(img)



    #perform polynomial fit
    if not detected:
        #slow line fit
        ret = line_fit(binary_warped)
        left_fit = ret['left_fit']
        right_fit = ret['right_fit']
        nonzerox = ret['nonzerox']
        nonzeroy = ret['nonzeroy']
        left_lane_inds = ret['left_lane_inds']
        right_lane_inds = ret['right_lane_inds']

        # Get moving average of line fit coefficients
        left_fit = left_line.add_fit(left_fit)
        right_fit = right_line.add_fit(right_fit)

        #calculate curvature
        left_curve, right_curve = calc_curve(left_lane_inds, right_lane_inds, nonzerox, nonzeroy)

        detected = True
        # slow line fit always detects the line

    else :
        # implies detected == True
		# Fast line fit
        left_fit = left_line.get_fit()
        right_fit = right_line.get_fit()
        ret = tune_fit(binary_warped, left_fit, right_fit)
        left_fit = ret['left_fit']
        right_fit = ret['right_fit']
        nonzerox = ret['nonzerox']
        nonzeroy = ret['nonzeroy']
        left_lane_inds = ret['left_lane_inds']
        right_lane_inds = ret['right_lane_inds']

        # Only make updates if we detected lines in current frame
        if ret is not None:
            left_fit = ret['left_fit']
            right_fit = ret['right_fit']
            nonzerox = ret['nonzerox']
            nonzeroy = ret['nonzeroy']
            left_lane_inds = ret['left_lane_inds']
            right_lane_inds = ret['right_lane_inds']
            
            left_fit = left_line.add_fit(left_fit)
            right_fit = right_line.add_fit(right_fit)
            left_curve, right_curve = calc_curve(left_lane_inds, right_lane_inds, nonzerox, nonzeroy)
        else:
            detected = False

        vehicle_offset = calc_vehicle_offset(undist, left_fit, right_fit)

        # Perform final visualization on top of original undistorted image
        result = final_viz(undist, left_fit, right_fit, m_inv, left_curve, right_curve, vehicle_offset)


def annotate_video(input_file, output_file):
    video = VideoFileClip(input_file)
    annotated_video = video.fl_image(annotate_image)
    annotated_video.write_videofile(output_file, audio=False)

if __name__ == '__main__':
	# Annotate the video
	annotate_video('project_video.mp4', 'out.mp4')

	# Show example annotated image on screen for sanity check
	img_file = 'test_images/test2.jpg'
	img = mtimg.imread(img_file)
	result = annotate_image(img)
	result = annotate_image(img)
	result = annotate_image(img)
	plt.imshow(result)
	plt.show()
     
