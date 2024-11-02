#importing libraries
import numpy as np
import pickle
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as ptimg
#importing scripts
from combined_threshold import combined_threshold
from perspective_transform import perspective_transform

def line_fit(binary_warped):
    #assuming yo have created a warped binary image called "binary_warped"
    #take a histogram of the botttom half of the image
    histogram = np.sum(binary_warped[binary_warped.shape[0]//2:,:], axis=0)

    #create an output image to draw on and visualize the result
    out_img = (np.dstack((binary_warped, binary_warped, binary_warped))*255).astype('uint8')
    #find the peak of the left and right halves of the histogram
    #These will be the starting point for the left and right lines
    mid_points = np.int(histogram.shape[0]/2)
    leftx_base = np.argmax(histogram[100:mid_points]) + 100
    rightx_base = np.argmax(histogram[mid_points:-100]) + mid_points

    # choose the number of sliding windows
    nwindows = 9
    #set height of windows
    Windows_height = np.int(binary_warped.shape[0]/nwindows)
    #identify the x and y positions of all nonzero pixels in the image
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    #curent position to be updated for each window
    leftx_current = leftx_base
    rightx_current = rightx_base
    #set the wdith of the window +/- marging
    margin = 100
    #set minimum number of pixels found to recenter window
    minpix = 50
    #create empty lists to recieve left and right lane pixel in indices
    left_lane_inds = []
    right_lane_inds = []

    #step through the window one by one
    for window in range(nwindows):
        #identify windows boundaries in x and y / also left and right
        win_y_low = binary_warped.shape[0] - (window+1)*Windows_height
        win_y_high = binary_warped.shape[0] - window*Windows_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin

        #now lets draw the windows on thw visualization image
        cv2.rectangle(out_img,(win_xleft_low, win_y_low), (win_xleft_high, win_y_high),(0, 255, 0), 2)
        cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 255, 0), 2)
        #identify the nonzero pixels in x and y within the window margin'
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy <= win_y_high) & (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high )).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low ) & (nonzerox < win_xright_high)).nonzero()[0]

        #now lets append following indices to the lists
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)
        # now lets make a IF statements.
        #if you found > minpixels, the necenter next window on their mean position
        if len(good_left_inds) > minpix:
            leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:
            rightx_current = np.int(np.mean(nonzerox(good_right_inds)))

    #concatenate the array of indices
    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)
    
    # extract left and right line pixel positions
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]

    #fit a second order polynominal to each
    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)

    #return a dict of relevant variables
    ret = {}
    ret['left_fit'] = left_fit
    ret['right_fit'] = right_fit
    ret['nonzerox'] = nonzerox
    ret['nonzeroy'] = nonzeroy
    ret['out_img'] = out_img
    ret['left_lane_inds'] = left_lane_inds
    ret['right_lane_inds'] = right_lane_inds

    return ret

#next function
#when given a previously fit line, quickly try to find the line based on previous lines
def tune_fit(binary_warped, left_fit, right_fit):
    # Assume you now have a new warped binary image
	# from the next frame of video (also called "binary_warped")
	# It's now much easier to find line pixels
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    margin = 100
    left_lane_inds = ((nonzerox > (left_fit[0]*(nonzeroy**2) + left_fit[1]*nonzeroy + left_fit[2] - margin)) & (nonzerox < (left_fit[0]*(nonzeroy**2) + left_fit[1]*nonzeroy + left_fit[2] + margin)))
    right_lane_inds = ((nonzerox > (right_fit[0]*(nonzeroy**2) + right_fit[1]*nonzeroy + right_fit[2] - margin)) & (nonzerox < (right_fit[0]*(nonzeroy**2) + right_fit[1]*nonzeroy + right_fit[2] + margin)))

    #Again, extract left and right line pixel position 
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]

    #If we dont find enough relevant points return all None (this means error)
    min_inds = 10
    if lefty.shape[0] < min_inds or righty.shape[0] < min_inds:
        return None
    
    #now let's fit a second order polynominal to each
    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)

    #generating x and y values for plottinh
    ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0])
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    right_fitx = right_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]

    #return a dict of relevant variables
    ret = {}
    ret['left_fit'] = left_fit 
    ret['right_fit'] = right_fit
    ret['nonezerox'] = nonzerox
    ret['nonezeroy'] = nonzeroy
    ret['left_lane_inds'] = left_lane_inds
    ret['right_lane_inds'] = right_lane_inds

    return ret

def viz1(binary_warped, ret, save_file = None):

    #grab variables from ret dictionary
    left_fit = ret['left_fit']
    right_fit = ret['right_fit']
    nonzerox = ret['nonzerox']
    nonzeroy = ret['nonzeroy']
    out_img = ret['out_img']
    left_lane_inds = ret['left_lane_inds']
    right_lane_inds = ret['right_lane_inds']
    
    #generating x and y values for plotting
    ploty = np.linspace(0, binary_warped.shape[0]-1 + binary_warped.shape[0])
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]

    out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
    out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]
    plt.imshow(out_img)
    plt.plot(left_fitx, ploty, color='yellow')
    plt.plot(left_fitx, ploty, color='yellow')
    plt.xlim(0, 1280)
    plt.ylim(720, 0)
    if save_file is None:
        plt.show()
    else:
        plt.savefig(save_file)
    plt.gcf().clear()


#viz2
#Visualize the predicted lane lines with margin, on binary warped image
#save_file is a string representing where to save the image (if None, then just display)
def viz2(binary_warped, ret, save_file=None):
    #grab the variable from ret dictionary 
    left_fit = ret['left_fit']
    right_fit = ret['right_fit']
    nonezerox = ret['nonezerox']
    nonezeroy = ret['nonezeroy']
    left_lane_inds = ret['left_lane_inds']
    right_lane_inds = ret['right_lane_inds']

    #create an image to draw on and an image to show the selection window
    out_img = (np.dstack((binary_warped, binary_warped, binary_warped))*255).astype('uint8')
    window_img = np.zeros_like(out_img)

    out_img[nonezeroy[left_lane_inds], nonezerox[left_lane_inds]] = [255, 0, 0]
    out_img[nonezeroy[right_lane_inds], nonezerox[right_lane_inds]] = [0, 0, 255]

    #generate x and y  values for plotting
    ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0])
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]

    # Generate a polygon to illustrate the search window area
	# And recast the x and y points into usable format for cv2.fillPoly()
    margin = 100 # NOTE: Keep this in sync with *_fit()
    left_line_window1 = np.array([np.transpose(np.vstack([left_fitx-margin, ploty]))])
    left_line_window2 = np.array([np.flipud(np.transpose(np.vstack([left_fitx+margin, ploty])))])
    left_line_pts = np.hstack((left_line_window1, left_line_window2))
    right_line_window1 = np.array([np.transpose(np.vstack([right_fitx-margin, ploty]))])
    right_line_window2 = np.array([np.flipud(np.transpose(np.vstack([right_fitx+margin, ploty])))])
    right_line_pts = np.hstack((right_line_window1, right_line_window2))

    #draw the lane on to the warped blank image 
    cv2.fillPoly(window_img, np.int_([left_line_pts]), (0,255, 0))
    cv2.fillPoly(window_img, np.int_([right_line_pts]), (0,255, 0))
    result = cv2.addWeighted(out_img, 1, window_img, 0.3, 0)
    plt.imshow(result)
    plt.plot(left_fitx, ploty, color='yellow')
    plt.plot(right_fitx, ploty, color='yellow')
    plt.xlim(0, 1280)
    plt.ylim(720, 0)
    if save_file is None:
        plt.show()
    else:
        plt.savefig(save_file)
    plt.gcf().clear()


#next funcion is to:
#calculate radius of curvature in meters
def calc_curve(left_lane_inds, right_lane_inds, nonzerox, nonzeroy):
    
    y_eval = 719  # 720p video/image, so last (lowest on screen) y index is 719

    #define conversions in x and y from pixels space to meters 
    ym_per_pix = 30/720 # meters per pixel in y dimension
    xm_per_pix = 3.7/700 # meters per pixel in x dimension

    #now let's extract left and right line pixel positions
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]

    # Fit new polynomials to x,y in world space
    left_fit_cr = np.polyfit(lefty*ym_per_pix, leftx*xm_per_pix, 2)
    right_fit_cr = np.polyfit(righty*ym_per_pix, rightx*xm_per_pix, 2)
	# Calculate the new radii of curvature
    left_curverad = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
    right_curverad = ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])
	# Now our radius of curvature is in meters

    #return
    return left_curverad, right_curverad

#next function is to :
#Calculate vehicle offset from lane center, in meters
def calc_vehicle_offset(undist, left_fit, right_fit):

    # Calculate vehicle center offset in pixels
	bottom_y = undist.shape[0] - 1
	bottom_x_left = left_fit[0]*(bottom_y**2) + left_fit[1]*bottom_y + left_fit[2]
	bottom_x_right = right_fit[0]*(bottom_y**2) + right_fit[1]*bottom_y + right_fit[2]
	vehicle_offset = undist.shape[1]/2 - (bottom_x_left + bottom_x_right)/2

	# Convert pixel offset to meters
	xm_per_pix = 3.7/700 # meters per pixel in x dimension
	vehicle_offset *= xm_per_pix

	return vehicle_offset

#next function is to:
#Final lane line prediction visualized and overlayed on top of original image
def final_viz(undist, left_fit, right_fit, m_inv, left_curve, right_curve, vehicle_offset):

    #genrate x and y value for plotting
    ploty = np.linespace(0, undist.shape[0]-1, undist[0])
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + left_fit[2]

    # Create an image to draw the lines on
	#warp_zero = np.zeros_like(warped).astype(np.uint8)
	#color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
    color_warp = np.zeros((720, 1280, 3), dtype='uint8')  # NOTE: Hard-coded image dimensions

    # Recast the x and y points into usable format for cv2.fillPoly()
    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
    pts = np.hstack((pts_left, pts_right))

    #draw the lane onto the warped blank image 
    cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))

    #warp the blank back to original image space using inverse
    newwarp = cv2.warpPerspective(color_warp, m_inv, (undist.shape[1], undist.shape[0]))
	# Combine the result with the original image
    result = cv2.addWeighted(undist, 1, newwarp, 0.3, 0)

    # Annotate lane curvature values and vehicle offset from center
    avg_curve = (left_curve + right_curve)/2
    label_str = 'Radius of curvature: %.1f m' % avg_curve
    result = cv2.putText(result, label_str, (30, 40), 0, 1, (0, 0, 0), 2 , cv2.LINE_AA)

    label_str = 'UAV offset from lane center: %.1f m' % vehicle_offset
    result = cv2.putText(result, label_str, (30,70), 0, 1, (0,0,0), 2, cv2.LINE_AA)

    return result
