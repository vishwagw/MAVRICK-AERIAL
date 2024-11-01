#import libraries
import cv2
import pickle
import matplotlib.pyplot as plt
import matplotlib.image as mtimg
import numpy as np

#creating the the function for camera calibration
def camera_calibrate():
    # Mapping each calibration image to number of checkerboard corners
	# Everything is (9,6) for now
	# inserting the calibration matrix using check boards in input data folder.
    obj_dict = {
		1: (9, 5),
		2: (9, 6),
		3: (9, 6),
		4: (9, 6),
		5: (9, 6),
		6: (9, 6),
		7: (9, 6),
		8: (9, 6),
		9: (9, 6),
		10: (9, 6),
		11: (9, 6),
		12: (9, 6),
		13: (9, 6),
		14: (9, 6),
		15: (9, 6),
		16: (9, 6),
		17: (9, 6),
		18: (9, 6),
		19: (9, 6),
		20: (9, 6),
	}

    #object points and corners list in input images
    obj_list = []
    corners_list = []

    #model will search through all the images to find corners.
    for k in obj_dict :
        nx, ny = obj_dict[k]

        #preparing the object points such- (0, 0 ,0) , (1, 0, 0)
        obj_p = np.zeros((nx*ny, 3), np.float32)
        obj_p[:,:2] = np.mgrid[0:nx, 0:ny].T.reshape(-1,2)

        #listing calibration input images(checkboard images)
        file_name = 'E:\Project MAVRICK\drone programming\lane or road detection model for an autonomous vehicle\camera calibration\calibration data set images\calibration%s.jpg' % str(k)
        img = cv2.imread(file_name)

        #lets convert the input calibration images into grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

        #find the corners in the checkboard images both x and y axis
        ret, corners = cv2.findChessboardCorners(gray, (nx, ny), None)

        #save and draw the corners if model find them:
        if ret == True:
            #saving object points and corresponding corners
            obj_list.append(obj_p)
            corners_list.append(corners)

            #Draw and display the corners
			#cv2.drawChessboardCorners(img, (nx, ny), corners, ret)
			#plt.imshow(img)
			#plt.show()
			#print('Found corners for %s' % fname)
        else:
            print('Warning : ret = %s for %s' % (ret, file_name))

    #calibrate camera and undistort a test image 
    img = cv2.imread('./camera calibration/test images/straight_lines1.jpg')
    img_size = (img.shape[1], img.shape[0])
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_list, corners_list, img_size, None, None)

    return mtx, dist

if __name__ == '__main__':
    mtx, dist = camera_calibrate()
    save_dist = {'mtx': mtx, 'dist': dist}
    with open('c_calibrate.p', 'wb') as f:
        pickle.dump(save_dist, f)

    #undistort example calibration image
    img = mtimg.imread('calibration dataset images/calibration5.jpg')
    plt.imshow(dist)
    plt.savefig('./output_images/undistort_calibration.png')
    
