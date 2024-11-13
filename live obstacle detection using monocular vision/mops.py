# importing libraries
import cv2
import numpy
from time import time
from math import sin, cos, sqrt

# importing python scripts
from utilities import debug, save_image
from corner import CornerExtraction
from base import FeatureExtractorThread

def adaptive_non_maximal_suppression(keypoints, number, robustness):
    suppressed = []
    length = len(keypoints)
    for x in range(0, length):
        # Inizializzazione ad un valore massimo. In questo caso il massimo degli interi.
        radius = numpy.iinfo(numpy.int32)
        xi, yi = keypoints[x].pt[0], keypoints[x].pt[1]
        for y in range(0, length):
            xj, yj = keypoints[y].pt[0], keypoints[y].pt[1]
            if (xi != xj and yi != yj) and keypoints[x].response < robustness * keypoints[y].response:
                dist = sqrt((xj - xi) ** 2 + (yj - yi) ** 2)
                if dist < radius:
                    radius = dist
        suppressed.append([keypoints[x], radius])
    suppressed.sort(key=lambda item: item[1])
    suppressed = suppressed[-number:-1]
    return zip(*suppressed)[0]

# Function that implements the MOPS algorithm. Dappprima comes from a Gaussian pyramid
# with livelli number specified in the configuration file. Avremo quindi un immagine
# scalata per a number x di livelli. Ad ogni livello della piramide applichiamo l'estrazione
# degli Harris corners, nei quali si veranno i feature point MOPS. Ogni livello della pyramid
# It is processed concurrently with a separate thread. A volta terminata questa
# computation, an adaptive algorithm is applied that does not maximal suppression
# The pressure of the corner is not affidabili, or it has a "strength" that is not elevated.
# At the end of this procedure we have a view of the keypoints in the format provided by OpenCV,
# I have accessed the cv2.Keypoint interface. Conseguenza can calculate the descriptions
# be used successively.
# param image: l'image sulla quale operare.
# param config: oggetto Configuration to access the configuration parameters.
# return: i keypoint ed i descriptors calculated.
def detectAndCompute(image, config):
    mops, number, levels = config.get('mops'), config.get('points'), config.get('levels')
    keypoints, corners_tasks = [], []

    # Generazione della piramide gaussiana.
    pyramid = [numpy.float32(image)]
    for i in range(1, levels):
        pyramid.append(pyramid[i - 1])
        pyramid[i] = cv2.pyrDown(cv2.GaussianBlur(pyramid[i], (3, 3), 1))
        pyramid[i] = cv2.pyrUp(pyramid[i])

    # Estrazione degli Harris corner da tutti i livelli della piramide.
    for i in range(0, levels):
        task = CornerExtraction(pyramid[i], i, keypoints, config)
        corners_tasks.append(task)
        task.start()
    
    # Attendiamo che i task di estrazione siano terminati.
    for i in range(0, levels):
        corners_tasks[i].join()

    # Applichiamo l'Adaptive Non-Maximal Suppression.
    if mops['use_anms']:
        keypoints = adaptive_non_maximal_suppression(keypoints, number, mops['anms_robustness'])

    # Calcolo dei descriptors dati i keypoints in input.
    return keypoints, cv2.SIFT(number, levels).compute(image, keypoints)[1]

def drawKeypoints(image, keypoints, config):
    
    mops = config.get('mops')
    thickness, radius, color = mops['kp_thickness'], mops['kp_radius'], tuple(mops['kp_color'])
    copy = image.copy()
    for keypoint in keypoints:
        y, x = int(keypoint.pt[0]), int(keypoint.pt[1])
        cv2.circle(copy, (y, x), mops['kp_radius'], color, thickness)
        circ_x = numpy.int32(y + sin(keypoint.angle) * radius)
        circ_y = numpy.int32(x + cos(keypoint.angle) * radius)
        cv2.line(copy, (y, x), (circ_x, circ_y), color, thickness)
    return copy

# Class that presents a MOPS thread for calculating it in a manner consistent with the MOPS descriptions
# its a specific image in input.
class MOPSThread(FeatureExtractorThread):
    
    def __init__(self, image_path, name, config):
        super(MOPSThread, self).__init__(image_path, name)
        self.results = None
        self.config = config

    def run(self):
        start_time = time()
        keypoints, descriptors = detectAndCompute(self.image, self.config)
        debug("MOPS time: {} seconds.".format(time() - start_time))
        self.results = {'img': self.image, 'ext': self.extension, 'kp': keypoints, 'desc': descriptors}
        copy = drawKeypoints(self.image, keypoints, self.config)
        save_image(copy, self.name, self.extension)

# Override of the joining method of a thread. The join of the thread is now effective
    def join(self, timeout=None):
        
        super(MOPSThread, self).join(timeout)
        return self.results

