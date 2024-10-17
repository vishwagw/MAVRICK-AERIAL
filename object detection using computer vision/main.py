#step -1 importing the libraires.
import cv2
import math
from ultralytics import YOLO

#step -2 video capturing function.
#starting the webcam
cap = cv2.VideoCapture(0) #defualt camera(0) is used to capture frames
#resolution 640x480
cap.set(3, 640) 
cap.set(4, 480)

#building the model:
model = YOLO('yolo-Weights/yolov8n.pt')

#defining the object classes:
classNames = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
              "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
              "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
              "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
              "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
              "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
              "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
              "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
              "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
              "teddy bear", "hair drier", "toothbrush"
              ]



#buildng the loop:
while True:
    success, img = cap.read()
    results = model(img, stream=True)

    #coordinates
    for r in results:
        boxes = r.boxes

        for box in boxes:
            #bounding box
            x1, y1, x2, y2 = box.xyxy[0]
            #converting to integer 
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

            #put box in cam
            cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)

            #cinfidence
            confidence = math.ceil((box.conf[0]*100))/100
            print("Confidence --->", confidence)

            #class name
            cls = int(box.cls[0])

            #object details 
            org = [x1, y1]
            font =cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 1
            color = (255, 0, 0)
            thickness = 2

            cv2.putText(img, classNames[cls], org, font, fontScale, color, thickness)
    
    cv2.imshow('WebCam', img)
    if cv2.waitKey(1) == ord('q') :
        break

cap.release()
cv2.destroyAllWindows()
