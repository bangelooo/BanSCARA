# This script opens the webcam and creates a bounding box around objects of a specific color range in HSV

import cv2
from PIL import Image

from util import get_limits

# from  import get_limits

greenishColor = [160,209,126] # Color in BGR colorspace 

lowerLimit,upperLimit = get_limits(color = greenishColor) # Get limits for the Hue limits for the desired color

cap = cv2.VideoCapture(0) # Start the webcam

while True:
    ret, frame = cap.read() # Read frame from the webcam

    hsvImage = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV) # Convert from BGR to HSV colorspace

    mask = cv2.inRange(hsvImage,lowerLimit,upperLimit) # Pixels in this range will be shown, while everything else gets 'erased'

    mask_ = Image.fromarray(mask) # Convert image from numpy array into Pillow

    bbox = mask_.getbbox() # Pillow method to find bounding box 

    if bbox is not None:
        x1,y1,x2,y2 = bbox

        frame = cv2.rectangle(frame,(x1,y1),(x2,y2),(0,255,0),3)

    cv2.imshow('frame',frame) 

    if cv2.waitKey(1) & 0xFF == ord('q'):  # Stop video capture if the 'q' key is pressed
        break

cap.release()
cv2.destroyAllWindows()