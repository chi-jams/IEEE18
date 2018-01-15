import cv2
import numpy as np

cap = cv2.VideoCapture(0);
while True:
	#capture a convert to grayscale
	_, img = cap.read()
	imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

	#convert to binary
	ret, thresh = cv2.threshold(imgray, 100, 255, cv2.THRESH_BINARY_INV)

	#erode and dilate
	kernel = np.ones((8,8), np.uint8)

	thresh = cv2.erode(thresh, kernel, iterations=1)
	cv2.imshow('eroded', thresh)
	thresh = cv2.dilate(thresh, kernel, iterations=1)
	cv2.imshow('dilated', thresh)

	im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	cv2.drawContours(img, contours, -1, (0,255,0), 3)
	cv2.imshow('dst',img)
	if cv2.waitKey(1) & 0xff == ord('q'):
	    cv2.destroyAllWindows()
	    break;

	    #https://docs.opencv.org/3.3.1/d7/d4d/tutorial_py_thresholding.html