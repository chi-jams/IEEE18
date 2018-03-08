#!/usr/bin/python3

import cv2
import numpy as np
from math import atan2

cap = cv2.VideoCapture(0)

kernel = np.ones((5,5) , np.uint8)

CENTER = [i/2 for i in cap.read()[1].shape[:2]]

while True:
    
    #take in image and clean it
    _, img = cap.read()
    img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur (img_gray, (5,5), 0)
    #threshold the image to binary
    #ret, thresh = cv2.threshold(blur, 60, 255,cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    ret, thresh = cv2.threshold(blur, 135, 255,cv2.THRESH_BINARY_INV)
    thresh = cv2.erode(thresh, kernel, iterations=1)
    thresh = cv2.dilate(thresh, kernel, iterations=1)
    #get the contours of the image
    _, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    #if contours exist
    if contours:
        #get the maximum contour
        cnt = contours[np.argmax(np.array([cv2.contourArea(cnt) for cnt in contours]), axis=0)]
        #get convex hull of max contour
        hull = cv2.convexHull(cnt,returnPoints = False)
        #get convexity defects between contour and convex hull
        defects = cv2.convexityDefects(cnt,hull)

        #if there are defects
        if defects is not None:
            #get the 4 largest defects
            defects = defects[defects[:,0][:,3].argsort()[-4:]]
            pts = []
            for i in range(defects.shape[0]):
                #get start, end, defect pt, dist
                s,e,f,d = defects[i,0]
                start = tuple(cnt[s][0])
                end = tuple(cnt[e][0])
                pts.append(start)
                pts.append(end)
                far = tuple(cnt[f][0])
                cv2.line(img,start,end,[0,255,0],2)
                cv2.circle(img,far,5,[0,0,255],-1)

            end_pts = []
            #get halfway points between pairs of convex hull points
            while pts:
                first = pts.pop()
                second = min(pts, key=lambda p: (p[0] - first[0]) ** 2 + 
                                                 (p[1] - first[1]) ** 2)
                pts.remove(second)    
                end_pts.append((int((first[0] + second[0])/2), 
                                int((first[1] + second[1])/2)))
                cv2.circle(img,end_pts[-1],5,[255,0,0],-1)
            print(end_pts)
            
            #get winding order of points and draw intersection to get center
            #and angle
            end_pts.sort(key=lambda p: atan2(p[1] - CENTER[1], p[0] - CENTER[0]))
            print(end_pts)
            if len(end_pts) > 2:
                cv2.line(img, end_pts[0],end_pts[2],[0,255,0],2)
            if len(end_pts) > 3:
                cv2.line(img, end_pts[1],end_pts[3],[0,255,0],2)


    cv2.imshow("heyo", img);
    cv2.imshow("heyo2", thresh)
    if cv2.waitKey(1) & 0xff == ord('q'):
        cv2.destroyAllWindows()
        break;
