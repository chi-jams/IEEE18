#!/usr/bin/python3


#Cross Detection
#Returns the difference between the center of the camera and the center of the 
#cross feature as well as the difference between the intended orientation and
#the goal orientation

import cv2
import numpy as np
from math import atan2
from geometryHelpers import *

cap = cv2.VideoCapture(1)

kernel = np.ones((5,5) , np.uint8)

CENTER = [i/2 for i in cap.read()[1].shape[:2]]

while True:
    
    #take in image and clean it
    _, img = cap.read()
    img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur (img_gray, (5,5), 0)
    #threshold the image to binary
    ret, thresh = cv2.threshold(blur, 200, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    #ret, thresh = cv2.threshold(blur, 135, 255,cv2.THRESH_BINARY_INV)
    thresh = cv2.erode(thresh, kernel, iterations=1)
    thresh = cv2.dilate(thresh, kernel, iterations=1)
    #get the contours of the image
    _, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    #if contours exist
    if contours:
        ###################
        #   CONTOURS AND  #
        #     DEFECTS     #
        ###################

        #get the maximum contour
        cnt = contours[np.argmax(np.array([cv2.contourArea(cnt) for cnt in contours]), axis=0)]
        #get convex hull of max contour
        hull = cv2.convexHull(cnt,returnPoints = False)
        #get convexity defects between contour and convex hull
        defects = cv2.convexityDefects(cnt,hull)
        pts = [tuple(pt) for pt in cv2.boxPoints(cv2.minAreaRect(cnt))]
        '''
        for pt in pts:
            cv2.circle(img,tuple(pt),5,[255,0,0],-1)
        print(pts)
        '''
        mid_pts = []
        while pts:
            first = pts.pop()
            if not pts:
                break
            second = min(pts, key=lambda p: length_squared(p, first) )
            pts.remove(second) 
            mid_pts.append(midpoint(first, second, toInt = True)) 
            cv2.circle(img,mid_pts[-1],5,[0,255,0],-1)
        if len(mid_pts) > 1:
            cv2.line(img, mid_pts[0], mid_pts[1], [0,255,0], 2)

        #cv2.drawContours(img, [np.int0(pts)], 0, (0, 0, 255), 2)
        
        '''
        epsilon = 0.1 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)

        mid_pts = []
        approx = [tuple(approx[i][0]) for i in range(len(approx))]
        #for p in approx:
        #    #cv2.circle(img, p, 5, [0,0,255], -1)
        while approx:
            first = approx.pop()
            if not approx:
                break
            second = min(approx, key=lambda p: length_squared(p, first) )
            approx.remove(second) 
            mid_pts.append(midpoint(first, second, toInt = True)) 
            cv2.circle(img,mid_pts[-1],5,[255,0,0],-1)
        if len(mid_pts) > 1:
            cv2.line(img, mid_pts[0], mid_pts[1], [255,0,0], 2)
        '''

        '''
        #if there are defects
        if defects is not None:
            #get the 4 largest defects
            defects = defects[defects[:,0][:,3].argsort()[-4:]]
            pts = []
            dists = []
            for i in range(defects.shape[0]):
                #get start, end, defect pt, dist
                s,e,f,d = defects[i,0]
                start = tuple(cnt[s][0])
                end = tuple(cnt[e][0])
                #append relevant hull points 
                pts.append(start)
                pts.append(end)
                far = tuple(cnt[f][0])
                #get the projection point
                proj = projection(start, end, far)
                #convert to pixel-friendly values
                proj = proj.astype(np.int32)
                #save shortline distance
                dists.append(length_squared(proj, far)) 
                #draw
                cv2.line(img,far,tuple(proj),[255,0,0],2)
                cv2.line(img,start,end,[0,255,0],2)
                cv2.circle(img,far,5,[0,0,255],-1)
            end_pts = []
            
            if len(dists) > 2:
                if all(dists[i] > 250 for i in range(len(dists))):
                    ##############
                    # DRAW CROSS #
                    ##############

                    #get halfway points between pairs of convex hull points
                    while pts:
                        first = pts.pop()
                        second = min(pts, key=lambda p: length_squared(p, first) )
                        pts.remove(second)    
                        end_pts.append(midpoint(first, second,toInt = True)) 
                        cv2.circle(img,end_pts[-1],5,[255,0,0],-1)
                    
                    #get winding order of points and draw intersection to get center
                    #and angle
                    end_pts.sort(key=lambda p: atan2(p[1] - CENTER[1], p[0] - CENTER[0]))
                    if len(end_pts) > 2:
                        cv2.line(img, end_pts[0],end_pts[2],[0,255,0],2)
                    if len(end_pts) > 3:
                        cv2.line(img, end_pts[1],end_pts[3],[0,255,0],2)
                        #get center
                        center = seg_intersect(end_pts[0], end_pts[2], 
                                               end_pts[1], end_pts[3], 
                                               toInt = True)
                        #get topmost pt
                        ref_pt = max(end_pts, key=lambda p: p[1] - center[1])
                        #get angle
                        angle = int((atan2(ref_pt[1] - center[1], 
                                       ref_pt[0] - center[0]) * 180 / 3.14159) - 90)
                        cv2.putText(img, str(angle), (100,400),
                                    cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255))
                        cv2.line(img, (center[0], 0), tuple(center), [0,0,255], 2)
                        cv2.circle(img, tuple(center), 5, [0,0,255], -1)

        '''
    cv2.imshow("cap", img);
    cv2.imshow("threshold", thresh)
    if cv2.waitKey(1) & 0xff == ord('q'):
        cv2.destroyAllWindows()
        break;
