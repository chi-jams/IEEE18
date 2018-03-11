import cv2
import numpy as np
from math import atan2
from geometryHelpers import *

class FeatureDetector:
    RAD_TO_DEG = 180 / 3.14159265
    def __init__(self, frameChecksPerFeature, debug = False):
        #video capture device
        self.cap = cv2.VideoCapture(1)
        #kernel for erode and dilate
        self.kernel = np.ones((5,5), np.uint8)
        #center of screen for comparisons
        self.SCREEN_CENTER = [i/2 for i in cap.read()[1].shape[:2]]
        self.frame_checks = frameChecksPerFeature
        self.debug = debug

    def read(self):
        #base img
        _, self.orig_img = self.cap.read()
        img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur (img_gray, self.kernel, 0)
        ret, thresh = cv2.threshold(blur, 60, 255,
                                    cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        thresh = cv2.erode(thresh, self.kernel, iterations=1)
        thresh = cv2.dilate(thresh, self.kernel, iterations=1)
        _, contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, 
                                          cv2.CHAIN_APPROX_SIMPLE)
        #contours exist, successful read
        if contours:
            #get the maximum contour
            self.max_contour = contours[np.argmax(
                                        np.array([cv2.contourArea(cnt) for cnt in contours]),
                                        axis=0)]
            #get convex hull of max contour
            self.hull = cv2.convexHull(self.max_contour, returnPoints = False)
            #get convexity defects between contour and convex hull
            self.defects = cv2.convexityDefects(self.max_contour,self.hull)
            return True
        #no contours found, failed read
        else:
            return False
    
    def crossDetect(self):
        if defects is not None:
            #get the 4 largest defects
            top_defects = self.defects[self.defects[:,0][:,3].argsort()[-4:]]
            pts = []
            dists = [] #BONUS: helps to determine if a cross exists
            for i in range(top_defects.shape[0]):
                #get start, end, defect pt, dist
                s,e,f,d = top_defects[i,0]
                start = tuple(self.max_countour[s][0])
                end = tuple(self.max_contour[e][0])
                #append relevant hull points 
                pts.append(start)
                pts.append(end)
                far = tuple(cnt[f][0])
                
                #BONUS: projection to determine if a cross exists
                #get the projection point
                proj = projection(start, end, far)
                #convert to pixel-friendly values
                proj = proj.astype(np.int32)
                #save shortline distance
                dists.append(length_squared(proj, far)) 
                
                #draw
                if debug:
                    cv2.line(orig_img,far,tuple(proj),[255,0,0],2)
                    cv2.line(orig_img,start,end,[0,255,0],2)
                    cv2.circle(orig_img,far,5,[0,0,255],-1)

            #BONUS: check if cross exists here **

            #get the midpoints of the cross lines
            mid_pts = []
            while pts:
                first = pts.pop()
                second = min(pts, key=lambda p: length_squared(first, p))
                pts.remove(second)
                mid_pts.append(midpoint(first, second, toInt = True))
                if debug:
                    cv2.circle(self.orig_img, mid_pts[-1], 5, [255,0,0], -1)
            #sort left to right
            mid_pts.sort(key=lambda p: p[0])
            if len(mid_pts) > 3:
                cross_center = seg_intersect(mid_pts[0], mid_pts[3],
                                             mid_pts[1], mid_pts[2], toInt = True)
                #x difference
                x_proj = proj(mid_pts[0], mid_pts[3], SCREEN_CENTER, toInt = True)
                x_error = length(cross_center, x_proj)
                x_error *= -1 if SCREEN_CENTER[0] - cross_center[0] < 0 else 1
           
                #y difference
                y_proj = proj(mid_pts[1], mid_pts[2], SCREEN_CENTER, toInt = True)
                y_error = length(cross_center, y_proj)
                y_error *= -1 if SCREEN_CENTER[1] - cross_center[1] < 0 else 1
           
                #r difference
                angle_ref = max(mid_pts, key=lambda p: p[1] - SCREEN_CENTER[1])
                r_error = (atan2(angle_ref[1] - cross_center[1],
                                angle_ref[0] - cross_center[0])
                                * RAD_TO_DEG) - 90
                print(x_error, y_error, r_error)
                return (x_error, y_error, r_error)



