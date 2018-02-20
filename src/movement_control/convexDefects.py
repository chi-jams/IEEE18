import cv2
import numpy as np

cap = cv2.VideoCapture(1)

kernel = np.ones((5,5) , np.uint8)

while True:

    _, img = cap.read()
    img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    img_gray = cv2.erode(img_gray, kernel, iterations=1)
    img_gray = cv2.dilate(img_gray, kernel, iterations=1)
    blur = cv2.GaussianBlur (img_gray, (5,5), 0)
    ret, thresh = cv2.threshold(blur, 0, 255,cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    something, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours is not None:
        for i in range(len(contours)):
            cnt = contours[i]
            hull = cv2.convexHull(cnt,returnPoints = False)
            defects = cv2.convexityDefects(cnt,hull)

            if defects is not None:
                for i in range(defects.shape[0]):
                    s,e,f,d = defects[i,0]
                    start = tuple(cnt[s][0])
                    end = tuple(cnt[e][0])
                    far = tuple(cnt[f][0])
                    cv2.line(img,start,end,[0,255,0],2)
                    cv2.circle(img,far,5,[0,0,255],-1)
    cv2.imshow("heyo", img);
    cv2.imshow("heyo2", thresh)
    if cv2.waitKey(1) & 0xff == ord('q'):
        cv2.destroyAllWindows()
        break;
