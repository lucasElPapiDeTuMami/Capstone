from math import atan2
from types import CoroutineType
import cv2 as cv
import numpy as np
import time
from matplotlib import pyplot as plt
img = cv.imread('RawImg.png')

h = img.shape[0]
w = img.shape[1]
print(h)
print(w)
crop = img[30:h,:,:]
r = img[30:h,:,0]
blur = cv.blur(r,(5,5))
hist = cv.calcHist([blur],[0],None,[256],[0,256])
_,thrsh = cv.threshold(blur,200,255,cv.THRESH_BINARY)
canny = cv.Canny(thrsh,threshold1=15,threshold2=30)
lines = cv.HoughLines(canny,1,np.pi/180,1,None,0,0)

if lines is not None :
    rho, theta = lines[0,0,:]
    
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a * rho
    y0 = b * rho

    x1 = int(x0 + 1000 * (-b))
    y1 = int(y0 + 1000 * (a))

    x2 = int(x0 - 1000 * (-b))
    y2 = int(y0 - 1000 * (a))

    cv.line(crop, (x1,y1), (x2,y2), (0,0,255), 2)
        #print(theta * (180 / np.pi) )
        

        # convert to coordinate frame: hard left is -89, hard right +90
    theta_deg = theta * (180/np.pi)
    if theta_deg > 90:
        theta_deg = theta_deg - 180
    else:
        theta_deg = theta_deg
        

    print(theta_deg)
else:
    print("no line found")       

# Show Images
cv.imshow('img',img)
cv.imshow('r',r)
cv.imshow('blur',blur)
cv.imshow('thrsh',thrsh)
cv.imshow('canny', canny)
cv.imshow('crop',crop)
# Plot Histogram
plt.figure()
plt.title("Pixel Histogram")
plt.xlabel("Bins")
plt.ylabel("Frequency")
plt.plot(hist)
plt.show()
cv.waitKey(0)