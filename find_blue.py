import cv2
import numpy as np
from matplotlib import pyplot as plt

img = cv2.imread('RawImg.png')


# Coordinates that you want to Perspective Transform
pts1 = np.float32([[105,0],[210,0],[0,200],[320,200]])
# Size of the Transformed Image
pts2 = np.float32([[0,0],[320,0],[0,200],[320,200]])

#cv2.circle(img,(105,0),3,(0,0,255),3)
#cv2.circle(img,(210,0),3,(0,0,0),3)
#cv2.circle(img,(0,200),3,(0,0,0),3)
#cv2.circle(img,(320,200),3,(0,0,0),3)

M = cv2.getPerspectiveTransform(pts1,pts2)
wrap = cv2.warpPerspective(img,M,(320,200))

blue = wrap[:,:,0]
green = wrap[:,:,1]
red = wrap[:,:,2]


keep = red #FIXME: adjust


blur = cv2.blur(keep,(10,10))
hist = cv2.calcHist([blur],[0],None,[256],[0,256])
_,thrsh = cv2.threshold(blur,70,255,cv2.THRESH_BINARY)
canny = cv2.Canny(thrsh,threshold1=25,threshold2=30)
lines = cv2.HoughLines(canny,1,np.pi/180,35)
print(lines)
if lines is not None :
    rho, theta = lines[2,0,:]
    
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a * rho
    y0 = b * rho

    x1 = int(x0 + 1000 * (-b))
    y1 = int(y0 + 1000 * (a))

    x2 = int(x0 - 1000 * (-b))
    y2 = int(y0 - 1000 * (a))

    cv2.line(wrap, (x1,y1), (x2,y2), (0,0,255), 2)

    theta_deg = theta * (180/np.pi)
    if theta_deg > 90:
        theta_deg = theta_deg - 180
    else:
        theta_deg = theta_deg
    
    print(theta_deg)
else:
    print("no line found")       

# Show Images
cv2.imshow('img',img)
cv2.imshow('blue', blue)
#cv2.imshow('green', green)
cv2.imshow('red', red)
cv2.imshow('blur',blur)
cv2.imshow('thrsh',thrsh)
cv2.imshow('canny', canny)
cv2.imshow('wrap',wrap)

# Plot Histogram
plt.figure()
plt.title("Pixel Histogram")
plt.xlabel("Bins")
plt.ylabel("Frequency")
plt.plot(hist)
plt.show()
cv2.waitKey(0)