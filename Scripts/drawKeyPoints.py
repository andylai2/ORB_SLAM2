import os
import glob
import cv2 as cv
import numpy as np

def readImgFloat(imagePath):
    # Helper function to read images and convert to float between 0,1
    tmp = cv.imread(imagePath,1)
    img = tmp.astype(float) / 255.0
    return img
def drawKeyPoint(img,x,y,color):
	x = int(x)
	y = int(y)
	cv.circle(img,(x,y),3,color,-1)

# COLOR_ONE = (180,119,31)
# COLOR_TWO = (14,127,255)
# COLOR_THREE = (44,160,44)
# COLOR_FOUR = (40,39,214)
# COLOR_FIVE = (189,103,148)
# COLOR_SIX = (75,86,140)
# COLOR_SEVEN = (194,119,227)
# COLOR_EIGHT = (127,127,127)
# COLOR_NINE = (34,189,188)
# COLOR_TEN = (207,190,23)
COLOR_BLUE = (255,0,0)
COLOR_RED = (0,0,255)
COLOR_GREEN = (0,255,0)
COLOR_CYAN = (200,10,0)
COLOR_MAG = (10,0,200)
COLOR_YELLOW = (0,255,255)
COLOR_WHITE = (255,255,255)
colors = [COLOR_BLUE,COLOR_RED,COLOR_GREEN,COLOR_CYAN,COLOR_MAG,COLOR_YELLOW,COLOR_WHITE]



dataDir = '/home/andylai2/group-data/kitti_data/data_tracking_image_2/training/image_02/0001/'
imagePaths = sorted(glob.glob(os.path.join(dataDir, '*.png'), recursive=True))
maskKeypointFile = '/home/andylai2/ORB_SLAM2/Outputs/MaskKeyPoints.txt'
keypointFile = '/home/andylai2/ORB_SLAM2/Outputs/KeyPoints.txt'
keypointX = []
keypointY = []
objMaskKeypoints = []

with open(maskKeypointFile,'r') as f:
    for line in f:
        objKeypoints = line.split(';')
        maskKeypoints = []
        for obj in objKeypoints:
            kp = obj.split()
            masked = list(map(int,kp))
            maskKeypoints.append(masked)
        objMaskKeypoints.append(maskKeypoints)

with open (keypointFile,'r') as f:
	for line in f:
		kp = line.split()
		kpX = list(map(float, kp[::2]))
		kpY = list(map(float, kp[1::2]))
		keypointX.append(np.array(kpX))
		keypointY.append(np.array(kpY))

img0 = readImgFloat(imagePaths[200])
img1 = readImgFloat(imagePaths[201])

for i in range(len(objMaskKeypoints[0])):
	color = colors[i % 7]
	for j in objMaskKeypoints[0][i]:
		x = keypointX[0][j]
		y = keypointY[0][j]
		drawKeyPoint(img0,x,y,color)
for i in range(len(objMaskKeypoints[1])):
	color = colors[i % 7]
	for j in objMaskKeypoints[1][i]:
		x = keypointX[1][j]
		y = keypointY[1][j]
		drawKeyPoint(img1,x,y,color)

# for j in objMaskKeypoints[0][0]:
# 	x = keypointX[0][j]
# 	y = keypointY[0][j]
# 	drawKeyPoint(img,x,y,COLOR_BLUE)


# for j in objMaskKeypoints[0][1]:
# 	x = keypointX[0][j]
# 	y = keypointY[0][j]

cv.imshow('image',img0)
cv.waitKey(0)
cv.imshow('image',img1)
cv.waitKey(0)