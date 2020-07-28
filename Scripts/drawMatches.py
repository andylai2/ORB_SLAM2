import os
import glob
import cv2
import numpy as np

def readImgFloat(imagePath):
    # Helper function to read images and convert to float between 0,1
    tmp = cv2.imread(imagePath,1)
    img = tmp.astype(float) / 255.0
    return img

def drawKeyPoint(img,x,y,color):
	x = int(x)
	y = int(y)
	cv2.circle(img,(x,y),3,color,-1)

COLOR_BLUE = (255,0,0)
COLOR_RED = (0,0,255)

dataDir = '/home/andylai2/group-data/kitti_data/data_tracking_image_2/training/image_02/0001/'
imagePaths = sorted(glob.glob(os.path.join(dataDir, '*.png'), recursive=True))
# keypointFile = '/home/andylai2/ORB_SLAM2/Outputs/KeyPoints.txt'
# matchesFile = '/home/andylai2/ORB_SLAM2/Outputs/IniMatches.txt'


keypointFile = '/home/andylai2/ORB_SLAM2/Outputs/KeyPointsf50f53.txt'
matchesFile = '/home/andylai2/ORB_SLAM2/Outputs/IniMatchesf50f53.txt'

img0 = readImgFloat(imagePaths[50])
img1 = readImgFloat(imagePaths[53])

keypointXCoords = []
keypointYCoords = []
matchIdcs = []
nI = 0
# read all keypoints
with open (keypointFile,'r') as kF:
	for line in kF:
		kp = line.split()
		kpX = list(map(float, kp[::2]))
		kpY = list(map(float, kp[1::2]))
		keypointXCoords.append(np.array(kpX))
		keypointYCoords.append(np.array(kpY))
		nI += 1

# read all matches
with open(matchesFile,'r') as mF:
	for line in mF:
		idcs = line.split()
		idcs = list(map(int, idcs))
		matchIdcs.append(np.array(idcs))


# tmpAllMatches = np.array(matchIdcs) > -1
# allMatches = np.logical_and.reduce(tmpAllMatches)
# allMatchIdcs = np.nonzero(allMatches)[0]


# Image 0:
for i in range(len(matchIdcs[0])):
	idx = matchIdcs[0][i]
	if idx > -1:
		x = keypointXCoords[0][i]
		y = keypointYCoords[0][i]
		drawKeyPoint(img0,x,y,COLOR_RED)
		# if idx in allMatchIdcs:
		# 	drawKeyPoint(img0,x,y,COLOR_RED)
		# else:
		# 	drawKeyPoint(img0,x,y,COLOR_BLUE)

# Image 1: 
for idx in matchIdcs[0]:
	if idx > -1:
		x = keypointXCoords[1][idx]
		y = keypointYCoords[1][idx]
		drawKeyPoint(img1,x,y,COLOR_RED)
		# if idx in allMatchIdcs:
		# 	drawKeyPoint(img1,x,y,COLOR_RED)
		# else:
		# 	drawKeyPoint(img1,x,y,COLOR_BLUE)

for i in range(3):
	cv2.imshow('image',img0)
	cv2.waitKey(0)
	cv2.imshow('image',img1)
	cv2.waitKey(0)