import numpy as np
from scipy import linalg as la
import triangulateHelpers as tri
import os
import glob
import cv2

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
keypointFile = '/home/andylai2/ORB_SLAM2/Outputs/KeyPoints.txt'
matchesFile = '/home/andylai2/ORB_SLAM2/Outputs/threeFrameMatch.txt'

keypointXCoords = []
keypointYCoords = []
matchIdcs = []

# read all keypoints
with open (keypointFile,'r') as kF:
	for line in kF:
		kp = line.split()
		kpX = list(map(float, kp[::2]))
		kpY = list(map(float, kp[1::2]))
		keypointXCoords.append(np.array(kpX))
		keypointYCoords.append(np.array(kpY))

# read all matches
with open(matchesFile,'r') as mF:
	for line in mF:
		idcs = line.split()
		idcs = list(map(int, idcs))
		matchIdcs.append(idcs)

print(len(matchIdcs))
img0 = readImgFloat(imagePaths[50])
img1 = readImgFloat(imagePaths[51])
img2 = readImgFloat(imagePaths[52])

# draw keypoint matches
for i in range(len(matchIdcs[1])):
	idx2 = matchIdcs[1][i]
	if idx2 > -1:
		x = keypointXCoords[0][i]
		y = keypointYCoords[0][i]
		drawKeyPoint(img0,x,y,COLOR_RED)

		idx1 = matchIdcs[0][i]
		x = keypointXCoords[1][idx1]
		y = keypointYCoords[1][idx1]
		drawKeyPoint(img1,x,y,COLOR_RED)

		x = keypointXCoords[2][idx2]
		y = keypointYCoords[2][idx2]
		drawKeyPoint(img2,x,y,COLOR_RED)


for i in range(2):
	cv2.imshow('image',img0)
	cv2.waitKey(0)
	cv2.imshow('image',img1)
	cv2.waitKey(0)
	cv2.imshow('image',img2)
	cv2.waitKey(0)