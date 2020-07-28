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

dataDir = '/home/andylai2/group-data/kitti_data/data_tracking_image_2/training/'
imageDir = os.path.join(dataDir, 'image_02/0000/')
maskDir = os.path.join(dataDir, 'masks/0000/')
imagePaths = sorted(glob.glob(os.path.join(imageDir, '*.png'), recursive=True))
maskPaths = sorted(glob.glob(os.path.join(maskDir, '*.png'), recursive=True))
keypointFile = '/home/andylai2/ORB_SLAM2/Outputs/KeyPoints.txt'
matchesFile = '/home/andylai2/ORB_SLAM2/Outputs/IniMatches.txt'
maskDir = '/home/andylai2/group-data/kitti'


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
		keypointXCoords.append(kpX)
		keypointYCoords.append(kpY)
		nI += 1

# read all matches
with open(matchesFile,'r') as mF:
	for line in mF:
		idcs = line.split()
		idcs = list(map(int, idcs))
		matchIdcs.append(idcs)

tmpAllMatches = np.asarray(matchIdcs) > -1
allMatches = np.logical_and.reduce(tmpAllMatches)
allMatchIdcs = np.nonzero(allMatches)[0]

img0 = readImgFloat(imagePaths[0])
img1 = readImgFloat(imagePaths[1])
img2 = readImgFloat(imagePaths[2])
img3 = readImgFloat(imagePaths[3])
img4 = readImgFloat(imagePaths[4])
mask0 = cv2.imread(maskPaths[0],0)
mask1 = cv2.imread(maskPaths[1],0)
mask2 = cv2.imread(maskPaths[2],0)
mask3 = cv2.imread(maskPaths[3],0)
mask4 = cv2.imread(maskPaths[4],0)

# Image 0:
for i in range(len(matchIdcs[0])):
	idx = matchIdcs[0][i]
	if idx > -1:
		x = keypointXCoords[0][i]
		y = keypointYCoords[0][i]
		if mask0[int(y),int(x)] > 0:
			drawKeyPoint(img0,x,y,COLOR_RED)
		else:
			drawKeyPoint(img0,x,y,COLOR_BLUE)

# Image 1: 
for idx in matchIdcs[0]:
	if idx > -1:
		x = keypointXCoords[1][idx]
		y = keypointYCoords[1][idx]
		if mask1[int(y),int(x)] > 0:
			drawKeyPoint(img1,x,y,COLOR_RED)
		else:
			drawKeyPoint(img1,x,y,COLOR_BLUE)
# Image 2: 
for idx in matchIdcs[1]:
	if idx > -1:
		x = keypointXCoords[2][idx]
		y = keypointYCoords[2][idx]
		if mask2[int(y),int(x)] > 0:
			drawKeyPoint(img2,x,y,COLOR_RED)
		else:
			drawKeyPoint(img2,x,y,COLOR_BLUE)

for i in range(1):
	cv2.imshow('image',img0)
	cv2.waitKey(0)
	cv2.imshow('image',img1)
	cv2.waitKey(0)
	cv2.imshow('image',img2)
	cv2.waitKey(0)
# for i in range(0,2):
	# img = readImgFloat(imgPaths[i])
