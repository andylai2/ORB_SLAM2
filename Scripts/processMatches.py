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
poseFile = '/home/andylai2/ORB_SLAM2/Outputs/PosesSave.txt'

keypointX = []
keypointY = []
matchIdcs = []
poses = []

# read all keypoints
with open (keypointFile,'r') as kF:
	for line in kF:
		kp = line.split()
		kpX = list(map(float, kp[::2]))
		kpY = list(map(float, kp[1::2]))
		keypointX.append(np.array(kpX))
		keypointY.append(np.array(kpY))

# read all matches
with open(matchesFile,'r') as mF:
	for line in mF:
		idcs = line.split()
		idcs = list(map(int, idcs))
		matchIdcs.append(idcs)

# read poses
with open(poseFile,'r') as pF:
	for line in pF:
		if not line.split():
			poses.append([])
		else:
			elements = line.split()
			poses.append( list(map(float, elements)) )

pose0 = np.array(poses[50]).reshape((4,4))
pose1 = np.array(poses[51]).reshape((4,4))
pose2 = np.array(poses[52]).reshape((4,4))

Rcw0 = pose0[:3,:3]
tcw0 = pose0[:3,3]
Rcw1 = pose1[:3,:3]
tcw1 = pose1[:3,3]
Rcw2 = pose2[:3,:3]
tcw2 = pose2[:3,3]

img0 = readImgFloat(imagePaths[50])
img1 = readImgFloat(imagePaths[51])
img2 = readImgFloat(imagePaths[52])

# draw keypoints
for i in range(len(matchIdcs[0])):
	idx1 = matchIdcs[0][i]
	idx2 = matchIdcs[1][i]
	if idx2 > -1:
		drawKeyPoint(img0,keypointX[0][i],keypointY[0][i],COLOR_RED)
		drawKeyPoint(img1,keypointX[1][i],keypointY[1][i],COLOR_RED)
		drawKeyPoint(img2,keypointX[2][i],keypointY[2][i],COLOR_RED)

for i in range(2):
	cv2.imshow('image',img0)
	cv2.waitKey(0)
	cv2.imshow('image',img1)
	cv2.waitKey(0)
	cv2.imshow('image',img2)
	cv2.waitKey(0)