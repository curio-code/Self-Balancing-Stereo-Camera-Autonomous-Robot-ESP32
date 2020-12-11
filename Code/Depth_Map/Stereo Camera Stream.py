#Author: Vivek Kumar Jaiswal
#Email Id: vivekjaiswal.iitdhn@gmail.com

import cv2
import urllib
import numpy as np
import time
from time import sleep

IntrinsicLeftCAM= np.array([[731.0650,0,0],[0,733.9671,0], [414.8902, 341.1697, 1.000]]).transpose()
IntrisicRightCAM = np.array([[740.4383,0,0],[0,  743.5752, 0], [445.3299,336.8437,1.0000]]).transpose()
R=np.array([[1.0000,0.0013,0.0055],[-0.0013,1.0000,0.0034],[-0.0055,-0.0034,1.0000]]).transpose()
T=np.array([[-93.8128,1.2146,0.8572]]).transpose()
distCoeff1 = np.array([[0.1593, -0.4790, 0, 0, 0]])
distCoeff2 = np.array([[0.1357, -0.3344, 0, 0, 0]])

imageSize=(800, 600)
(leftRectification, rightRectification, leftProjection, rightProjection, dispartityToDepthMap, leftROI, rightROI) = cv2.stereoRectify(IntrinsicLeftCAM, distCoeff1, IntrisicRightCAM, distCoeff2, imageSize, R, T, None, None, None, None, None, cv2.CALIB_ZERO_DISPARITY, 0)


leftMapX, leftMapY = cv2.initUndistortRectifyMap(
        IntrinsicLeftCAM, distCoeff1, leftRectification,
        leftProjection, imageSize, cv2.CV_32FC1)

rightMapX, rightMapY = cv2.initUndistortRectifyMap(
        IntrisicRightCAM, distCoeff2, rightRectification,
        rightProjection, imageSize, cv2.CV_32FC1)

win_size = 15
cn=1
stereoMatcher = cv2.StereoSGBM_create(uniquenessRatio = 1,disp12MaxDiff = 15, mode= 2, P1=8*cn*win_size^2, P2=32*cn*win_size^2)
stereoMatcher.setMinDisparity(-4)
stereoMatcher.setPreFilterCap(100)
stereoMatcher.setNumDisparities(152)
stereoMatcher.setBlockSize(1)
stereoMatcher.setSpeckleRange(32)
stereoMatcher.setSpeckleWindowSize(32)


rightCamURL='http://192.168.0.111/Img.jpg'
leftCamURL='http://192.168.0.113/Img.jpg'
retry_count = 0

while True:
    try:
        leftImgResp=urllib.request.urlopen(leftCamURL)
        rightImgResp=urllib.request.urlopen(rightCamURL)
        retry_count = 0
        leftImgNp=np.array(bytearray(leftImgResp.read()),dtype=np.uint8)
        rightImgNp=np.array(bytearray(rightImgResp.read()),dtype=np.uint8)
        leftImg=cv2.imdecode(leftImgNp,-1)
        rightImg=cv2.imdecode(rightImgNp,-1)
        # all the opencv processing is done here
        fixedLeft = cv2.remap(leftImg, leftMapX, leftMapY, cv2.INTER_LINEAR)
        fixedRight = cv2.remap(rightImg, rightMapX, rightMapY, cv2.INTER_LINEAR)

        grayLeft = cv2.cvtColor(fixedLeft, cv2.COLOR_BGR2GRAY)
        grayRight = cv2.cvtColor(fixedRight, cv2.COLOR_BGR2GRAY)

        depth = stereoMatcher.compute( grayLeft,  grayRight)

        DEPTH_VISUALIZATION_SCALE = 2048

        grayLeft = cv2.resize(grayLeft, (0, 0), None, .75, .75)
        grayRight = cv2.resize(grayRight, (0, 0), None, .75, .75)
        depth = cv2.resize(depth, (0, 0), None, .75, .75)

        numpy_horizontal= np.hstack((grayLeft/200, grayRight/200))
        numpy_horizontal1= np.concatenate((grayLeft/200, (depth / DEPTH_VISUALIZATION_SCALE)), axis=1)
        numpy_vertical = np.vstack((numpy_horizontal,numpy_horizontal1 ))

        cv2.imshow('Disparity Map',numpy_horizontal1)

        if ord('q')==cv2.waitKey(1):
            exit(0)
    except ConnectionResetError as e:
        if retry_count == 10000:
            raise e
        time.sleep(0.05)
        retry_count = retry_count +1
        print(retry_count)
