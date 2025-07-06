#!/usr/bin/env python
# -*- coding:utf-8 -*-
import cv2
import numpy as np
import glob

dis = 1.8
size = ()
object_points = [(3*i, 3*j, 0) for j in range(size[0]) for i in range(size[1])]
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

img = cv2.imread('captured.jpg')
# 获取画面中心点
#获取图像的长宽
h1, w1 = img.shape[0], img.shape[1]
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
_, gray = cv2.threshold(gray, 110, 255, cv2.THRESH_BINARY)
u, v = img.shape[:2]
# 找到棋盘格角点
ret, corners = cv2.findChessboardCorners(gray, size, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
# 如果找到足够点对，将其存储起来
print(ret)
if ret == True:
    # 在原角点的基础上寻找亚像素角点
    cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
    #追加进入世界三维点和平面二维点中
    # objpoints.append(objp)
    # imgpoints.append(corners)
    # 将角点在图像上显示
    cv2.drawChessboardCorners(img, size, corners, ret)

cv2.imwrite('points.jpg', gray)