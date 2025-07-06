import cv2
import numpy as np

dis = 3
size = (7, 7)
object_points = [(3*i, 3*j, 0) for j in range(size[0]) for i in range(size[1])]
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

image = cv2.imread("captured.jpg")

# def find_corners(img):
#     # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#     params = cv2.SimpleBlobDetector_Params()
#     params.maxArea = 10000
#     params.minArea = 0.01
#     params.minDistBetweenBlobs = 0.01
#     blobDetector = cv2.SimpleBlobDetector_create(params)
#     ret, corners = cv2.findCirclesGrid(img, size, cv2.CALIB_CB_SYMMETRIC_GRID, blobDetector, None)
#     if ret:
#         cv2.cornerSubPix(img, corners, size, (1, 1), criteria)
#         # cv2.find4QuadCornerSubpix(img, corners, (w, h))
#         return corners

gray_img= cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
image = cv2.resize(gray_img, (image.shape[1]*2,image.shape[0]*2))
 
minThreshValue = 120
_, binary = cv2.threshold(gray_img, minThreshValue, 255, cv2.THRESH_BINARY)

params = cv2.SimpleBlobDetector_Params()
# params.filterByArea = True
# params.minArea = 1
# params.maxArea = 5000
# params.filterByCircularity = True
# params.minCircularity = 0.8

blobDetector = cv2.SimpleBlobDetector_create(params)

# found, centers = cv2.findCirclesGrid(binary, size, flags=cv2.CALIB_CB_SYMMETRIC_GRID, blobDetector=blobDetector)


# print(centers)
# cv2.drawChessboardCorners(image, size, centers, found is not None)

# # Detect blobs
# keypoints = blobDetector.detect(image)
# # print(keypoints)

# # Draw detected blobs as red circles.
# image_with_keypoints = cv2.drawKeypoints(image, keypoints, np.array([]), (0, 255, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

# Detect circles
circles = cv2.HoughCircles(image, cv2.HOUGH_GRADIENT, dp=1, minDist=20,
                           param1=50, param2=30, minRadius=3, maxRadius=10)

# Draw circles
if circles is not None:
    circles = np.uint16(np.around(circles))
    for x, y, r in circles[0, :]:
        cv2.circle(image, (x, y), r, (0, 255, 0), 2)

cv2.imwrite('points.jpg', image)