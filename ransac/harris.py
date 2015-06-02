import cv2
import numpy as np
import itertools
import pprint

from matplotlib import pyplot as plt
#
# line segment intersection using vectors
# see Computer Graphics by F.S. Hill
#
img = cv2.imread('Selection_032.png')
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
# edges = cv2.Canny(gray,50,150,apertureSize = 3)

gray = np.float32(gray)
dst = cv2.cornerHarris(gray, 2, 3, 0.24)

dst = cv2.dilate(dst, None)

img[dst > 0.01 * dst.max()] = [0, 0, 255]

cv2.imshow('dst', img)

if cv2.waitKey(0) & 0xff == 27:
    cv2.destroyAllWindows()