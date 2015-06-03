import cv2
import numpy as np
import itertools
import pprint

from matplotlib import pyplot as plt
from intersec import seg_intersect


# calculate hough transform and line intersections
# of found lines to obtain corners 


img = cv2.imread('Selection_032.png')
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

# edges = cv2.Canny(gray,50,150,apertureSize = 3)


minLineLength = 100
maxLineGap = 1
lines = cv2.HoughLinesP(gray,1,np.pi/180,130,minLineLength,maxLineGap)

for x1,y1,x2,y2 in lines[0]:
    cv2.line(img,(x1,y1),(x2,y2),(0,255,0),2)

intersections = []

print(lines[0])

for combo in itertools.combinations(lines[0], 2):
	line = combo[0]
	line2 = combo[1]
	isect = seg_intersect(np.array([line[0], line[1]]), np.array([line[2], line[3]]), 
						  np.array([line2[0], line2[1]]), np.array([line2[2], line2[3]]))
	print(isect, "\n")
	if not np.isnan(isect).all():
		intersections.append(isect.tolist())
	else:
		print("NOT A NUMBER")


pprint.pprint(intersections)


ar = np.array(intersections)
plt.hold(True)
plt.scatter(ar[:, 0], ar[:, 1])
for x1,y1,x2,y2 in lines[0]:
	plt.plot([x1, x2], [y1, y2], 'r')
plt.show()

cv2.imwrite('houghlines6.jpg', img)
