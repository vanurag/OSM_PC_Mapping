import cv2
import numpy as np
import itertools
import pprint

from matplotlib import pyplot as plt
#
# line segment intersection using vectors
# see Computer Graphics by F.S. Hill
#
def perp( a ) :
    b = np.empty_like(a)
    b[0] = -a[1]
    b[1] = a[0]
    return b

# line segment a given by endpoints a1, a2
# line segment b given by endpoints b1, b2
# return 
def seg_intersect(a1,a2, b1,b2) :
    da = a2-a1
    db = b2-b1
    dp = a1-b1
    dap = perp(da)
    denom = np.dot( dap, db)
    num = np.dot( dap, dp )
    return (num / denom.astype(float))*db + b1

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

cv2.imwrite('houghlines6.jpg',img)
