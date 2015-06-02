import pcl
import numpy as np


import sys
sys.setrecursionlimit(10000)
import random

import array

import IPython

import blist

from intersec import seg_intersect

from matplotlib import pyplot as plt

p = pcl.PointCloud()
p.from_file(b"filtered_output.pcd")
parr = np.asarray(p)
# a = p.as_array()

kd = p.make_kdtree_flann()
# oc = pcl.OctreePointCloudSearch(0.5)
# oc.set_input_cloud(p)
# this should find all bounding boxes, hopefully
# indexes = array.array('i', range(0, p.size))
indexes = blist.sortedlist(range(0, p.size))
THRESHHOLD = 1 ** 2

# visited_idx = blist.sortedlist()

idx = random.choice(indexes)

# a, b = oc.radius_search([0, 0, 0], 5)
# print(a, b)

def get_corner(idx):
	nearest_idx, sqr_dist = kd.nearest_k_search_for_point(p, idx, 150)

	grid_p = pcl.PointCloud()
	temp_list = list()
	for i in nearest_idx:
		temp_list.append(p[i])
	grid_p.from_list(temp_list)

	seg = grid_p.make_segmenter()
	seg.set_model_type(pcl.SACMODEL_LINE)
	seg.set_method_type(pcl.SAC_RANSAC)
	seg.set_distance_threshold(0.5)

	indices, model = seg.segment()

	# print(model)

	if(grid_p.size - len(indices) < 5):
		return None
	g2 = pcl.PointCloud()
	g2.from_list([grid_p[idx] for idx in range(0, grid_p.size) if idx not in indices])

	if(g2.size):
		seg2 = g2.make_segmenter()
		seg2.set_model_type(pcl.SACMODEL_LINE)
		seg2.set_method_type(pcl.SAC_RANSAC)
		seg2.set_distance_threshold(0.5)

		indices2, model2 = seg2.segment()

		if not model2:
			return None
		
		angle = np.arccos(np.dot(np.array([model[3], model[4]]), np.array([model2[3], model2[4]])))

		if angle < 0.6 or angle > np.pi - 0.6:
			print("rejected", angle)
			return None

		intersection = seg_intersect(
			np.array([model[0], model[1]]),
			np.array([model[0] + model[3], model[1] + model[4]]),
			np.array([model2[0], model2[1]]),
			np.array([model2[0] + model2[3], model2[1] + model2[4]])
		)
		# print(intersection)

		# g_coords = np.asarray(grid_p)

		return intersection
		# plt.hold(True)
		# plt.scatter(g_coords[:, 0], g_coords[:, 1], color=np.random.rand(3,1))
		# plt.scatter(intersection[0], intersection[1], s=20)
		# plt.plot([model[0], model[0] + model[3] * 10], [model[1], model[1] + model[4] * 10] )
		# plt.plot([model2[0], model2[0] + model2[3] * 10], [model2[1], model2[1] + model2[4] * 10])
		# plt.show()
	else:
		return None


intersections = []
for i in range(0, 1000):
	idx = random.choice(indexes)
	a = get_corner(idx)

	if a != None:
		intersections.append(a)

g_coords = np.asarray(p)

plt.hold(True)
plt.scatter(g_coords[:, 0], g_coords[:, 1], color=(0.2, 0.2, 0.2))
ix = np.array(intersections)
plt.scatter(ix[:, 0], ix[:, 1], s=24, color=(1,0, 0))

plt.show()

iarr = np.array(intersections)

np.savetxt('corners_from_img.csv', iarr)
# while len(indexes):
# 	curr_idx = random.choice(indexes)
# 	# nearest_idx, sqr_dist = kd.nearest_k_search_for_point(p, curr_idx, 100)
# 	# group = []
# 	# print(nearest_idx, "\n\n")

# 	groups.append(find_group(curr_idx, blist.sortedset()))

# 	# for idx, val in enumerate(sqr_dist):
# 	# 	if val < THRESHHOLD:
# 	# 		group.append(nearest_idx[idx])

# 	# visited_idx.add(curr_idx)

# parr = np.asarray(p)

# plt.hold(True)
# for g in groups:
# 	if len(g) > 100:
# 		g_coords = np.array([p[idx] for idx in g])
# 		plt.scatter(g_coords[:, 0], g_coords[:, 1], color=np.random.rand(3,1))
# plt.show()

