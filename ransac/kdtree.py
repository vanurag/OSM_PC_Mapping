import pcl
import numpy as np


import sys
sys.setrecursionlimit(10000)
import random

import array

import IPython

import blist

from matplotlib import pyplot as plt

p = pcl.PointCloud()
p.from_file(b"filtered_output.pcd")

# a = p.as_array()

kd = p.make_kdtree_flann()

# this should find all bounding boxes, hopefully
# indexes = array.array('i', range(0, p.size))
indexes = blist.sortedlist(range(0, p.size))
print(indexes)
THRESHHOLD = 1 ** 2

# visited_idx = blist.sortedlist()

groups = []
visited_idx = blist.sortedlist()
def find_group(idx, res):
	if idx in visited_idx:
		return res
	visited_idx.add(idx)
	print("Checking", idx)
	nearest_idx, sqr_dist = kd.nearest_k_search_for_point(p, idx, 20)
	next_idxs = []

	for cidx, val in enumerate(sqr_dist):
		if val < THRESHHOLD and val > 0:
			res.add(nearest_idx[cidx])
			next_idxs.append(nearest_idx[cidx])

	for n in next_idxs:
		res = find_group(n, res)
	
	for i in nearest_idx:
		try:
			indexes.remove(i)
		except ValueError:
			pass

	return res 

while len(indexes):
	curr_idx = random.choice(indexes)
	# nearest_idx, sqr_dist = kd.nearest_k_search_for_point(p, curr_idx, 100)
	# group = []
	# print(nearest_idx, "\n\n")

	groups.append(find_group(curr_idx, blist.sortedset()))

	# for idx, val in enumerate(sqr_dist):
	# 	if val < THRESHHOLD:
	# 		group.append(nearest_idx[idx])

	# visited_idx.add(curr_idx)

parr = np.asarray(p)

plt.hold(True)
for g in groups:
	if len(g) > 100:
		g_coords = np.array([p[idx] for idx in g])
		plt.scatter(g_coords[:, 0], g_coords[:, 1], color=np.random.rand(3,1))
plt.show()

IPython.embed()
