import numpy as np
from matplotlib import pyplot as plt

from scipy.spatial.distance import cdist, sqeuclidean
from scipy.spatial import cKDTree

osm_corners = np.loadtxt(open("OSM_corners.csv","rb"))
print("Before unique", osm_corners.shape)
nada, uqi = np.unique(osm_corners[:, 0], return_index=True)
osm_corners = osm_corners[uqi, :]
print("After unique", osm_corners.shape)

img_corners = np.loadtxt(open("corners_from_img__1.csv", "rb"))

dists = cdist(img_corners, img_corners, 'sqeuclidean')

# plt.hold(True)
# plt.scatter(img_corners[:, 0], img_corners[:, 1])

chose_from = np.argwhere(dists < 2)
print(chose_from)
print(chose_from.shape)
delete_idxs = []
for el in chose_from:
	# print(dists[el], el)
	if el[0] != el[1] and el[0] > el[1] and el[1] not in delete_idxs:
		delete_idxs.append(el[1]) 

print(delete_idxs)

img_corners = np.delete(img_corners, np.array(delete_idxs), 0)
# plt.scatter(img_corners[:, 0], img_corners[:, 1], color=(0,1,0))
# plt.show()
# print(osm_corners[:, 0:2])

img_corners = np.hstack( ( img_corners, np.ones( (img_corners.shape[0], 1) ) ) )
osm_corners = np.hstack( ( osm_corners, np.ones( (osm_corners.shape[0], 1) ) ) )

kd_osm = cKDTree(osm_corners[:, 0:2])

prev_distance = 100000000000000000000000000
best_trafo = None
loop_i = 0
while prev_distance > 5000 and loop_i < 50000:
	loop_i += 1
	choice_a = np.random.choice(img_corners.shape[0], 3)

	choice_b = np.random.choice(osm_corners.shape[0], 3)

	# translation = osm_corners[choice_b[0], :] - img_corners[choice_a[0], :]
	src = np.transpose( img_corners[choice_a, :] )
	ref = np.transpose( osm_corners[choice_b, :] )

	# print("SRC: \n", src )
	# print("REF: \n", ref)
	try:
		trafo = np.dot(ref, np.linalg.inv(src))
	except:
		continue

	# print("TransformMatrix \n", trafo)
	# print("Result: \n", np.dot(trafo, src))

	transformed_img = np.dot(trafo, np.transpose(img_corners))
	transformed_img = np.transpose(transformed_img)
	# print(transformed_img)

	dsts, nearest_idxs = kd_osm.query(transformed_img[:, 0:2], distance_upper_bound=200)
	u, unique_idx = np.unique(nearest_idxs, return_index=True)

	# print(nearest_idxs)
	# print("N:  ", kd_osm.n)

	sum_dist = 0
	for i in range(0, len(nearest_idxs)):
		if nearest_idxs[i] == kd_osm.n:
			sum_dist = prev_distance + 1
			break
		if i not in unique_idx:
			sum_dist += 400
		else:
			sum_dist += sqeuclidean(osm_corners[nearest_idxs[i], 0:2], transformed_img[i, 0:2])

	if abs(sum_dist) < prev_distance:
		print(sum_dist)
		best_trafo = trafo
		prev_distance = abs(sum_dist)
	# plt.hold(False)
	# plt.scatter(osm_corners[:, 0], osm_corners[:, 1], color=(1,0,0))
	# plt.hold(True)
	# plt.scatter(transformed_img[:, 0], transformed_img[:, 1], color=(0,1,0))
	# plt.scatter(src[:, 0], src[:, 1], color=(0,0,1), s=30)
	# plt.show()

print("\n\n\nBest Result: ")
print(best_trafo)
print("DISTANCE: ", prev_distance)

transformed_img = np.dot(best_trafo, np.transpose(img_corners))
transformed_img = np.transpose(transformed_img)
plt.hold(False)
plt.scatter(osm_corners[:, 0], osm_corners[:, 1], color=(1,0,0))
plt.hold(True)
plt.scatter(transformed_img[:, 0], transformed_img[:, 1], color=(0,1,0))
plt.scatter(src[:, 0], src[:, 1], color=(0,0,1), s=30)
plt.show()
