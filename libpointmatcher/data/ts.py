import pcl 
import numpy as np

p = pcl.PointCloud()
p.from_file(b'filtered_output.pcd')

arr = np.asarray(p)

np.savetxt('filtered.csv', arr)