# OSM_PC_Mapping
Might map OSM outlines to pointclouds in the future

###Bundler Parser###
Compile: 
```bash
catkin build bundler_parser
```

Run:

Camera Query:

```bash
devel/lib/bundler_parser/parse_bundler --alsologtostderr --colorlogtostderr --file=[full_path]/file_name.out --query=camera --indices="1 3 12"
```
3D Point Query:

```bash
devel/lib/bundler_parser/parse_bundler --alsologtostderr --colorlogtostderr --file=[full_path]/file_name.out --query=3dpoint --indices="1 5 8"
```


###Point Cloud Segmentation###
Compile: 
```bash 
catkin build pc_segmentation
```

Run:

Segment Point Cloud:

```bash
devel/lib/pc_segmentation/segment_pc --alsologtostderr --colorlogtostderr --bundler_file=[full_path]/file_name.out --outline_file_path=[storage_path] --show_cameras=true --show_cloud=true --show_normals=false --segmentation_threshold=0.001 --search_radius=0.4
```

### Point Cloud matching

#### Obtaining Ground truth

Execute osm/visualization.py

#### Filtering

In folder ransac, `cmake .`, `make` and then executable filters will yield filtered output from `test_pcd.pcd` and save it as `filtered_output.pcd`.

Corners are found by using `corner_extract.py` on filtered output.

Then corners can be matched by using `match_corners.py` (note long runtime since dumb bruteforce method).

#### ICP

In libpointmatcher folder, executables are placed in libpointmatcher/examples.

Execute `./pmicp --config config.yaml --initTranslation 80,100 ../data/cl.csv ../data/cl2.csv` to generate a mapping based on a Point-to-Point ICP.
