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
