# OSM_PC_Mapping
Might map OSM outlines to pointclouds in the future

###Bundler Parser###
Compile: ```bash catkin build bundler_parser```
Run:

Camera Query:

```bash
devel/bundler_parser/lib/bundler_parser/parse_bundler --alsologtostderr --colorlogtostderr --file=[full_path]/aachen.out --query=camera --indices="1 3 12"
```
3D Point Query:

```bash
devel/bundler_parser/lib/bundler_parser/parse_bundler --alsologtostderr --colorlogtostderr --file=[full_path]/aachen.out --query=3dpoint --indices="1 5 8"
```
