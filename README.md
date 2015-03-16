# OSM_PC_Mapping
Might map OSM outlines to pointclouds in the future

###Bundler Parser###
Compile: ```bash catkin build bundler_parser```
Run:

Camera Query:

```bash
devel/bundler_parser/lib/bundler_parser/parse_bundler --alsologtostderr --colorlogtostderr --file=[full_path]/aachen.out --query=camera --index=0
```
3D Point Query:

```bash
devel/bundler_parser/lib/bundler_parser/parse_bundler --alsologtostderr --colorlogtostderr --file=[full_path]/aachen.out --query=3dpoint --index=0
```
