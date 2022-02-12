# Map creation

STEPS:
- feed correct topics
- deleete intensity field from .pcd
- convert to binary_compressed

TODO - for realese:
- pc_filter _transform params not loading correctly in ndt mapping launch
- script to automatically remove intensity and transform to binary_compressed

# Lanelet map

SRTEPS:
- Download original autoware map
- make your own using https://tools.tier4.jp/vector_map_builder_ll2/
- load .pcd to builder and 
- based on autoware map, path point need to be on 90000, 30000 cordinates for some reason
- lanelet map only works when building on scraps from avp demo map

TODO - for realese:
- Building map on pcd in tier4 tools without nedd to move it
- Building map from the beginning not from avp demo map
- Visualization in rviz without moving lanelet
