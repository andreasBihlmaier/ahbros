geometry_msgs/Point[] points # in camera optical frame [m]
float64[] sizes # sizes[i]: size in camera optical frame at points[i] position [m], if sizes.length != points.length size[0] is used for all
std_msgs/ColorRGBA[] colors # if colors.length != points.length colors[0] is used for all
