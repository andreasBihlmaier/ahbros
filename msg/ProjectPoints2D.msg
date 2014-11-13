Point2D[] points # OpenCV image coordinates [px]
float64[] sizes # [px], if sizes.length != points.length sizes[0] is used for all
std_msgs/ColorRGBA[] colors # if colors.length != points.length colors[0] is used for all
bool numbers # Draw points index into image
