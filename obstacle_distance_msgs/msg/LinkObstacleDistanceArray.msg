# Array of distance measurements across all close robot link and obstacle pairs
std_msgs/Header header
obstacle_distance_msgs/LinkObstacleDistance[] distances

float64 overall_min_distance
string overall_closest_link
string overall_closest_obstacle
