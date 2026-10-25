# Robot agent -> fleet manager (10 Hz).
std_msgs/Header header
string robot_id
geometry_msgs/Pose2D pose          # map frame
float64 linear_velocity
float64 angular_velocity
uint32 command_id                  # last RouteCommand.command_id received
int32 reached_index                # last waypoint index reached on that route (-1 = none)
bool route_complete                # last waypoint reached and final yaw aligned
float32 battery_percent
bool charging
bool obstacle_stop                 # lidar / camera safety field triggered
float32 min_obstacle_distance      # closest obstacle in the driving sector [m]
