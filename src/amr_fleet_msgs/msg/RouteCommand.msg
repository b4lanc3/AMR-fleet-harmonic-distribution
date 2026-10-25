# Fleet manager -> robot agent. Sent whenever the route or the granted
# segment changes, and re-sent periodically as a heartbeat.
std_msgs/Header header

uint8 MODE_IDLE=0      # no route, hold position
uint8 MODE_FOLLOW=1    # follow waypoints up to granted_index
uint8 MODE_HOLD=2      # route kept but robot must stop (fleet pause / e-stop)

uint32 command_id       # increments every time a new route is issued
uint8 mode
string[] node_ids       # traffic-graph node id of every waypoint
geometry_msgs/Point[] waypoints   # map frame
float32[] speed_limits  # max speed [m/s] for the segment that ends at waypoint i
int32 granted_index     # robot may drive up to (and stop at) waypoints[granted_index]
float64 final_yaw       # yaw to align to at the last waypoint (NaN = don't care)
bool charge             # robot is docked on a charger -> enable charging
bool carrying           # goods on board -> raise the cargo (visual load in Gazebo)
