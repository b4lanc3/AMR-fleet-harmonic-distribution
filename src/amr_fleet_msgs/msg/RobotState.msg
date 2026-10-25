# Fleet-level view of one robot (published by the fleet manager).
string robot_id
string state                 # IDLE, TO_PICKUP, LOADING, TO_DROPOFF, UNLOADING, TO_CHARGER, CHARGING, WAIT_CHARGER, TO_PARK, PARKED, YIELDING, WAITING, OFFLINE
string state_detail          # human-readable explanation
int32 priority_rank          # 1 = highest priority in the fleet right now
float64 priority_score
int32 base_rank              # configured static rank (tie breaker)
geometry_msgs/Pose2D pose
float64 speed
float32 battery_percent
bool charging
string cargo                 # what the robot carries ("" = empty)

string task_id
string task_type
string task_description
string pickup
string dropoff
string current_goal          # node / station the robot is driving to now
float64 eta_goal_s           # ETA to current_goal
float64 eta_task_s           # ETA until the current task is completed
float64 distance_remaining_m

string[] route_node_ids      # remaining route nodes
geometry_msgs/Point[] route_points
int32 granted_count          # number of route_points already granted by traffic control
string route_summary         # e.g. "north+ -> x35- -> B1_IN"

string next_task_id
string next_task_description
float64 next_task_eta_start_s

string blocked_by            # robot this one is waiting for
string yielding_to           # robot this one gives way to
float64 waiting_time_s
uint32 tasks_completed
float64 odometer_m
