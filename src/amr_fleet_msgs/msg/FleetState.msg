std_msgs/Header header
bool paused
float64 sim_time_s
RobotState[] robots
TaskInfo[] tasks             # queued + active tasks
string[] charger_ids
string[] charger_occupants   # robot id or "" per charger
string[] charger_queue       # robots waiting for a free charger (in order)
uint32 tasks_completed
uint32 yield_maneuvers
uint32 deadlocks_resolved
