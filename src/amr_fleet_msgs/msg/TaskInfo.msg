string task_id
string task_type             # KIT_DELIVERY, BOX_TRANSFER, FG_DELIVERY, MANUAL
string description
int32 priority               # 0 low, 1 normal, 2 high, 3 urgent
string status                # QUEUED, ASSIGNED, TO_PICKUP, LOADING, IN_TRANSIT, UNLOADING, DONE, CANCELLED
string pickup
string dropoff
string robot_id
string next_for_robot        # task is queued as "next task" of this robot
float64 age_s
float64 eta_s
