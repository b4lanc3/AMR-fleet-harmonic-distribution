string line_id
string line_type             # production | packaging | storage | supply
string name
string status                # RUNNING, STARVED, BLOCKED, IDLE
int32 input_buffer
int32 input_capacity
int32 inbound                # transports on the way to the input
int32 work_in_progress       # products in the current box / pallet
int32 output_buffer
int32 output_capacity
int32 produced
float64 utilization          # 0..1
