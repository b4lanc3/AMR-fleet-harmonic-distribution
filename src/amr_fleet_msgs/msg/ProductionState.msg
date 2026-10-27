std_msgs/Header header
float64 elapsed_s
LineState[] lines             # production / packaging lines, warehouse "D", gates, supply
int32 kits_delivered
int32 products_made
int32 cartons_stored          # cartons put away in the finished goods warehouse
int32 cartons_picked          # cartons taken out of it to the packaging lines
int32 warehouse_stock         # cartons in the warehouse
int32 products_packaged
int32 cartons_shipped         # packed cartons loaded onto the trucks at the gates
int32 products_shipped
int32 trucks_departed
float64 throughput_per_hour   # products shipped per hour
