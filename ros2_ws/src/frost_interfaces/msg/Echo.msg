# MAKE SURE TO MODIFY "Echo.srv" AS WELL
# time stamp
std_msgs/Header header
# measured distance (in mm)
uint32 distance
# confidence level (from 0 to 100)
uint8 conf_level
# profile data array 
uint8[] profile_data
