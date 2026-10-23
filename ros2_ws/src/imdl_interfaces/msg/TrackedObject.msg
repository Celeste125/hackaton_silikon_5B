uint8 STATUS_TENTATIVE=0
uint8 STATUS_CONFIRMED=1
uint8 STATUS_STALE=2
uint8 STATUS_CARRIED=3

std_msgs/Header header
string object_id
string color
geometry_msgs/PoseWithCovariance pose
float32 confidence
uint32 confirmations
float32 age_sec
uint8 status
