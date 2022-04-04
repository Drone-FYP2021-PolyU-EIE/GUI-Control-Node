# Set control node state
#
#

std_msgs/Header header
<<<<<<< HEAD
bool useCurrentPx4Mode
string setPx4mode
bool useCurrentMode
string controlMode
bool useLocalPos
geometry_msgs/Pose setTargetPos
=======
string setPx4mode
bool useLocalPos
geometry_msgs/PoseStamped setPx4LocalPOS4
bool allowRemote
string controlMode
>>>>>>> b73a5795e31cb98180bb93433c1473466d8f25a9

# for px4mode  refer to system_status values
# see https://mavlink.io/en/messages/common.html#MAV_STATE
string MODE_PX4_MANUAL = MANUAL
string MODE_PX4_ACRO = ACRO
string MODE_PX4_ALTITUDE = ALTCTL
string MODE_PX4_POSITION = POSCTL
string MODE_PX4_OFFBOARD = OFFBOARD
string MODE_PX4_STABILIZED = STABILIZED
string MODE_PX4_RATTITUDE = RATTITUDE
string MODE_PX4_MISSION = AUTO.MISSION
string MODE_PX4_LOITER = AUTO.LOITER
string MODE_PX4_RTL = AUTO.RTL
string MODE_PX4_LAND = AUTO.LAND
string MODE_PX4_RTGS = AUTO.RTGS
string MODE_PX4_READY = AUTO.READY
string MODE_PX4_TAKEOFF = AUTO.TAKEOFF

# controlMode
string MODE_CONTROL_MODE_MANUAL = manual
string MODE_CONTROL_MODE_AUTO = auto