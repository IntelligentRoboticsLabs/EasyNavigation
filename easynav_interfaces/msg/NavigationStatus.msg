# A message to represent the status of the navigation system

# Message header
std_msgs/Header header

# The current robot pose
geometry_msgs/PoseStamped current_pose

# The current goal in the system
geometry_msgs/PoseStamped goal
# A unique identifier for the goal
string goal_id

# Total time navigating towards this goal
builtin_interfaces/Duration navigation_time

# Estimated remaining time to reach the goal
builtin_interfaces/Duration estimated_time_remaining

# Amount of distance that the robot has covered since it received this goal
float32 distance_covered
# Estimated remaining distance to the goal
float32 distance_to_goal

# The current status of the navigation system
uint8 STARTING=0    # No goal received yet
uint8 NAVIGATING=1  # Navigating towards the goal
uint8 PAUSED=2      # Navigation paused
uint8 CANCELED=3    # Navigation canceled from outside
uint8 ABORTED=4     # Navigation aborted due to an error
uint8 PREEMPTED=5   # Navigation aborted by a new (higher priority) goal
uint8 FINISHED=6    # Navigation finished successfully

uint8 status

# Message to provide additional information about the status
string status_message
