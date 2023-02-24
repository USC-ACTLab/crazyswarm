std_msgs/Header header  # Header including the ROS 2 timestamp when the log data was received
uint32 timestamp        # on-board timestamp from the STM32 (in ms)
float32[] values        # converted values, in the order as specified for the log block
