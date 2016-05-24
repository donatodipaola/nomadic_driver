# nomadic_driver
ROS driver for Nomadic robots

## Subscribed Topics

#### `cmd_vel` *geometry_msgs::Twist*
The velocity command that the robot should execute.

## Published Topics

#### `odom` *nav_msgs::Odometry*
The robot odometry.

## Params

#### `port` *string*
Default: `/dev/ttyUSB0`.

#### `model` *string*
The robot model. Can be "N200", "N150", "Scout", "Scout2".
Default: `Scout2`.

#### `retry_interval` *int*
If the driver can't connect to the robot, retry after this time interval (in
milliseconds).
If this param is set to `0` the driver quits when the first conection fails.
Default: `2500`.

#### `odom_frame_id` *string*
The tf frame attached to the origin of the reference system.
Default: `/odom`.

#### `frame_id` *string*
The tf frame attached to the robot.
Default: `/base_link`.

#### `publish_tf` *bool*
Whether the node should brodcast a tf transform from `odom_frame_id` to
`frame_id`.
Default: `true`.
