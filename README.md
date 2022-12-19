# slider_publisher

This packages proposes a slider-based publisher node similar to the joint_state_publisher, but that can publish any message or service.

**Known bug** version 2.2.0 introduced a `rate` parameter to choose at which rate messages are published. The default value for the rate was erroneously set to `0.1` while it should have been `10`. This version was released in the `Foxy` September sync. A way to avoid the issue is to set the `rate` parameter to `10`, or use the source version until the bug fix makes it to the next release.

Several examples are given:
*   `ros2 launch example.launch` for timestamped Twist (cmd_vel)
*   `ros2 launch example.launch file:=Twist.yaml` for non-timestamped Twist
*   `ros2 launch example.launch file:=VelPose.yaml` for 2 topics (Twist + Pose)
*   `ros2 launch example.launch file:=MultiArray.yaml` for a topic with 4 floats
*   `ros2 launch gazebo_service.launch` for a service call (uses `gazebo_msgs`)

Only numerical types or their derivatives can be publisher (ie there is no text input to change such value on-the-fly).
Array-based messages are also possible for numerical values.

The packages reduces to a single node that has to be called with an argument leading to a YAML file with the following structure (examples corresponding to the VelPose.yaml file):

    topic_to_be_published:  
        type: full message or service type               (geometry_msgs/Twist or geometry_msgs/msg/Twist to specify it is a message)  
        key_as_in_gui:                        (vx)  
            to: corresponding_message_field   (linear.x)  
            min: slider minimum value         (-1)  
            max: slider maximum value         (+1)  
            default: slider default value     (if not: (min+max)/2)
        other_key:  
            ...
    other_topic_to_be_published:  
        type...

Hard-coded numeric values (bounds / constant values in messages) can be defined using fractions of pi.

3D rotations (ie quaternions) can be parameterized as roll / pitch / yaw (see `RPY.yaml`). While these fields are of course not part of a `Quaternion` message, the corresponding 3D rotation will be built and published.

If the type is a service interface then the corresponding slider will call the service, after waiting for the server.

## Parameters

- `config` (string): this parameter has the same effect as the raw file argument, that is the path to the YAML configuration file
- `rate` (double): publication rate (0-100 Hz)
