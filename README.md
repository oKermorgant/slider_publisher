# slider_publisher

This packages proposes a slider-based publisher node similar to the joint_state_publisher, but that can publish any message or service.

Many examples are given, especially

*   `ros2 launch slider_publisher example.launch file:=BasicTypes.yaml` for the 4 basic types
*   `ros2 launch slider_publisher example.launch file:=Twist.yaml` for timestamped Twist (cmd_vel)
*   `ros2 launch slider_publisher example.launch file:=tf.yaml` for a TF message
*   `ros2 launch slider_publisher example.launch file:=VelPose.yaml` for 2 topics (Twist + Pose)
*   `ros2 launch slider_publisher example.launch file:=MultiArray.yaml` for a topic with 4 floats
*   `ros2 launch slider_publisher add_two_ints.launch` for a service call (uses `demo_nodes_cpp server`)

Array-based messages are also possible (nested arrays are not).

The packages reduces to a single node that has to be called with an argument leading to a YAML file with the following structure (examples corresponding to the VelPose.yaml file):

    topic_to_be_published:  
        type: full message or service type    (geometry_msgs/Twist or geometry_msgs/msg/Twist)
        key_as_in_gui:                        (vx, can also be the message field)
            to: corresponding_message_field   (linear.x if not used as the key or not unique field)
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

## Input controls

Input will rely either on a checkbox (`Bool` type) or a text input.

- Numeric types: if min/max values are provided, a slider will also be available
- All types: if the default value is a list, a combobox will be used for the input, the first element of the list being the defalut value (see e.g. `tf.yaml` example)


## Parameters

- `config` (string): this parameter has the same effect as the raw file argument, that is the path to the YAML configuration file
- `rate` (double): publication rate (0-100 Hz)
