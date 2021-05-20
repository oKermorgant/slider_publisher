# slider_publisher

This packages proposes a slider-based publisher node similar to the joint_state_publisher, but that can publish any type of message.

Several examples are given:
*   `ros2 launch example.launch` for timestamped Twist (cmd_vel)
*   `ros2 launch example.launch file:=Twist.yaml` for non-timestamped Twist
*   `ros2 launch example.launch file:=VelPose.yaml` for 2 topics (Twist + Pose)
*   `ros2 launch example.launch file:=MultiArray.yaml` for a topic with 4 floats

Array-based messages are possible only for floating point arrays. 

The packages reduces to a single node that has to be called with an argument leading to a YAML file with the following structure (examples corresponding to the VelPose.yaml file):

    topic_to_be_published:  
        type: full message type               (geometry_msgs/Twist)  
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
