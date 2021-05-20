# slider_publisher

This package proposes a slider-based publisher node similar to the joint_state_publisher, but that can publish any type of message.

Several examples are given:
*   roslaunch example.launch for timestamped Twist (cmd_vel)
*   roslaunch example.launch file:=Twist.yaml for non-timestamped Twist
*   roslaunch example.launch file:=VelPose.yaml for 2 topics (Twist + Pose)
*   roslaunch example.launch file:=MultiArray.yaml for a topic with 4 floats

Array-based messages are possible only for floating point arrays. 

The packages reduces to a single node that has to be called either with a ~file param or an argument leading to a YAML file with the following structure (examples corresponding to the VelPose.yaml file):

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
All sliders may also have a default value defined with the `default` keyword.
