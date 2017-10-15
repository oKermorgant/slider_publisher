# slider_publisher

This packages proposes a slider-based publisher node similar to the joint_state_publisher, but that can publish any type of message.

Two examples are given, for 1 (velocity.launch) and 2 topics (velpose.launch) published from the same GUI.

Array-based messages are not implemented yet.

The packages reduces to a single node that has to be called either with a ~file param or an argument leading to a YAML file with the following structure:

