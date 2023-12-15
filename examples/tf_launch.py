# This launch file requires simple_launch
# It allows building a slider_publisher to publish an arbitrary TF

from simple_launch import SimpleLauncher

sl = SimpleLauncher()
sl.declare_arg('frame_id', 'world')
sl.declare_arg('child_frame_id', 'base_link')


def launch_setup():

    config = '''/tf:
    transforms[0].child_frame_id: __child_frame_id
    transforms[0].header.frame_id: __frame_id
    type: tf2_msgs/msg/TFMessage
    x:
        max: 5.0
        min: -5.0
        to: transforms[0].transform.translation.x
    y:
        max: 5.0
        min: -5.0
        to: transforms[0].transform.translation.y
    z:
        max: 5.0
        min: -5.0
        to: transforms[0].transform.translation.z
    pitch:
        max: pi
        min: -pi
        to: transforms[0].transform.rotation.pitch
    roll:
        max: pi
        min: -pi
        to: transforms[0].transform.rotation.roll
    yaw:
        max: pi
        min: -pi
        to: transforms[0].transform.rotation.yaw'''

    for key in ('frame_id', 'child_frame_id'):
        config = config.replace('__'+key, sl.arg(key))

    tf_yaml = '/tmp/tf_publisher.yaml'

    name = f'tf_{sl.arg("frame_id")}_{sl.arg("child_frame_id")}'.replace('/','')

    with open(tf_yaml,'w') as f:
        f.write(config)

    sl.node('slider_publisher', name= name, arguments = [tf_yaml])

    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)
