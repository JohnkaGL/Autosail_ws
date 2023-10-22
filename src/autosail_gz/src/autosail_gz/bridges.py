from autosail_gz.bridge import Bridge, BridgeDirection

def prefix(world_name, model_name, link_name):
    return f'/world/{world_name}/model/{model_name}/link/{link_name}/sensor'

def magnetometer(world_name, model_name, link_name='base_link'):
    sensor_prefix = prefix(world_name, model_name, link_name)
    return Bridge(
        gz_topic='magnetometer',
        ros_topic='magnetic_field',
        gz_type='gz.msgs.Magnetometer',
        ros_type='sensor_msgs/msg/MagneticField',
        direction=BridgeDirection.GZ_TO_ROS)

def air_pressure(world_name, model_name, link_name='base_link'):
    sensor_prefix = prefix(world_name, model_name, link_name)
    return Bridge(
        gz_topic=f'{sensor_prefix}/air_pressure/air_pressure',
        ros_topic='air_pressure',
        gz_type='gz.msgs.FluidPressure',
        ros_type='sensor_msgs/msg/FluidPressure',
        direction=BridgeDirection.GZ_TO_ROS)

def pose(model_name):
    return Bridge(
        gz_topic=f'/model/{model_name}/pose',
        ros_topic='pose',
        gz_type='gz.msgs.Pose_V',
        ros_type='tf2_msgs/msg/TFMessage',
        direction=BridgeDirection.GZ_TO_ROS)

def pose_static(model_name):
    return Bridge(
        gz_topic=f'/model/{model_name}/pose_static',
        ros_topic='pose_static',
        gz_type='gz.msgs.Pose_V',
        ros_type='tf2_msgs/msg/TFMessage',
        direction=BridgeDirection.GZ_TO_ROS)

def cmd_vel(model_name):
    return Bridge(
        gz_topic=f'/model/{model_name}/cmd_vel',
        ros_topic='cmd_vel',
        gz_type='gz.msgs.Twist',
        ros_type='geometry_msgs/msg/Twist',
        direction=BridgeDirection.ROS_TO_GZ)

def clock():
    return Bridge(
        gz_topic='/clock',
        ros_topic='/clock',
        gz_type='gz.msgs.Clock',
        ros_type='rosgraph_msgs/msg/Clock',
        direction=BridgeDirection.GZ_TO_ROS)

def task_info():
    return Bridge(
        gz_topic=f'/autosail/task/info',
        ros_topic=f'/autosail/task/info',
        gz_type='gz.msgs.Param',
        ros_type='ros_gz_interfaces/msg/ParamVec',
        direction=BridgeDirection.GZ_TO_ROS)

    