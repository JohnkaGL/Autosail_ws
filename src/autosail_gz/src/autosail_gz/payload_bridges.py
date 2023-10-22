from autosail_gz.bridge import Bridge, BridgeDirection

import sdformat13 as sdf


def gz_prefix(world_name, model_name, link_name, sensor_name):
    return f'/world/{world_name}/model/{model_name}/link/{link_name}/sensor/{sensor_name}'


def ros_prefix(sensor_name, sensor_type):
    return f'sensors/{sensor_type}/{sensor_name}'


def image(world_name, model_name, link_name, sensor_name):
    gz_sensor_prefix = gz_prefix(world_name, model_name, link_name, sensor_name)
    ros_sensor_prefix = ros_prefix(sensor_name, 'cameras')
    return Bridge(
        gz_topic=f'{gz_sensor_prefix}/image',
        ros_topic=f'{ros_sensor_prefix}/image_raw',
        gz_type='ignition.msgs.Image',
        ros_type='sensor_msgs/msg/Image',
        direction=BridgeDirection.GZ_TO_ROS)


def depth_image(world_name, model_name, link_name, sensor_name):
    gz_sensor_prefix = gz_prefix(world_name, model_name, link_name, sensor_name)
    ros_sensor_prefix = ros_prefix(sensor_name, 'cameras')
    return Bridge(
        gz_topic=f'{gz_sensor_prefix}/depth_image',
        ros_topic=f'{ros_sensor_prefix}/depth',
        gz_type='ignition.msgs.Image',
        ros_type='sensor_msgs/msg/Image',
        direction=BridgeDirection.GZ_TO_ROS)


def camera_info(world_name, model_name, link_name, sensor_name):
    gz_sensor_prefix = gz_prefix(world_name, model_name, link_name, sensor_name)
    ros_sensor_prefix = ros_prefix(sensor_name, 'cameras')
    return Bridge(
        gz_topic=f'{gz_sensor_prefix}/camera_info',
        ros_topic=f'{ros_sensor_prefix}/camera_info',
        gz_type='ignition.msgs.CameraInfo',
        ros_type='sensor_msgs/msg/CameraInfo',
        direction=BridgeDirection.GZ_TO_ROS)


def lidar_scan(world_name, model_name, link_name, sensor_name):
    gz_sensor_prefix = gz_prefix(world_name, model_name, link_name, sensor_name)
    ros_sensor_prefix = ros_prefix(sensor_name, 'lidars')
    return Bridge(
        gz_topic=f'{gz_sensor_prefix}/scan',
        ros_topic=f'{ros_sensor_prefix}/scan',
        gz_type='ignition.msgs.LaserScan',
        ros_type='sensor_msgs/msg/LaserScan',
        direction=BridgeDirection.GZ_TO_ROS)


def lidar_points(world_name, model_name, link_name, sensor_name):
    gz_sensor_prefix = gz_prefix(world_name, model_name, link_name, sensor_name)
    ros_sensor_prefix = ros_prefix(sensor_name, 'lidars')
    return Bridge(
        gz_topic=f'{gz_sensor_prefix}/scan/points',
        ros_topic=f'{ros_sensor_prefix}/points',
        gz_type='ignition.msgs.PointCloudPacked',
        ros_type='sensor_msgs/msg/PointCloud2',
        direction=BridgeDirection.GZ_TO_ROS)


def camera_points(world_name, model_name, link_name, sensor_name):
    gz_sensor_prefix = gz_prefix(world_name, model_name, link_name, sensor_name)
    ros_sensor_prefix = ros_prefix(sensor_name, 'cameras')
    return Bridge(
        gz_topic=f'{gz_sensor_prefix}/points',
        ros_topic=f'{ros_sensor_prefix}/points',
        gz_type='ignition.msgs.PointCloudPacked',
        ros_type='sensor_msgs/msg/PointCloud2',
        direction=BridgeDirection.GZ_TO_ROS)


def rfranger(world_name, model_name, link_name, sensor_name):
    gz_sensor_prefix = gz_prefix(world_name, model_name, link_name, sensor_name)
    ros_sensor_prefix = ros_prefix(sensor_name)
    return Bridge(
        gz_topic=f'{gz_sensor_prefix}/rfsensor',
        ros_topic=f'{ros_sensor_prefix}/rfsensor',
        gz_type='ignition.msgs.Param_V',
        ros_type='ros_gz_interfaces/msg/ParamVec',
        direction=BridgeDirection.GZ_TO_ROS)

def imu(world_name, model_name, link_name, sensor_name):
    gz_sensor_prefix = gz_prefix(world_name, model_name, link_name, sensor_name)
    ros_sensor_prefix = ros_prefix('', 'imu')
    return Bridge(
        gz_topic='/imu',
        ros_topic='imu/data',
        gz_type='ignition.msgs.IMU',
        ros_type='sensor_msgs/msg/Imu',
        direction=BridgeDirection.GZ_TO_ROS)

def navsat(world_name, model_name, link_name, sensor_name):
    return Bridge(
        gz_topic='/model/gps/fix',
        ros_topic='gps/fix',
        gz_type='ignition.msgs.NavSat',
        ros_type='sensor_msgs/msg/NavSatFix',
        direction=BridgeDirection.GZ_TO_ROS)


def contacts():
    return Bridge(
        gz_topic=f'/autosail/contacts',
        ros_topic=f'/autosail/contacts',
        gz_type='ignition.msgs.Contacts',
        ros_type='ros_gz_interfaces/msg/Contacts',
        direction=BridgeDirection.GZ_TO_ROS)


def joint_control(joint_name,topic_name,model_name):
    # ROS naming policy indicates that first character of a name must be an alpha
    # character. In the case below, the gz topic has the joint index 0 as the
    temp2=topic_name
    # print(temp)
    return Bridge(
        gz_topic=topic_name, #joint_name=${namespace}/${link_name}_joint
        ros_topic=temp2,
        gz_type='ignition.msgs.Double',
        ros_type='std_msgs/msg/Float64',
        direction=BridgeDirection.ROS_TO_GZ)
def joint_position(joint_name,topic_name,model_name):
    #joint_name=${namespace}/${link_name}_joint
    # temp=joint_name.replace(model_name,"")  #Extract model name from joint_name
    # temp1=temp.replace("//","/")            #Extract extra colons
    temp2=topic_name+'_joint'
    return Bridge(
        gz_topic=topic_name, 
        ros_topic=temp2, 
        gz_type='ignition.msgs.Model',
        ros_type='sensor_msgs/msg/JointState',
        direction=BridgeDirection.GZ_TO_ROS)
    
def payload_bridges(world_name, model_name, link_name, sensor_name, sensor_type):
    bridges = []
    if sensor_type == sdf.Sensortype.CAMERA:
        bridges = [
            image(world_name, model_name, link_name, sensor_name),
            camera_info(world_name, model_name, link_name, sensor_name)
        ]
    elif sensor_type == sdf.Sensortype.IMU:
        bridges = [
            imu(world_name, model_name, link_name, sensor_name)
        ]
    elif sensor_type == sdf.Sensortype.CONTACT:
        bridges = [
            contacts(),
        ]
    elif sensor_type == sdf.Sensortype.NAVSAT:
        bridges = [
            navsat(world_name, model_name, link_name, sensor_name),
        ]
    elif sensor_type == sdf.Sensortype.GPU_LIDAR:
        bridges = [
            lidar_scan(world_name, model_name, link_name, sensor_name),
            lidar_points(world_name, model_name, link_name, sensor_name)
        ]
    elif sensor_type == sdf.Sensortype.RGBD_CAMERA:
        bridges = [
            image(world_name, model_name, link_name, sensor_name),
            camera_info(world_name, model_name, link_name, sensor_name),
            depth_image(world_name, model_name, link_name, sensor_name),
            camera_points(world_name, model_name, link_name, sensor_name),
        ]
    elif 'servo_' in sensor_name:
        bridges= [
            joint_control(link_name,sensor_type,model_name)
        ]
    elif 'angle_' in sensor_name:
        bridges=[
            joint_position(link_name,sensor_type,model_name)
        ]
    # elif 'speed_' in sensor_name:
    #     bridges=[
    #         joint_position(link_name,model_name)
    #     ]

    return bridges
