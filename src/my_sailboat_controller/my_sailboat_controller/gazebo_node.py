#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from sensor_msgs.msg import MagneticField,FluidPressure,Imu,NavSatFix,JointState
'''This node is made with the purpuse of enable ros_gz_bridge topics to be read by HITL estrategies 
without changing Mälardalens University scheme for messages considered in autosail_message package.

It is supposed to work together with the control node
'''
from autosail_message.msg import RudderControlMessage
from autosail_message.msg import SailAngleMessage
from autosail_message.msg import IMUMessage
from autosail_message.msg import WindMessage
from autosail_message.msg import PositionMessage
import numpy as np

class gz_decode_message(Node):
    def __init__(self):
        super().__init__('gazebo_decoder_node')
        self.get_logger().info("Gazebo decoder is armed!!")
        # Publishers ROS-END
        self.GPS_position_publisher = self.create_publisher(PositionMessage,'Position',10)
        self.GPS_position_message=PositionMessage()

        self.windsensor_publisher= self.create_publisher(WindMessage,'Wind',10)
        self.windsensor_message=WindMessage()

        self.IMUpose_publisher= self.create_publisher(IMUMessage,'IMU',10)
        self.IMUpose_message=IMUMessage()

        # Publisher ROS_GZ_BRIDGE        
        self.rudder_publisher = self.create_publisher(Float64,'rudder_joint',10) 
        self.rudder_gz_msg = Float64()

        self.sail_publisher = self.create_publisher(Float64,'main_sail_joint',10)
        self.sail_gz_msg = Float64()

        # Subscribers ROS-END
        self.rudder_message=RudderControlMessage()
        self.rudder_subscriber = self.create_subscription(RudderControlMessage,'rudder_ctrl',self.rudder_command,10)
        self.rudder_subscriber

        self.sail_message=SailAngleMessage()
        self.sail_subscriber = self.create_subscription(SailAngleMessage,'sail_ctrl',self.sail_command,10)
        self.sail_subscriber

        # Subcribers ROS_GZ_BRIDGE
        self.imu_message=Imu()
        self.imu_subscriber = self.create_subscription(Imu,'imu/data',self.gz_imu,10)
        self.imu_subscriber

        self.gps_message=NavSatFix()
        self.gps_subscriber = self.create_subscription(NavSatFix,'gps/fix',self.gz_gps,10)
        self.gps_subscriber

        # self.air_message=FluidPressure()
        # self.air_subscriber = self.create_subscription(FluidPressure,'air_presure',self.gz_air,10)
        # self.air_subscriber

        self.airdir_message=JointState()
        self.airdir_subscriber = self.create_subscription(JointState,'wind_angle_joint',self.gz_windpos,10)
        self.airdir_subscriber

        self.airdir_message=JointState()
        self.airdir_subscriber = self.create_subscription(JointState,'wind_speed_joint',self.gz_windspd,10)
        self.airdir_subscriber

        self.magnetometer_message=MagneticField()
        self.magnetometer_subscriber = self.create_subscription(MagneticField,'magnetic_field',self.gz_magnet,10)
        self.magnetometer_subscriber
        self.wind_spd=0

    def rudder_command(self,msg):
        self.rudder_gz_msg.data=float(msg.rudder_angle)
        # self.get_logger().info('Rudder direction: %f'%msg.rudder_angle)
        self.rudder_publisher.publish(self.rudder_gz_msg)

        
    def sail_command(self,msg):
        self.sail_gz_msg.data=float(msg.sail_angle)
        # self.get_logger().info('Sail direction: %f'%msg.sail_angle)
        self.sail_publisher.publish(self.sail_gz_msg)

    def gz_imu(self,msg):
        self.IMUpose_message.linear_acceleration_x=msg.linear_acceleration.x
        self.IMUpose_message.linear_acceleration_y=msg.linear_acceleration.y
        self.IMUpose_message.linear_acceleration_z=msg.linear_acceleration.z
        self.IMUpose_message.roll=msg.angular_velocity.x
        self.IMUpose_message.pitch=msg.angular_velocity.y
        self.IMUpose_message.yaw=msg.angular_velocity.z
        self.IMUpose_publisher.publish(self.IMUpose_message)

    def gz_gps(self,msg):
        self.GPS_position_message.latitude=msg.latitude
        self.GPS_position_message.longitude=msg.longitude
        self.GPS_position_publisher.publish(self.GPS_position_message)
        # self.get_logger().info('position:'+self.GPS_position_message)
    def gz_air(self,msg):
        pass
    def gz_windpos(self,msg):
        self.windsensor_message.wind_speed=float(self.wind_spd)
        self.airdir=float(msg.position[0])*180.0/np.pi
        self.windsensor_message.wind_angle=int(self.airdir)
        self.windsensor_publisher.publish(self.windsensor_message)
        pass
    def gz_windspd(self,msg):
        self.wind_spd=msg.position[0]
    def gz_magnet(self,msg):
        self.magnetometer_message=msg
        pass
def main(args=None):
    rclpy.init(args=args)
    # Node
    node = gz_decode_message()
    rclpy.spin(node)#keep the node alive
    node.destroy_node()
    rclpy.shutdown()

# If you want it to run from the terminal
if __name__ == '__main__':
    main()