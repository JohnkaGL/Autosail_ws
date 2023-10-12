#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64


from autosail_message.msg import IMUMessage
from autosail_message.msg import WindMessage
from autosail_message.msg import PositionMessage
from autosail_message.msg import RudderControlMessage
from autosail_message.msg import SailAngleMessage
from .Simulator.Autosail_Simulator import App 

# Node description and architecture 
class basic_GUI(Node):
    def __init__(self):
        super().__init__('GUI_node')
        self.get_logger().info("GUI activated!!")
        
        # Publishers
        # General form
        # self.[name]_publisher = self.create_publiser([Type_of_message_imported], 'Topic', [message_stack_size])
        # self.[name]_msg= [Type_of_message_imported]()
        # self.create_timer([rate_in_seconds],[callback_function_name])
        self.create_timer(1,self.Update_GUI)

        # Subscribers
        self.rudder_subscriber = self.create_subscription(RudderControlMessage,'/autosail/rudder_ctrl',self.receive_rudder_position,10)
        self.rudder_subscriber
        
        self.sail_subscriber = self.create_subscription(RudderControlMessage,'/autosail/sail_ctrl',self.receive_sail_position,10)
        self.sail_subscriber

        self.GPS_position_subscriber = self.create_subscription(PositionMessage,'/autosail/Position',self.receive_position,10)
        self.GPS_position_subscriber

        self.windsensor_subscriber = self.create_subscription(WindMessage,'/autosail/Wind',self.receive_WD,10)
        self.windsensor_subscriber

        self.IMUpose_subscriber = self.create_subscription(IMUMessage,'/autosail/IMU',self.receive_IMU,10)
        self.IMUpose_subscriber

        # Recieved variables
        self.IMU_message = IMUMessage()
        self.Wind_message = WindMessage()
        self.Position_message = PositionMessage()
        self.sail_msg = SailAngleMessage()
        self.rudder_msg = RudderControlMessage()
        
        # GUI
        self.app = App()
        self.app.mainloop()
        self.app.quit()

    # Function to update the GUI
    def Update_GUI(self):
        self.get_logger().info("GUI Sensors updated")
        sensors={'windvel':self.Wind_message.wind_speed,'windangle':self.Wind_message,
                 'imuXacc':self.IMU_message.linear_acceleration_x,'imuYacc':self.IMU_message.linear_acceleration_y,
                 'imuZacc':self.IMU_message.linear_acceleration_z, 'imuRoll':self.IMU_message.roll, 'imuYaw':self.IMU_message.yaw,
                 'imuPitch':self.IMU_message.pitch,'latitude':self.Position_message.latitude,'longitude':self.Position_message.longitude}
        self.app.update_from_ros(sensors)
    def receive_sail_position(self,msg):
        self.get_logger().info('Rudder angle: "%f"'% msg.rudder_angle)
        self.rudder_msg.rudder_angle=msg.rudder_angle
        pass
    def receive_rudder_position(self,msg):
        self.get_logger().info('Sail angle: "%f"'% msg.sail_angle)
        self.sail_msg.sail_angle=msg.sail_angle
        pass
    # Function to update the value of position variables
    def receive_position(self,msg):
        self.get_logger().info('Latitude "%f"' % msg.latitude)
        self.get_logger().info('Longitude "%f"'% msg.longitude)
        self.Position_message.latitude=msg.latitude
        self.Position_message.longitude=msg.longitude

    # Function to update the value of wind sensor variables
    def receive_WD(self,msg):
        self.get_logger().info('Wind Speed: "%f"' % msg.wind_speed)
        self.get_logger().info('Wind Direction: "%i"' % msg.wind_angle)
        self.Wind_message.wind_angle=msg.wind_angle
        self.Wind_message.wind_speed=msg.wind_speed

    # Function to update the value of IMU variables
    def receive_IMU(self,msg):
        self.get_logger().info('Boat yaw: "%f"' % msg.yaw)
        self.get_logger().info('Boat pitch: "%f"' % msg.pitch)
        self.get_logger().info('Boat roll: "%f"' % msg.roll)
        self.get_logger().info('Boat x accelation: "%f"' % msg.linear_acceleration_x)
        self.get_logger().info('Boat y accelation: "%f"' % msg.linear_acceleration_y)
        self.get_logger().info('Boat z accelation: "%f"' % msg.linear_acceleration_z)
        self.IMU_message.linear_acceleration_x= msg.linear_acceleration_x
        self.IMU_message.linear_acceleration_y= msg.linear_acceleration_y
        self.IMU_message.linear_acceleration_z= msg.linear_acceleration_z
        self.IMU_message.yaw= msg.yaw
        self.IMU_message.pitch= msg.pitch
        self.IMU_message.roll= msg.roll

    
def main(args=None):
    rclpy.init(args=args) #ROS Comand Line Python init
    # Node
    node = basic_GUI()
    rclpy.spin(node) # Keep the node alive
    node.destroy_node()
    rclpy.shutdown()

# If you want it to run from the terminal
if __name__ == '__main__':
    main()
