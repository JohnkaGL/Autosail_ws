#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64
from .pid_controller import quaternion_from_euler,euler_from_quaternion,rudder_ctrl,verify_result,sail_ctrl,GPS_to_waypoint
''' Para utilizar mensajes personalizados se usa una carpeta al nivel del paquete que contega
    una subcarpeta msg dentro de la cual esten los modelos (estructuras) de los mensajes que se
    van a utilizar, se crea un modelo dentro de la carpeta y se importa poniendo la carpeta como 
    dependencia en el archivo package.xlm 
'''

from autosail_message.msg import IMUMessage
from autosail_message.msg import WindMessage
from autosail_message.msg import PositionMessage
from autosail_message.msg import RudderControlMessage
from autosail_message.msg import SailAngleMessage

# Node description and architecture 
class basic_controller(Node):
    def __init__(self):
        super().__init__('my_controller_node')
        self.get_logger().info("Controller is armed!!")

        # Config Constants
        self.waypoints=[[0,5],[3,6],[8,6],[8,0],[1,0]] # Default config to test functioning
        self.base_lat=6.267286
        self.base_long=-75.569320
        self.waypoint_type='cartesian'

        # Variables
        self.next_x=0
        self.next_y=0
        self.contador=0
        
        # Publishers
        # General form
        # self.[name]_publisher = self.create_publiser([Type_of_message_imported], 'Topic', [message_stack_size])
        # self.[name]_msg= [Type_of_message_imported]()
        # self.create_timer([rate_in_seconds],[callback_function_name])
        
        self.rudder_publisher = self.create_publisher(RudderControlMessage,'rudder_ctrl',10)
        self.rudder_msg = RudderControlMessage()
        self.create_timer(1.0,self.rudder_controller_callback)

        self.sail_publisher = self.create_publisher(SailAngleMessage,'sail_ctrl',10)
        self.sail_msg = SailAngleMessage()
        self.create_timer(1.0,self.sail_controller_callback)

        # Subscribers
        self.GPS_position_subscriber = self.create_subscription(PositionMessage,'Position',self.receive_position,10)
        self.GPS_position_subscriber

        self.windsensor_subscriber = self.create_subscription(WindMessage,'Wind',self.receive_WD,10)
        self.windsensor_subscriber

        self.IMUpose_subscriber = self.create_subscription(IMUMessage,'IMU',self.receive_IMU,10)
        self.IMUpose_subscriber

        # self.magnetometer_subscriber = self.create_subscription(IMUMessage,'IMU',self.receive_IMU,10)
        # self.magnetometer_subscriber

        # Recieved variables
        self.IMU_message = IMUMessage()
        self.Wind_message = WindMessage()
        self.Position_message = PositionMessage()

    # Function to update the rudder controller command
    def rudder_controller_callback(self):
        if not verify_result():
            #rudder_ctrl()
            actualpos=[self.Position_message.latitude,self.Position_message.longitude]
            pose_euler=[self.IMU_message.roll,self.IMU_message.pitch,self.IMU_message.yaw]
            self.rudder_msg.rudder_angle=float(rudder_ctrl(actualpos,self.waypoints[self.contador],pose_euler))
            self.get_logger().info('Rudder direction: %f'%self.rudder_msg.rudder_angle)
            self.rudder_publisher.publish(self.rudder_msg)
        else:
            self.next_waypoint()
    
    #Function to evaluate next waypoint 
    def next_waypoint(self):
        self.contador+=1
        if self.waypoint_type=='latlong':
            self.waypoints[self.contador]=self.GPS_to_Cartesian(latlong=self.waypoints[self.contador])
            #self.waypoints[self.contador]=self.Cartesian_to_GPS(cartpos=self.waypoints[self.contador])
    
    # Function to update the sail controller command
    def sail_controller_callback(self):
        self.sail_msg.sail_angle=sail_ctrl(self.Wind_message.wind_angle)
        self.get_logger().info('Sail direction: %s'%self.sail_msg.sail_angle)
        self.sail_publisher.publish(self.sail_msg)

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

    #Functions to transform between coordinates systems
    def Cartesian_to_GPS(self,cartpos,R=6378000):#cartesian_pos=[x,y] #R= Earth radius
        lati=cartpos[1]/R+self.base_lat
        longi=cartpos[0]/R+self.base_long
        return lati,longi     
    
    def GPS_to_Cartesian(self,latlong,R=6378000):
        x=R*(latlong[1]-self.base_long)
        y=R*(latlong[0]-self.base_lat)
        return ([x,y])

    
def main(args=None):
    rclpy.init(args=args) #ROS Comand Line Python init
    # Node
    node = basic_controller()
    rclpy.spin(node) # Keep the node alive
    rclpy.shutdown()
    node.destroy_node()

# If you want it to run from the terminal
if __name__ == '__main__':
    main()