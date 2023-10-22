#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64
from my_sailboat_controller.pid_controller import quaternion_from_euler,euler_from_quaternion,rudder_ctrl,verify_result,sail_ctrl,GPS_to_waypoint
from sensor_msgs.msg import MagneticField
import numpy as np
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
        self.waypoints=[[0,40],[30,60],[0,80],[-10,100],[0,125]] # Default config to test functioning
        self.base_lat=-33.724223
        self.base_long=150.679736
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

        # Subscribers
        self.GPS_position_subscriber = self.create_subscription(PositionMessage,'Position',self.receive_position,10)
        self.GPS_position_subscriber

        self.windsensor_subscriber = self.create_subscription(WindMessage,'Wind',self.receive_WD,10)
        self.windsensor_subscriber

        self.IMUpose_subscriber = self.create_subscription(IMUMessage,'IMU',self.receive_IMU,10)
        self.IMUpose_subscriber

        self.magnetometer_subscriber = self.create_subscription(MagneticField,'magnetic_field',self.receive_MAG,10)
        self.magnetometer_subscriber

        # Recieved variables
        self.IMU_message = IMUMessage()
        self.Wind_message = WindMessage()
        self.Position_message = PositionMessage()
        self.ManeticField_message = MagneticField()
        self.contador=0
        self.iant=0
        self.current_heading=0
        self.file=open('datos_de_navegacion.csv','w')
        self.file.write('heading,targetX,targetY,posX,posY,windir,rudder_angle,sail_angle\n')
        self.wind_dir=0.0

    # Function to update the rudder controller command
    def rudder_controller_callback(self):
        actualpos=self.GPS_to_Cartesian([self.Position_message.latitude,self.Position_message.longitude])
        self.rudder_msg.rudder_angle,self.iant=rudder_ctrl(actualpos,self.waypoints[self.contador],self.current_heading,self.iant)
        # self.rudder_msg.rudder_angle=self.rudder_msg.rudder_angle-temp
        # self.get_logger().info('Current heading: %s'%self.current_heading)
        # self.get_logger().info('Current position: %s'%actualpos)
        self.sail_controller_callback()
        datos=[str(self.current_heading),str(self.waypoints[self.contador][0]),str(self.waypoints[self.contador][1]),str(actualpos[0]),str(actualpos[1]),str(self.wind_dir),str(self.rudder_msg.rudder_angle),str(self.sail_msg.sail_angle)]
        result=','.join(datos)
        self.file.write(result+'\n')
        self.get_logger().info('Data: %s'%result)
        if verify_result(actualpos,self.waypoints[self.contador]):
            self.get_logger().info('llegó a uno de los destinos!')
            self.next_waypoint()
            self.iant=0
            # self.rudder_msg.rudder_angle=0.0
        self.rudder_publisher.publish(self.rudder_msg)
    
    #Function to evaluate next waypoint 
    def next_waypoint(self):
        self.contador+=1
        if self.contador==len(self.waypoints):
            self.contador=0
        if self.waypoint_type=='latlong':
            self.waypoints[self.contador]=self.GPS_to_Cartesian(latlong=self.waypoints[self.contador])
        self.get_logger().info('Next position: %s'%self.waypoints[self.contador])
    
    # Function to update the sail controller command
    def sail_controller_callback(self):
        self.sail_msg.sail_angle,self.wind_dir=sail_ctrl(self.Wind_message.wind_angle)
        # self.get_logger().info('Wind direction: %f'%float(self.Wind_message.wind_angle)*180/np.pi)
        # self.get_logger().info('Sail direction: %f'%float(self.sail_msg.sail_angle)*180/np.pi)
        self.sail_publisher.publish(self.sail_msg)

    # Function to update the value of position variables
    def receive_position(self,msg):
        # self.get_logger().info('Latitude "%f"' % msg.latitude)
        # self.get_logger().info('Longitude "%f"'% msg.longitude)
        self.Position_message.latitude=msg.latitude
        self.Position_message.longitude=msg.longitude

    # Function to update the value of wind sensor variables
    def receive_WD(self,msg):
        # self.get_logger().info('Wind Speed: "%f"' % msg.wind_speed)
        # self.get_logger().info('Wind Direction: "%i"' % msg.wind_angle)
        self.Wind_message.wind_angle=msg.wind_angle
        self.Wind_message.wind_speed=msg.wind_speed

    # Function to update the value of IMU variables
    def receive_IMU(self,msg):
        # self.get_logger().info('Boat yaw: "%f"' % msg.yaw)
        # self.get_logger().info('Boat pitch: "%f"' % msg.pitch)
        # self.get_logger().info('Boat roll: "%f"' % msg.roll)
        # self.get_logger().info('Boat x accelation: "%f"' % msg.linear_acceleration_x)
        # self.get_logger().info('Boat y accelation: "%f"' % msg.linear_acceleration_y)
        # self.get_logger().info('Boat z accelation: "%f"' % msg.linear_acceleration_z)
        self.IMU_message.linear_acceleration_x= msg.linear_acceleration_x
        self.IMU_message.linear_acceleration_y= msg.linear_acceleration_y
        self.IMU_message.linear_acceleration_z= msg.linear_acceleration_z
        self.IMU_message.yaw= msg.yaw
        self.IMU_message.pitch= msg.pitch
        self.IMU_message.roll= msg.roll
    def receive_MAG(self,msg):
        # self.get_logger().info('Magnetic field x: "%f"' % msg.magnetic_field.x)
        # self.get_logger().info('Boat pitch: "%f"' % msg.magnetic_field.y)
        self.current_heading=np.arctan2(msg.magnetic_field.y,msg.magnetic_field.x)
    #Functions to transform between coordinates systems
    def Cartesian_to_GPS(self,cartpos,R=120000):#cartesian_pos=[x,y] #R= Earth radius
        lati=cartpos[1]/R+self.base_lat
        longi=cartpos[0]/R+self.base_long
        return lati,longi     
    
    def GPS_to_Cartesian(self,latlong,R=120000):
        x=R*(latlong[1]-self.base_long)
        y=R*(latlong[0]-self.base_lat)
        return ([x,y])
    def __del__(self):
        if self.file is not None and not self.file.closed:
            self.file.close()
            print("Archivo cerrado.")
    
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