#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import my_sailboat_controller.communicate as cm
''' Para utilizar mensajes personalizados se usa una carpeta al nivel del paquete que contega
    una subcarpeta msg dentro de la cual esten los modelos (estructuras) de los mensajes que se
    van a utilizar, se crea un modelo dentro de la carpeta y se importa poniendo la carpeta como 
    dependencia en el archivo package.xlm 
'''
from autosail_message.msg import RudderControlMessage
from autosail_message.msg import SailAngleMessage
from autosail_message.msg import IMUMessage
from autosail_message.msg import WindMessage
from autosail_message.msg import PositionMessage

class decode_message(Node):
    def __init__(self):
        super().__init__('my_decoder_node')
        self.get_logger().info("Decoder is armed!!")
        self.declare_parameter('sensor_source','Ros')

        # Publishers
     
        self.GPS_position_publisher = self.create_publisher(PositionMessage,'Position',10)
        self.GPS_position_message=PositionMessage()

        self.windsensor_publisher= self.create_publisher(WindMessage,'Wind',10)
        self.windsensor_message=WindMessage()

        self.IMUpose_publisher= self.create_publisher(IMUMessage,'IMU',10)
        self.IMUpose_message=IMUMessage()


        # Serial
        self.serialport=cm.serial_initialization('dev/','ttyS3',15)#Setting port
        if not self.serialport:
            self.get_logger().info("Port could not be opened")
        else:
            self.get_logger().info("Port Openned")
        
        # Timers=>threads(?)
        self.create_timer(1.0,self.pico_decode)## Decode
        self.create_timer(1.0,self.pico_encode)

        # Subscribers
        self.rudder_message=RudderControlMessage()
        self.rudder_subscriber = self.create_subscription(RudderControlMessage,'rudder_ctrl',self.rudder_encoder,10)
        self.rudder_subscriber

        self.sail_message=SailAngleMessage()
        self.sail_subscriber = self.create_subscription(SailAngleMessage,'sail_ctrl',self.sail_encoder,10)
        self.sail_subscriber

        # self.create_timer(1.0,self.rudder_controller_callback)

    def pico_decode(self):
        sensor_source = self.get_parameter('sensor_source').get_parameter_value().string_value
        if sensor_source=="ROS":
            band = False
            [band,recibe] = cm.read_data()
            try:
                if band and self.serialport:
                    self.get_logger().info("Latitude: "+str(recibe['S1'])+" Longitude: "+str(recibe['S2']))
                    self.GPS_position_message.latitude=recibe['S1']
                    self.GPS_position_message.longitude=recibe['S2']
                    self.GPS_position_publisher.publish(self.GPS_position_message)
                    # angles = [recibe['S5'],recibe['S6'],recibe['S7']]
                    self.get_logger().info("Wind Speed: "+str(recibe['S5']))
                    self.windsensor_message.wind_speed = recibe['S5']
                    self.get_logger().info("Wind Angle: "+str(recibe['S8']))
                    self.windsensor_message.wind_angle= recibe['S8']
                    self.windsensor_publisher.publish(self.windsensor_message)
                    self.get_logger().info("Curse: "+str(recibe['S7'])+"°N")
                    self.IMUpose_message.yaw=recibe['S7']
                    self.get_logger().info("Pitch: "+str(recibe['S6']))
                    self.IMUpose_message.pitch=recibe['S6']
                    self.IMUpose_publisher.publish(self.IMUpose_message)
                    #band = [band]+position+angles+[wind]+speed
            except:
                band =  False
                self.get_logger().info("Data Error")

    def rudder_encoder(self,msg):
        self.rudder_message.rudder_angle=msg.rudder_angle
        
    def sail_encoder(self,msg):
        self.sail_message.sail_angle=msg.sail_angle

    def pico_encode(self):
        band=False
        if self.serialport:
            datos={'A1' : self.rudder_message.rudder_angle, 'A2' : self.sail_message.sail_angle}
                   #,'A3' : control_action[2], 'A4' : control_action[3]}
            band=cm.write_data(datos)
        return band

def main(args=None):
    rclpy.init(args=args)
    # Node
    node = decode_message()
    rclpy.spin(node)#keep the node alive
    node.destroy_node()
    rclpy.shutdown()

# If you want it to run from the terminal
if __name__ == '__main__':
    main()