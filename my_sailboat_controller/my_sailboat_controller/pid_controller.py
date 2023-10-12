import math
import numpy as np
Kp = 1
Ki = 0
ianterior = 0
target_distance = 0
spHeading = 10
current_heading = 0
heeling = 0
windDir = 0
f_distance = 4 # Error Radius
rate_value = 10


def quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
  qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
  qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
  qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
 
  return [qx, qy, qz, qw]

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return ([roll_x, pitch_y, yaw_z]) # in radians


def angle_saturation(sensor):
    while sensor >180 or sensor < -180:
        if sensor > 180:
            sensor = sensor - 360
        if sensor < -180:
            sensor = sensor + 360
    return sensor


def P(erro):
    global Kp
    return Kp * erro


def Integral(erro):
    global Ki
    global ianterior
    global rate_value
    if (ianterior > 0 and erro < 0) or (ianterior < 0 and erro > 0):
        ianterior = ianterior + Ki * erro * 50 * (1. / rate_value)
    else:
        ianterior = ianterior + Ki * erro * (1. / rate_value)
    return ianterior


def rudder_ctrl(act_pos, dest_pos, euler):  # act_pos=[x1,y1] dest_pos=[x2,y2]  euler=[roll, pitch, yaw]
    global target_distance
    global spHeading
    global current_heading
    myradians = math.atan2(dest_pos[1] - act_pos[1], dest_pos[0] - act_pos[0])
    sp_angle = math.degrees(myradians)-90 #Se restan 90 puesto que el angulo que entrega la tangente esta medido desde el eje x sin embargo el angulo que entregan los sensores esta medido desde el norte que puede ser asemejado al eje Y
    target_distance = math.hypot(dest_pos[0] - act_pos[0], dest_pos[1] - act_pos[1])
    target_angle = euler[2]
    sp_angle = angle_saturation(sp_angle)
    spHeading = sp_angle
    sp_angle = -sp_angle
    target_angle = angle_saturation(target_angle)
    target_angle = -target_angle
    current_heading = math.radians(target_angle)
    err = sp_angle - target_angle
    err = angle_saturation(err)
    err = P(err) + Integral(err)
    rudder_angle = err / 2
    if err > 60:
        err = 60
    if err < -60:
        err = -60
    return rudder_angle


def sail_ctrl(global_dir):
    wind_dir = global_dir - math.degrees(current_heading) # Direccion del viento aparente. Viento 
    wind_dir = angle_saturation(wind_dir+180) # offset=180 para el servo
    sail_angle = math.radians(wind_dir)/2 # la vela se ubica en la mitad del viento aparente
    if math.degrees(sail_angle) < -80:
        sail_angle = -sail_angle
    return math.degrees(-sail_angle)

def GPS_to_waypoint(lati,longi,R=6378000): #latitude=degrees,longitude=degrees R=planetRadius in m
    lon1 = math.radians(longi)
    lat1 = math.radians(lati)
    h=math.sin((lat1[0]-lat1[1])/2)**2+math.cos(lat1[0])*np.cos(lat1[1])*math.sin((lon1[0]-lon1[1])/2)**2
    d=2*R*np.arcsin(math.sqrt(h))

    return d

def verify_result():
    global target_distance
    global f_distance
    if target_distance < f_distance:
        result = 1
    if target_distance >= f_distance:
        result = 0
    return result
