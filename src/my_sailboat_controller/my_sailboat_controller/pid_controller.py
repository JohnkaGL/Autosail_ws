import math
import numpy as np

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
    """
    This function gets de variable into the range of -180 to 180 degrees
    """
    while sensor >180 or sensor < -180:
        if sensor > 180:
            sensor = sensor - 360
        if sensor < -180:
            sensor = sensor + 360
    return sensor


def P(erro):
    Kp=0.25
    return Kp * erro


def Integral(erro,ianterior):
    Ki = 0.04
    rate_value=100
    if (ianterior > 0 and erro < 0) or (ianterior < 0 and erro > 0):
        ianterior = ianterior + Ki * erro*50 * (1. / rate_value)
    else:
        ianterior = ianterior + Ki * erro * (1. / rate_value)
    return ianterior


def rudder_ctrl(act_pos, dest_pos, heading_angle,ant):  # act_pos=[x1,y1] dest_pos=[x2,y2] heading_angle=[heading_radians]
    """Simple PI controller asuming cartesian coordinates of the world and getting information of the magnetometer sensor
    positioned with Y axes parallel to boat head, and north not necesarelly corresponding to the Y axes of the world.
    """
    rudder_angle_correction=90
    myradians = math.atan2(dest_pos[1] - act_pos[1], dest_pos[0] - act_pos[0]) # desired heading angle meassured from world's x axes
    target_angle = math.degrees(myradians)-rudder_angle_correction # desired heading angle corrected to have a reference from north
    current_heading = math.degrees(heading_angle)-90 # current heading angle corrected to have a reference from north
    err = target_angle - current_heading # if the target angle and the current heading are equal we dont have to correct the heading
    err = angle_saturation(err)#prevent going out of bounds
    iant= err
    correction = P(err) + Integral(err,ant)  # Digital PI
    #If the correction is too big, the rudder will just flip the boat gradually to the most efficient direction
    if correction > 60:
        correction = 60
    if correction < -60:
        correction = -60
    return float(math.radians(-correction)), iant

def sail_ctrl(global_dir):
    wind_correction=180
    wind_dir = global_dir+wind_correction# Apparent wind direction 
    wind_dir = angle_saturation(wind_dir) # wind_angle is meassured from the head  
    if wind_dir>=0:
        sail_angle = math.radians(wind_dir+90)/2 # the sail goes in the middle range of the apparen
        if math.degrees(sail_angle) > 90:
            sail_angle=sail_angle-math.pi
        else:
            sail_angle = math.pi/2-sail_angle
    elif wind_dir <0:
        sail_angle = math.radians(wind_dir-90)/2 
        if math.degrees(sail_angle)<-90:
            sail_angle=sail_angle+math.pi
        else:
            sail_angle = -math.pi/2-sail_angle

    return float(-sail_angle),wind_dir
def GPS_to_waypoint(lati,longi,R=6378000): #latitude=degrees,longitude=degrees R=planetRadius in m
    lon1 = math.radians(longi)
    lat1 = math.radians(lati)
    h=math.sin((lat1[0]-lat1[1])/2)**2+math.cos(lat1[0])*np.cos(lat1[1])*math.sin((lon1[0]-lon1[1])/2)**2
    d=2*R*np.arcsin(math.sqrt(h))

    return d

def verify_result(act_pos, dest_pos):
    target_distance = math.hypot(dest_pos[0] - act_pos[0], dest_pos[1] - act_pos[1]) # distance between the boat and the next way point
    f_distance=5
    if target_distance < f_distance:
        result = 1
    if target_distance >= f_distance:
        result = 0
    return result

# def setPI_const(integralk=0,proportional=1):
#     global Kp
#     global Ki
#     Kp=proportional
#     Ki=integralk