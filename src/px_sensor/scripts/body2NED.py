#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from px_comm.msg import OpticalFlow
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header
from numpy import *
import tf
import math

# Nuevo
# from bebop_msgs import Imu
# from std_msgs.msg import Float32

q = array([0,0,0,1])
v = array([0,0,0])
vx_arr = [0, 0, 0, 0, 0]
vy_arr = [0, 0, 0, 0, 0]
z_prev = 0
t_prev = 0
z_now = 0
t_now = 0

yaw = 130.0/180 * math.pi  # from NED to Vicon frame

def imucallback(msg):
    # attitude measured in ENU frame
    global q
    q = array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])

def toNED(msg):
    # fliter measured velocity
    global vx_arr, vy_arr
    vx_arr.pop(0)
    vx_arr.append(msg.velocity_x)
    vy_arr.pop(0)
    vy_arr.append(msg.velocity_y)

    vx = 0.5*vx_arr[4]+0.2*vx_arr[3]+0.15*vx_arr[2]+0.1*vx_arr[1]+0.05*vx_arr[0]
    vy = 0.5*vy_arr[4]+0.2*vy_arr[3]+0.15*vy_arr[2]+0.1*vy_arr[1]+0.05*vy_arr[0]
    v_body = array([vx, -vy, 0]) * 1.2
    
    # transform body velocity to ENU 
    global q
    [qx, qy, qz, qw] = [q[0], q[1], q[2], q[3]]
    Tenu = array([[1-2*qy*qy-2*qz*qz,   2*qx*qy-2*qz*qw,   2*qx*qz+2*qy*qw],
                  [2*qx*qy + 2*qz*qw, 1-2*qx*qx-2*qz*qz,   2*qy*qz-2*qx*qw],
                  [2*qx*qz-2*qy*qw, 2*qy*qz + 2*qx*qw, 1-2*qx*qx-2*qy*qy]])

    v = dot(Tenu, v_body)
    
    # estimate z velocity by height derivative
    global z_now, t_now, z_prev, t_prev
    z_now = msg.ground_distance
    t_now = msg.header.stamp.to_sec()
    if ((t_now - t_prev) < 1):
        vz = (z_now-z_prev)/(t_now-t_prev)
    else:
        vz = 0

    # ENU to NED: (x,y,z) -> (x,-y,-z)
    twist = TwistStamped()
    twist.header = Header()
    twist.header.frame_id = "ned"
    twist.header.stamp = msg.header.stamp
    twist.twist.linear.x = v[0]
    twist.twist.linear.y = -v[1]
    twist.twist.linear.z = -vz
    pub.publish(twist)

    z_prev = z_now
    t_prev = t_now

    # record data in vicon frame, compare with vicon
    q_ned_vicon = tf.transformations.quaternion_from_euler(math.pi, 0, -yaw)
    [qx, qy, qz, qw] = [q_ned_vicon[0], q_ned_vicon[1], q_ned_vicon[2], q_ned_vicon[3]]
    Tv = array([[1-2*qy*qy-2*qz*qz,   2*qx*qy-2*qz*qw,   2*qx*qz+2*qy*qw],
                  [2*qx*qy + 2*qz*qw, 1-2*qx*qx-2*qz*qz,   2*qy*qz-2*qx*qw],
                  [2*qx*qz-2*qy*qw  , 2*qy*qz + 2*qx*qw, 1-2*qx*qx-2*qy*qy]])
    vr = dot(Tv, array([v[0],-v[1],0]))

    outtxt.write(str.format("{0:.9f} ", msg.header.stamp.to_sec()))
    outtxt.write(str.format("{0:.9f} ", vr[0]))
    outtxt.write(str.format("{0:.9f} ", vr[1]))
    outtxt.write(str.format("{0:.9f} ", vz))
    outtxt.write('0 ')
    outtxt.write('0 ')
    outtxt.write('0 ')
    outtxt.write('0\n')


if __name__ == '__main__':

    outtxt = open('/home/jitete/drones/src/px-ros-pkg/drivers/px4flow/script/flow5.txt','w')
    outtxt.write('# text file' + '\n# format: time stamp x y z qx qy qz qw\n')

    rospy.init_node('body2NED')
    rospy.Subscriber('/imu/data', Imu, imucallback)
    pub = rospy.Publisher('velocity', TwistStamped, queue_size=0)	
    rospy.Subscriber('/px_sensor/velx', Float32, toNED)
    rospy.spin()


