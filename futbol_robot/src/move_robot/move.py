#!/usr/bin/env python
#import roslib; roslib.load_manifest('move_robot')
import numpy as np
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import usb.core
import usb.util
import sys, select, termios, tty

rospy.init_node('base_controller')
odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)


###################################################3
# Robot Dimensions
alphasDeg = [45, 45+90, 45+2*90, 45+3*90]
# alphasDeg = [40, 180-40, 180+40, 360-40]
# alphasDeg = [50, 180-50, 180+50, 360-50]
alphas = np.radians(alphasDeg)
# D matrix
Dmat = np.array( [ [-np.sin(alphas[0]), np.cos(alphas[0]), 1],
                   [-np.sin(alphas[1]), np.cos(alphas[1]), 1],
                   [-np.sin(alphas[2]), np.cos(alphas[2]), 1],
                   [-np.sin(alphas[3]), np.cos(alphas[3]), 1] ] )
# USB descriptor
ep = None
# Package
vx = []
vy = []
vw = []
kx = []
kz = []
sp = []

MAX_PWM = 300
wheel_diameter=2/1000;#put here the real value
msg = []


def init():
    global ep
    # find our device
    dev = usb.core.find(idVendor=0x04d2, idProduct=0x0001)
    # was it found?
    if dev is None:
        raise ValueError('Device not found')
    # set the active configuration. With no arguments, the first
    # configuration will be the active one
    reattach = False
    if dev.is_kernel_driver_active(0):
        reattach = True
        dev.detach_kernel_driver(0)
    dev.set_configuration()
    # get an endpoint instance
    cfg = dev.get_active_configuration()
    intf = cfg[(0, 0)]
    ep = usb.util.find_descriptor(
        intf,
        # match the first OUT endpoint
        custom_match= \
        lambda e: \
            usb.util.endpoint_direction(e.bEndpointAddress) == \
            usb.util.ENDPOINT_OUT)
    assert ep is not None


def package_init(num_robots):
    global vx, vy, vw
    vx = [0.0] * num_robots
    vy = [0.0] * num_robots
    vw = [0.0] * num_robots

def checkSum(msg):
    ch = msg[0]
    for i in range(7):
        ch ^= (msg[i+1]<<(i+1))
    chsum = (ch & 0xFF) ^ (ch >> 8)
    return chsum

def getMsg(id, w1, w2, w3, w4, rod, pot, type, prog):
    dir1 = 0 if w1 > 0 else 1;
    dir2 = 0 if w2 > 0 else 1;
    dir3 = 0 if w3 > 0 else 1;
    dir4 = 0 if w4 > 0 else 1;
    w1 = int(abs(w1))
    w2 = int(abs(w2))
    w3 = int(abs(w3))
    w4 = int(abs(w4))
    msg = [id, w1&0xFF, w2&0xFF, w3&0xFF, w4&0xFF]
    b5 = (dir1<<7) | ((w1&0x100)>>2) | (dir2<<5) | ((w2&0x100)>>4) | (dir3<<3) | ((w3&0x100)>>6) | (dir4<<1) | ((w4&0x100)>>8)
    msg += [b5]
    if type:
        msg += [pot&0x7F | 0x80] #aereo
    else:
        msg += [pot & 0x7F]
    if prog:
        msg += [ 0x40 | rod&0x3F] #pateo programado
    else:
        msg += [0x00 | rod & 0x3F]
    msg += [checkSum(msg)]
    return msg


def send( ROBOTS_IN_FIELD ):
    global Dmat, msg
    msg = []
    for cRobot in ROBOTS_IN_FIELD:
        if cRobot != -1:
            motVel = np.matmul(Dmat, [[vy[cRobot]], [vx[cRobot]], [vw[cRobot]]])
            motVel[0] = motVel[0] if motVel[0] <= 1.0 else 1.0
            motVel[1] = motVel[1] if motVel[1] <= 1.0 else 1.0
            motVel[2] = motVel[2] if motVel[2] <= 1.0 else 1.0
            motVel[3] = motVel[3] if motVel[3] <= 1.0 else 1.0
            n= 60/pi*wheel_diameter #here u've to put the relation between the rpm and the pwm, this is the relation between linear v and rpm
            motPwm = motVelRpm*n * MAX_PWM ## change this with correct values of pwm values

            msg = msg + getMsg(cRobot, motPwm[0][0], motPwm[1][0], motPwm[2][0], motPwm[3][0],  0, 0, False, False)
    ep.write(msg)

data=[]
def cmd_Veldata(msg):

    data[0]=msg.linear.x
    data[1]=msg.linear.y
    data[2]=msg.angular.z

###################################### ros publisher

init()

rospy.Subscriber("cmd_vel",Twist,cmd_Veldata)
rospy.spin()
while not rospy.is_shutdown():


    current_time = rospy.Time.now()


    # compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).to_sec()
    vx = data[0]
    vy = data[1]
    vth = data[2]
    delta_x = (vx * cos(th) - vy * sin(th)) * dt
    delta_y = (vx * sin(th) + vy * cos(th)) * dt
    delta_th = vth * dt

    x += delta_x
    y += delta_y
    th += delta_th

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    # publish the message
    odom_pub.publish(odom)

    last_time = current_time
    r.sleep()
