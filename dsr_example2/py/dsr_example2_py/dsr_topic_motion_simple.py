#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ##
# @brief    Simple relay making possible to move the robot through a simple ROS topic.
# @author   Tamas D. Nagy (tamas.daniel.nagy@irob.uni-obuda.hu)


import rclpy
import sys
import time
import signal

from dsr_msgs2.msg import *
from dsr_msgs2.srv import *
from sensor_msgs.msg import JointState

g_node = None
rclpy.init()
g_node = rclpy.create_node("dsr_topic_motion_simple_py")
srv = g_node.create_client(MoveJoint, 'motion/move_joint')

def set_robot_mode(robot_mode):
    global g_node

    srv = g_node.create_client(SetRobotMode, 'system/set_robot_mode')

    req = SetRobotMode.Request()
    req.robot_mode = robot_mode

    future = srv.call_async(req)
    rclpy.spin_until_future_complete(g_node, future)

    result = future.result()
    if result == None:
        ret = -1
    else:
        ret = 0 if (result.success == True) else -1
    return ret
def movej(pos, vel=None, acc=None, time=0.0, radius=0.0, mod= 0, ra=0):
    global g_node
    global srv

    req = MoveJoint.Request()

    req.pos         = [float(x) for x in pos]
    req.vel         = float(vel)
    req.acc         = float(acc)
    req.time        = float(time)
    req.radius      = float(radius)
    req.mode        = int(mod)
    req.blend_type  = int(ra)
    req.sync_type   = 0

    future = srv.call_async(req)

    # This part is commented out because
    # causes program to stuck in here
    #rclpy.spin_until_future_complete(g_node, future)

    #try:
    #    result = future.result()
    #except Exception as e:
    #    g_node.get_logger().info('movej Service call failed %r' % (e,))
    #else:
    #    if result == None:
    #        ret = -1
    #    else:
    #        ret = 0 if (result.success == True) else -1

    #g_node.get_logger().info('movej done.')
    #return ret


def signal_handler(sig, frame):
    print('XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX signal_handler')
    global g_node
    publisher = g_node.create_publisher(RobotStop, 'stop', 10)

    msg = RobotStop()
    msg.stop_mode =1

    publisher.publish(msg)
    #sys.exit(0)
    rclpy.shutdown()

def cb_joint_cmd(msg):
    global g_node
    g_node.get_logger().info('Joint command received...')
    p1 = [msg.position[0], msg.position[1], msg.position[2],
          msg.position[3], msg.position[4], msg.position[5]]
    movej(p1, vel=180, acc=6000)
    #g_node.get_logger().info('Movement done.')



def main(args=None):
    global g_node
    signal.signal(signal.SIGINT, signal_handler)
    g_node.create_subscription(JointState, '/joint_cmd', cb_joint_cmd, 10)
    g_node.get_logger().info('DSR topic controller launched.')

    rclpy.spin(g_node)

    g_node.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()
