#!/usr/bin/env python
import sys
import rospy
import time
from tm_motion.srv import *
import actionlib
from tm_motion.msg import ActionAction, ActionGoal
from om_aiv_navigation.msg import ActionAction, ActionGoal
from pymodbus.client.sync import ModbusTcpClient
import tf
import tf2_ros
import geometry_msgs.msg
import math
host = '192.168.1.2'
port_modbus = 502
client = ModbusTcpClient(host, port_modbus)
client.connect()

def start_program():
    print "starting program"
    status = client.write_coil(7104, True, unit=1)
    print(status)
    time.sleep(6)

def stop_program():
    print "stopping program"
    status = client.write_coil(7105, True, unit=1)
    print(status)
    time.sleep(2)

def grip():
    print "griping object"
    #go into gipper function
    status = client.write_coil(0003, False, unit=1)
    print(status)
    status = client.write_coil(0002, True, unit=1)
    print(status)
    time.sleep(1)
    status = client.write_coil(0000, True, unit=1)
    time.sleep(1)

def release():
    #go into gipper function
    status = client.write_coil(0002, True, unit=1)
    print(status)
    time.sleep(1)
    #open gripper
    print "opening gripper"
    status = client.write_coil(0001, True, unit=1)
    time.sleep(1)

def landmark_location_service_client():
    global nc
    rospy.wait_for_service('landmark_location')
    try:
        service = rospy.ServiceProxy('landmark_location', TmMotion)
        resp1 = service()
        nc  = resp1.device
        return resp1.device
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def feedback_cb(msg):
 print 'Feedback received:', msg

def call_server():
    client = actionlib.SimpleActionClient('move_action', ActionAction)
    client.wait_for_server()
    client.send_goal(goal, feedback_cb=feedback_cb)
    client.wait_for_result()
    result = client.get_result()
    return result

def call_ld_server():
    client = actionlib.SimpleActionClient('goTo', ActionAction)
    client.wait_for_server()
    client.send_goal_and_wait(goal)
    client.wait_for_result()
    result = client.get_result()
    return result

if __name__ == "__main__":
    rospy.init_node('moma')
    print "REMEMBER TO PUT TM ARM IN SAFE POSITION"

    start_program()
    from tm_motion.msg import ActionAction, ActionGoal
    print "tm moving to pick location to scan tm landmark"
    try:
        goal = ActionGoal()
        goal.goal_goal1 = 440.52
        goal.goal_goal2 = 362.69
        goal.goal_goal3 = 706.25
        goal.goal_goal4 = 178.59
        goal.goal_goal5 = 0.93
        goal.goal_goal6 = 135.04
        result = call_server()
        print 'The result is:', result
    except rospy.ROSInterruptException as e:
        print 'Something went wrong:', e

    print "scanning for tm landmark location"
    print landmark_location_service_client()
    rate = rospy.Rate(10.0)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('base_link', 'landmark_location', rospy.Time())
            print "landmark_location wrt base_link"
            print trans.transform
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
    #input values to move to here
    broadcaster3 = tf2_ros.StaticTransformBroadcaster()
    entry_to_pod = geometry_msgs.msg.TransformStamped()
    entry_to_pod.header.stamp = rospy.Time.now()
    entry_to_pod.header.frame_id = "base_link"
    entry_to_pod.child_frame_id = "object_location"
    entry_to_pod.transform.translation.x = 419.67
    entry_to_pod.transform.translation.y = 460.16
    entry_to_pod.transform.translation.z = 680.94
    quat = tf.transformations.quaternion_from_euler(
               -178.26,-1.37,-46.13)
    entry_to_pod.transform.rotation.x = quat[0]
    entry_to_pod.transform.rotation.y = quat[1]
    entry_to_pod.transform.rotation.z = quat[2]
    entry_to_pod.transform.rotation.w = quat[3]
    broadcaster3.sendTransform(entry_to_pod)

    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('landmark_location', 'object_location', rospy.Time())
            print "object_location wrt landmark_location"
            print trans.transform
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
    quaternion = (
    trans.transform.rotation.x,
    trans.transform.rotation.y,
    trans.transform.rotation.z,
    trans.transform.rotation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    Rx = math.degrees(euler[0])
    Ry = math.degrees(euler[1])
    Rz = math.degrees(euler[2])
    print trans.transform.translation.x
    print trans.transform.translation.y
    print trans.transform.translation.z
    print Rx
    print Ry
    print Rz
