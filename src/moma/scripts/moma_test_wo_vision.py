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

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    landmark_to_obj = geometry_msgs.msg.TransformStamped()
    landmark_to_obj.header.stamp = rospy.Time.now()
    landmark_to_obj.header.frame_id = "base_link"
    landmark_to_obj.child_frame_id = "pod_location"
    landmark_to_obj.transform.translation.x = 318.96
    landmark_to_obj.transform.translation.y = 276.43
    landmark_to_obj.transform.translation.z = 729.98
    quat = tf.transformations.quaternion_from_euler(
               -178.12,-1.73,-42.14)
    landmark_to_obj.transform.rotation.x = quat[0]
    landmark_to_obj.transform.rotation.y = quat[1]
    landmark_to_obj.transform.rotation.z = quat[2]
    landmark_to_obj.transform.rotation.w = quat[3]
    broadcaster.sendTransform(landmark_to_obj)

    rate = rospy.Rate(10.0)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('base_link', 'pod_location', rospy.Time())
            print "pod wrt base:"
            print trans.transform
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

    quaternion = (
    quat[0],
    quat[1],
    quat[2],
    quat[3])
    euler = tf.transformations.euler_from_quaternion(quaternion)
    Rx = math.degrees(euler[0])
    Ry = math.degrees(euler[1])
    Rz = math.degrees(euler[2])
    print Rx
    print Ry
    print Rz

    broadcaster3 = tf2_ros.StaticTransformBroadcaster()
    entry_to_pod = geometry_msgs.msg.TransformStamped()
    entry_to_pod.header.stamp = rospy.Time.now()
    entry_to_pod.header.frame_id = "base_link"
    entry_to_pod.child_frame_id = "entry"
    entry_to_pod.transform.translation.x = 443.78
    entry_to_pod.transform.translation.y = 479.99
    entry_to_pod.transform.translation.z = 565.03
    quat = tf.transformations.quaternion_from_euler(
               -178.12,-1.73,-42.14)
    entry_to_pod.transform.rotation.x = quat[0]
    entry_to_pod.transform.rotation.y = quat[1]
    entry_to_pod.transform.rotation.z = quat[2]
    entry_to_pod.transform.rotation.w = quat[3]
    broadcaster3.sendTransform(entry_to_pod)




    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('base_link', 'entry', rospy.Time())
            print "entry wrt base:"
            print trans.transform
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('pod_location', 'entry', rospy.Time())
            print "entry wrt pod_location:"
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

    exit(0)

    from tm_motion.msg import ActionAction, ActionGoal
    print "tm moving to entry position"
    try:
        goal = ActionGoal()
        goal.goal_goal1 = trans.transform.translation.x
        goal.goal_goal2 = trans.transform.translation.y
        goal.goal_goal3 = trans.transform.translation.z
        # goal.goal_goal4 = Rx
        # goal.goal_goal5 = Ry
        # goal.goal_goal6 = Rz
        goal.goal_goal4 = 179.81
        goal.goal_goal5 = -1.51
        goal.goal_goal6 = -43.95
        result = call_server()
        print 'The result is:', result
        print "moved to position to pick object"
    except rospy.ROSInterruptException as e:
        print 'Something went wrong:', e

    exit(0)
