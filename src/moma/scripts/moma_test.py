#!/usr/bin/env python
import sys
import rospy
import socket
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

    broadcaster2 = tf2_ros.StaticTransformBroadcaster()
    landmark_to_obj = geometry_msgs.msg.TransformStamped()
    landmark_to_obj.header.stamp = rospy.Time.now()
    landmark_to_obj.header.frame_id = "landmark_location"
    landmark_to_obj.child_frame_id = "object_location"
    landmark_to_obj.transform.translation.x = 1.19036159068
    landmark_to_obj.transform.translation.y = 238.103980033
    landmark_to_obj.transform.translation.z = -229.23858539
    # quat = tf.transformations.quaternion_from_euler(
    #            -177.636227475,38.2654257937,--11.8564440799)
    quat = tf.transformations.quaternion_from_euler(
               0,0,0)
    landmark_to_obj.transform.rotation.x = quat[0]
    landmark_to_obj.transform.rotation.y = quat[1]
    landmark_to_obj.transform.rotation.z = quat[2]
    landmark_to_obj.transform.rotation.w = quat[3]
    broadcaster2.sendTransform(landmark_to_obj)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('base_link', 'object_location', rospy.Time())
            print "object_location wrt base_link:"
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

    # exit(0)


    from tm_motion.msg import ActionAction, ActionGoal
    print "tm moving to safe postion to rotate arm"
    try:
        goal = ActionGoal()
        # goal.goal_goal1 = float(values[0])
        # goal.goal_goal2 = float(values[1])
        # goal.goal_goal3 = float(values[2])
        # goal.goal_goal4 = float(values[3])
        # goal.goal_goal5 = float(values[4])
        # goal.goal_goal6 = float(values[5])
        goal.goal_goal1 = 318.56
        goal.goal_goal2 = 182.78
        goal.goal_goal3 = 577.43
        goal.goal_goal4 = -179.14
        goal.goal_goal5 = 2.53
        goal.goal_goal6 = 139.78
        result = call_server()
        print 'The result is:', result
    except rospy.ROSInterruptException as e:
        print 'Something went wrong:', e

    from tm_motion.msg import ActionAction, ActionGoal
    print "tm rotating arm"
    try:
        goal = ActionGoal()
        # goal.goal_goal1 = float(values[0])
        # goal.goal_goal2 = float(values[1])
        # goal.goal_goal3 = float(values[2])
        # goal.goal_goal4 = float(values[3])
        # goal.goal_goal5 = float(values[4])
        # goal.goal_goal6 = float(values[5])
        goal.goal_goal1 = 194.16
        goal.goal_goal2 = 66.05
        goal.goal_goal3 = 683.99
        goal.goal_goal4 = 179.40
        goal.goal_goal5 = -2.49
        goal.goal_goal6 = -40.45
        result = call_server()
        print 'The result is:', result
    except rospy.ROSInterruptException as e:
        print 'Something went wrong:', e

    from tm_motion.msg import ActionAction, ActionGoal
    print "tm moving to pick object"
    try:
        goal = ActionGoal()
        goal.goal_goal1 = trans.transform.translation.x
        goal.goal_goal2 = trans.transform.translation.y
        goal.goal_goal3 = trans.transform.translation.z
        goal.goal_goal4 = Rx
        goal.goal_goal5 = Ry
        goal.goal_goal6 = Rz
        # goal.goal_goal4 = 179.91
        # goal.goal_goal5 = -1.50
        # goal.goal_goal6 = -42.80
        result = call_server()
        print 'The result is:', result
        print "moved to position to pick object"
    except rospy.ROSInterruptException as e:
        print 'Something went wrong:', e

    broadcaster2 = tf2_ros.StaticTransformBroadcaster()
    obj1_to_obj2 = geometry_msgs.msg.TransformStamped()
    obj1_to_obj2.header.stamp = rospy.Time.now()
    obj1_to_obj2.header.frame_id = "object_location"
    obj1_to_obj2.child_frame_id = "object_location2"
    obj1_to_obj2.transform.translation.x = 0
    obj1_to_obj2.transform.translation.y = -180
    obj1_to_obj2.transform.translation.z = 0
    # quat = tf.transformations.quaternion_from_euler(
    #            86.1037342411,85.5429655101,-74.7943630375+360)
    quat = tf.transformations.quaternion_from_euler(
               0,0,0)
    obj1_to_obj2.transform.rotation.x = quat[0]
    obj1_to_obj2.transform.rotation.y = quat[1]
    obj1_to_obj2.transform.rotation.z = quat[2]
    obj1_to_obj2.transform.rotation.w = quat[3]
    broadcaster2.sendTransform(obj1_to_obj2)

    while not rospy.is_shutdown():
        try:
            trans2 = tfBuffer.lookup_transform('base_link', 'object_location2', rospy.Time())
            print "object_location2 wrt base_link:"
            print trans2.transform
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rate.sleep()
            print e
            print "waiting"
            continue
    quaternion = (
    trans2.transform.rotation.x,
    trans2.transform.rotation.y,
    trans2.transform.rotation.z,
    trans2.transform.rotation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    Rx = math.degrees(euler[0])
    Ry = math.degrees(euler[1])
    Rz = math.degrees(euler[2])
    print trans2.transform.translation.x
    print trans2.transform.translation.y
    print trans2.transform.translation.z
    print Rx
    print Ry
    print Rz

    from tm_motion.msg import ActionAction, ActionGoal
    print "tm moving in to pick object"
    try:
        goal = ActionGoal()
        goal.goal_goal1 = trans2.transform.translation.x
        goal.goal_goal2 = trans2.transform.translation.y
        goal.goal_goal3 = trans2.transform.translation.z
        goal.goal_goal4 = Rx
        goal.goal_goal5 = Ry
        goal.goal_goal6 = Rz
        # goal.goal_goal4 = 179.91
        # goal.goal_goal5 = -1.50
        # goal.goal_goal6 = -42.80
        result = call_server()
        print 'The result is:', result
        print "moved to position to pick object"
    except rospy.ROSInterruptException as e:
        print 'Something went wrong:', e

    # print "end"
    # exit(0)

    broadcaster3 = tf2_ros.StaticTransformBroadcaster()
    obj2_to_obj3 = geometry_msgs.msg.TransformStamped()
    obj2_to_obj3.header.stamp = rospy.Time.now()
    obj2_to_obj3.header.frame_id = "object_location"
    obj2_to_obj3.child_frame_id = "object_location3"
    obj2_to_obj3.transform.translation.x = 0
    obj2_to_obj3.transform.translation.y = -180
    obj2_to_obj3.transform.translation.z = -70
    # quat = tf.transformations.quaternion_from_euler(
    #            86.1037342411,85.5429655101,-74.7943630375+360)
    quat = tf.transformations.quaternion_from_euler(
               0,0,0)
    obj2_to_obj3.transform.rotation.x = quat[0]
    obj2_to_obj3.transform.rotation.y = quat[1]
    obj2_to_obj3.transform.rotation.z = quat[2]
    obj2_to_obj3.transform.rotation.w = quat[3]
    broadcaster3.sendTransform(obj2_to_obj3)

    while not rospy.is_shutdown():
        try:
            trans3 = tfBuffer.lookup_transform('base_link', 'object_location3', rospy.Time())
            print "object_location3 wrt base_link:"
            print trans3.transform
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rate.sleep()
            print e
            print "waiting2"
            continue
    quaternion = (
    trans3.transform.rotation.x,
    trans3.transform.rotation.y,
    trans3.transform.rotation.z,
    trans3.transform.rotation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    Rx = math.degrees(euler[0])
    Ry = math.degrees(euler[1])
    Rz = math.degrees(euler[2])
    print trans3.transform.translation.x
    print trans3.transform.translation.y
    print trans3.transform.translation.z
    print Rx
    print Ry
    print Rz

    from tm_motion.msg import ActionAction, ActionGoal
    print "tm moving up to pick object"
    try:
        goal = ActionGoal()
        goal.goal_goal1 = trans3.transform.translation.x
        goal.goal_goal2 = trans3.transform.translation.y
        goal.goal_goal3 = trans3.transform.translation.z
        goal.goal_goal4 = Rx
        goal.goal_goal5 = Ry
        goal.goal_goal6 = Rz
        # goal.goal_goal4 = 179.91
        # goal.goal_goal5 = -1.50
        # goal.goal_goal6 = -42.80
        result = call_server()
        print 'The result is:', result
        print "pick object"
    except rospy.ROSInterruptException as e:
        print 'Something went wrong:', e

    broadcaster4 = tf2_ros.StaticTransformBroadcaster()
    obj2_to_obj3 = geometry_msgs.msg.TransformStamped()
    obj2_to_obj3.header.stamp = rospy.Time.now()
    obj2_to_obj3.header.frame_id = "object_location"
    obj2_to_obj3.child_frame_id = "object_location4"
    obj2_to_obj3.transform.translation.x = 0
    obj2_to_obj3.transform.translation.y = 100
    obj2_to_obj3.transform.translation.z = -70
    # quat = tf.transformations.quaternion_from_euler(
    #            86.1037342411,85.5429655101,-74.7943630375+360)
    quat = tf.transformations.quaternion_from_euler(
               0,0,0)
    obj2_to_obj3.transform.rotation.x = quat[0]
    obj2_to_obj3.transform.rotation.y = quat[1]
    obj2_to_obj3.transform.rotation.z = quat[2]
    obj2_to_obj3.transform.rotation.w = quat[3]
    broadcaster4.sendTransform(obj2_to_obj3)

    while not rospy.is_shutdown():
        try:
            trans4 = tfBuffer.lookup_transform('base_link', 'object_location4', rospy.Time())
            print "object_location4 wrt base_link:"
            print trans4.transform
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rate.sleep()
            print e
            print "waiting2"
            continue
    quaternion = (
    trans4.transform.rotation.x,
    trans4.transform.rotation.y,
    trans4.transform.rotation.z,
    trans4.transform.rotation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    Rx = math.degrees(euler[0])
    Ry = math.degrees(euler[1])
    Rz = math.degrees(euler[2])
    print trans4.transform.translation.x
    print trans4.transform.translation.y
    print trans4.transform.translation.z
    print Rx
    print Ry
    print Rz

    from tm_motion.msg import ActionAction, ActionGoal
    print "tm moving out to place object on ld"
    try:
        goal = ActionGoal()
        goal.goal_goal1 = trans4.transform.translation.x
        goal.goal_goal2 = trans4.transform.translation.y
        goal.goal_goal3 = trans4.transform.translation.z
        goal.goal_goal4 = Rx
        goal.goal_goal5 = Ry
        goal.goal_goal6 = Rz
        # goal.goal_goal4 = 179.91
        # goal.goal_goal5 = -1.50
        # goal.goal_goal6 = -42.80
        result = call_server()
        print 'The result is:', result
        print "pick object"
    except rospy.ROSInterruptException as e:
        print 'Something went wrong:', e

    from tm_motion.msg import ActionAction, ActionGoal
    print "tm rotating to place object on ld"
    try:
        goal = ActionGoal()
        goal.goal_goal1 = 222.48
        goal.goal_goal2 = 223.86
        goal.goal_goal3 = 707.08
        goal.goal_goal4 = 178.39
        goal.goal_goal5 = -0.56
        goal.goal_goal6 = 43.48
        result = call_server()
        print 'The result is:', result
        print "pick object"
    except rospy.ROSInterruptException as e:
        print 'Something went wrong:', e

    from tm_motion.msg import ActionAction, ActionGoal
    print "tm align object on ld"
    try:
        goal = ActionGoal()
        goal.goal_goal1 = 288.33
        goal.goal_goal2 = 131.60
        goal.goal_goal3 = 332.66
        goal.goal_goal4 = 179.44
        goal.goal_goal5 = -0.15
        goal.goal_goal6 = 44.52
        result = call_server()
        print 'The result is:', result
        print "pick object"
    except rospy.ROSInterruptException as e:
        print 'Something went wrong:', e

    from tm_motion.msg import ActionAction, ActionGoal
    print "tm placing object on ld"
    try:
        goal = ActionGoal()
        goal.goal_goal1 = 287.79
        goal.goal_goal2 = 128.34
        goal.goal_goal3 = 300.88
        goal.goal_goal4 = 179.07
        goal.goal_goal5 = -0.26
        goal.goal_goal6 = 43.70
        result = call_server()
        print 'The result is:', result
        print "pick object"
    except rospy.ROSInterruptException as e:
        print 'Something went wrong:', e
