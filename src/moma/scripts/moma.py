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
    time.sleep(5)

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
    client.send_goal(goal, feedback_cb=feedback_cb)
    client.wait_for_result()
    result = client.get_result()
    return result

if __name__ == "__main__":
    rospy.init_node('moma')
    try:
        goal = ActionGoal()
        goal.goal_goal = "pickup"
        result = call_ld_server()
        print 'The result is:', result
        print "ld moved to pickup location"
    except rospy.ROSInterruptException as e:
        print 'Something went wrong:', e

    from tm_motion.msg import ActionAction, ActionGoal
    print "tm moving to pick location to scan tm landmark"
    try:
        goal = ActionGoal()
        # goal.goal_goal1 = float(values[0])
        # goal.goal_goal2 = float(values[1])
        # goal.goal_goal3 = float(values[2])
        # goal.goal_goal4 = float(values[3])
        # goal.goal_goal5 = float(values[4])
        # goal.goal_goal6 = float(values[5])
        goal.goal_goal1 = 431.48
        goal.goal_goal2 = 414.43
        goal.goal_goal3 = 349.58
        goal.goal_goal4 = -179.57
        goal.goal_goal5 = -0.38
        goal.goal_goal6 = 136.14
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
    landmark_to_obj.transform.translation.x = 100
    landmark_to_obj.transform.translation.y = 0
    landmark_to_obj.transform.translation.z = -300
    quat = tf.transformations.quaternion_from_euler(
               0,0,-math.pi)
    landmark_to_obj.transform.rotation.x = quat[0]
    landmark_to_obj.transform.rotation.y = quat[1]
    landmark_to_obj.transform.rotation.z = quat[2]
    landmark_to_obj.transform.rotation.w = quat[3]
    broadcaster2.sendTransform(landmark_to_obj)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('base_link', 'object_location', rospy.Time())
            print "new values:"
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

    start_program()
    time.sleep(1)
    release()
    stop_program()

    from tm_motion.msg import ActionAction, ActionGoal
    try:
        goal = ActionGoal()
        goal.goal_goal1 = trans.transform.translation.x
        goal.goal_goal2 = trans.transform.translation.y
        goal.goal_goal3 = trans.transform.translation.z +100
        goal.goal_goal4 = Rx
        goal.goal_goal5 = Ry
        goal.goal_goal6 = Rz
        result = call_server()
        print 'The result is:', result
        print "moved to top of pickup location"
    except rospy.ROSInterruptException as e:
        print 'Something went wrong:', e

    try:
        goal = ActionGoal()
        goal.goal_goal1 = trans.transform.translation.x
        goal.goal_goal2 = trans.transform.translation.y
        goal.goal_goal3 = trans.transform.translation.z
        goal.goal_goal4 = Rx
        goal.goal_goal5 = Ry
        goal.goal_goal6 = Rz
        result = call_server()
        print 'The result is:', result
        print "moved to pickup location"
    except rospy.ROSInterruptException as e:
        print 'Something went wrong:', e

    start_program()
    time.sleep(1)
    grip()
    print "gripped object"
    stop_program()

    try:
        goal = ActionGoal()
        goal.goal_goal1 = trans.transform.translation.x
        goal.goal_goal2 = trans.transform.translation.y
        goal.goal_goal3 = trans.transform.translation.z + 100
        goal.goal_goal4 = Rx
        goal.goal_goal5 = Ry
        goal.goal_goal6 = Rz
        result = call_server()
        print 'The result is:', result
    except rospy.ROSInterruptException as e:
        print 'Something went wrong:', e

    # print "tm moving to home postion"
    # try:
    #     goal = ActionGoal()
    #     goal.goal_goal1 = 191.08
    #     goal.goal_goal2 = 223.33
    #     goal.goal_goal3 = 400.54
    #     goal.goal_goal4 = -178.77
    #     goal.goal_goal5 = 1.46
    #     goal.goal_goal6 = 131.84
    #     result = call_server()
    #     print 'The result is:', result
    # except rospy.ROSInterruptException as e:
    #     print 'Something went wrong:', e

    print "tm moving to postion to place object on moma"
    try:
        goal = ActionGoal()
        goal.goal_goal1 = 64.25
        goal.goal_goal2 = 266.76
        goal.goal_goal3 = 266.57
        goal.goal_goal4 = -179.71
        goal.goal_goal5 = -0.90
        goal.goal_goal6 = 135.55
        result = call_server()
        print 'The result is:', result
    except rospy.ROSInterruptException as e:
        print 'Something went wrong:', e

    start_program()
    time.sleep(1)
    release()
    stop_program()

    print "tm moving to safe position"
    try:
        goal = ActionGoal()
        goal.goal_goal1 = 28.11
        goal.goal_goal2 = 268.09
        goal.goal_goal3 = 357.99
        goal.goal_goal4 = 114.94
        goal.goal_goal5 = 78.09
        goal.goal_goal6 = 92.66
        result = call_server()
        print 'The result is:', result
    except rospy.ROSInterruptException as e:
        print 'Something went wrong:', e

    from om_aiv_navigation.msg import ActionAction, ActionGoal
    try:
        goal = ActionGoal()
        goal.goal_goal = "dropoff"
        result = call_ld_server()
        print 'The result is:', result
        print "ld moved to dropoff location"
    except rospy.ROSInterruptException as e:
        print 'Something went wrong:', e

    from tm_motion.msg import ActionAction, ActionGoal
    print "TM moving to location to scan tm landmark for dropoff"
    try:
        goal = ActionGoal()
        # goal.goal_goal1 = float(values3[0])
        # goal.goal_goal2 = float(values3[1])
        # goal.goal_goal3 = float(values3[2])
        # goal.goal_goal4 = float(values3[3])
        # goal.goal_goal5 = float(values3[4])
        # goal.goal_goal6 = float(values3[5])
        goal.goal_goal1 = 343.51
        goal.goal_goal2 = 502.13
        goal.goal_goal3 = 580.12
        goal.goal_goal4 = 179.80
        goal.goal_goal5 = -2.56
        goal.goal_goal6 = 134.55
        result = call_server()
        print 'The result is:', result
    except rospy.ROSInterruptException as e:
        print 'Something went wrong:', e

    from tm_motion.msg import ActionAction, ActionGoal
    print "scanning for tm landmark location"
    print landmark_location_service_client()
    rate = rospy.Rate(10.0)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('base_link', 'landmark_location', rospy.Time())
            print trans.transform
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
    broadcaster2 = tf2_ros.StaticTransformBroadcaster()
    landmark_to_obj = geometry_msgs.msg.TransformStamped()
    landmark_to_obj.header.stamp = rospy.Time.now()
    landmark_to_obj.header.frame_id = "landmark_location"
    landmark_to_obj.child_frame_id = "place_location"
    landmark_to_obj.transform.translation.x = 100
    landmark_to_obj.transform.translation.y = 0
    landmark_to_obj.transform.translation.z = -280
    quat = tf.transformations.quaternion_from_euler(
               0,0,-math.pi)
    landmark_to_obj.transform.rotation.x = quat[0]
    landmark_to_obj.transform.rotation.y = quat[1]
    landmark_to_obj.transform.rotation.z = quat[2]
    landmark_to_obj.transform.rotation.w = quat[3]
    broadcaster2.sendTransform(landmark_to_obj)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('base_link', 'place_location', rospy.Time())
            print "place location:"
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

    try:
        goal = ActionGoal()
        goal.goal_goal1 = trans.transform.translation.x
        goal.goal_goal2 = trans.transform.translation.y
        goal.goal_goal3 = trans.transform.translation.z
        goal.goal_goal4 = Rx
        goal.goal_goal5 = Ry
        goal.goal_goal6 = Rz
        result = call_server()
        print 'The result is:', result
        print "moved to dropoff location"
    except rospy.ROSInterruptException as e:
        print 'Something went wrong:', e

    start_program()
    release()
    stop_program()

    print "tm moving to home postion"
    try:
        goal = ActionGoal()
        goal.goal_goal1 = 191.08
        goal.goal_goal2 = 223.33
        goal.goal_goal3 = 400.54
        goal.goal_goal4 = -178.77
        goal.goal_goal5 = 1.46
        goal.goal_goal6 = 131.84
        result = call_server()
        print 'The result is:', result
    except rospy.ROSInterruptException as e:
        print 'Something went wrong:', e
