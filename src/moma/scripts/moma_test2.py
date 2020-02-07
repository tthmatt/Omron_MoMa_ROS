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
    client.send_goal(goal, feedback_cb=feedback_cb)
    client.wait_for_result(rospy.Duration(0))
    result = client.get_result()
    return result

if __name__ == "__main__":
    rospy.init_node('moma')
    print "REMEMBER TO PUT TM ARM IN SAFE POSITION"

    try:
        goal = ActionGoal()
        goal.goal_goal = "pickup"
        result = call_ld_server()
        print 'The result is:', result
        print "ld moved to pickup location"
    except rospy.ROSInterruptException as e:
        exit(0)
        print 'Something went wrong:', e

    start_program()
    from tm_motion.msg import ActionAction, ActionGoal
    print "tm moving to pick location to scan tm landmark"
    try:
        goal = ActionGoal()
        goal.goal_goal1 = 373.52
        goal.goal_goal2 = -165
        goal.goal_goal3 = 683.37
        goal.goal_goal4 = -178.86
        goal.goal_goal5 = 0.57
        goal.goal_goal6 = 47.18
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
    #input offset here (found from moma_find_offset.py program)
    landmark_to_obj.transform.translation.x = 12.7157474008
    landmark_to_obj.transform.translation.y = 220.10747054
    landmark_to_obj.transform.translation.z = -211.602548852
    # quat = tf.transformations.quaternion_from_euler(
    #            42.0302727731,6.08534532397,-122.612682455)
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
        #input safe position coordinates that arm can rotate
        goal.goal_goal1 = 370.24
        goal.goal_goal2 = 215.52
        goal.goal_goal3 = 589.34
        goal.goal_goal4 = -179.23
        goal.goal_goal5 = 1.45
        goal.goal_goal6 = 47.21
        result = call_server()
        print 'The result is:', result
    except rospy.ROSInterruptException as e:
        print 'Something went wrong:', e

    from tm_motion.msg import ActionAction, ActionGoal
    print "tm rotating arm"
    try:
        goal = ActionGoal()
        goal.goal_goal1 = 367.99
        goal.goal_goal2 = 211.02
        goal.goal_goal3 = 593.15
        goal.goal_goal4 = -179.22
        goal.goal_goal5 = -1.09
        goal.goal_goal6 = 154.50
        result = call_server()
        print 'The result is:', result
    except rospy.ROSInterruptException as e:
        print 'Something went wrong:', e

    from tm_motion.msg import ActionAction, ActionGoal
    print "tm moving to pick object"
    try:
        goal = ActionGoal()
        goal.goal_goal1 = 157.71
        goal.goal_goal2 = 48.50
        goal.goal_goal3 = 652.12
        goal.goal_goal4 = 179.49
        goal.goal_goal5 = -1.41
        goal.goal_goal6 = -130.75
        result = call_server()
        print 'The result is:', result
    except rospy.ROSInterruptException as e:
        print 'Something went wrong:', e

    from tm_motion.msg import ActionAction, ActionGoal
    print "tm alligning to pick object using offset and transformations"
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
    obj1_to_obj2.transform.translation.y = -200 #move in to pick offset
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
    obj2_to_obj3.transform.translation.y = -200
    obj2_to_obj3.transform.translation.z = -90
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
    obj2_to_obj3.transform.translation.y = 150
    obj2_to_obj3.transform.translation.z = -90
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

    from om_aiv_navigation.msg import ActionAction, ActionGoal
    try:
        goal = ActionGoal()
        goal.goal_goal = "dropoff"
        result = call_ld_server()
        print 'The move to dropoff status is:', result
        print "ld moved to dropff location"
    except rospy.ROSInterruptException as e:
        exit(0)
        print 'Something went wrong:', e



    from tm_motion.msg import ActionAction, ActionGoal
    print "tm lifting up object on ld"
    try:
        goal = ActionGoal()
        goal.goal_goal1 = 286.13
        goal.goal_goal2 = 141.24
        goal.goal_goal3 = 712.49
        goal.goal_goal4 = -179.87
        goal.goal_goal5 = -0.12
        goal.goal_goal6 = 48.47
        result = call_server()
        print 'The result is:', result
        print "tm lifted up object on ld"
    except rospy.ROSInterruptException as e:
        print 'Something went wrong:', e

    from tm_motion.msg import ActionAction, ActionGoal
    print "tm lifting up object more"
    try:
        goal = ActionGoal()
        goal.goal_goal1 = 312.68
        goal.goal_goal2 = -175.57
        goal.goal_goal3 = 644.44
        goal.goal_goal4 = -178.21
        goal.goal_goal5 = -0.24
        goal.goal_goal6 = -44.48
        result = call_server()
        print 'The result is:', result
        print "lifted object"
    except rospy.ROSInterruptException as e:
        print 'Something went wrong:', e

    from tm_motion.msg import ActionAction, ActionGoal
    print "tm moving to scan landmark"
    try:
        goal = ActionGoal()
        goal.goal_goal1 = 576.27
        goal.goal_goal2 = -241.48
        goal.goal_goal3 = 613.98
        goal.goal_goal4 = -179.85
        goal.goal_goal5 = -1.43
        goal.goal_goal6 = -42.93
        result = call_server()
        print 'The result is:', result
        print "moved to postion to scan landmark"
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
            print "landmark_location wrt base_link at dropoff"
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
    #input offset here (found from moma_find_offset.py program)
    landmark_to_obj.transform.translation.x = -46.5926566695
    landmark_to_obj.transform.translation.y = -76.3642348521
    landmark_to_obj.transform.translation.z = -452.65707614
    quat = tf.transformations.quaternion_from_euler(
               0,0,0)
    # quat = tf.transformations.quaternion_from_euler(
    #            162.885145622,40.0967567944,174.423639365)
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

    from tm_motion.msg import ActionAction, ActionGoal
    print "tm alligning to place object using offset and transformations"
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
        print "moved to position to allign to place object"
    except rospy.ROSInterruptException as e:
        print 'Something went wrong:', e

    roadcaster2 = tf2_ros.StaticTransformBroadcaster()
    obj1_to_obj2 = geometry_msgs.msg.TransformStamped()
    obj1_to_obj2.header.stamp = rospy.Time.now()
    obj1_to_obj2.header.frame_id = "object_location"
    obj1_to_obj2.child_frame_id = "object_location2"
    obj1_to_obj2.transform.translation.x = 0
    obj1_to_obj2.transform.translation.y = 0
    obj1_to_obj2.transform.translation.z = 140
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
    print "tm moving down to place object"
    try:
        goal = ActionGoal()
        goal.goal_goal1 = trans2.transform.translation.x
        goal.goal_goal2 = trans2.transform.translation.y
        goal.goal_goal3 = trans2.transform.translation.z
        goal.goal_goal4 = Rx
        goal.goal_goal5 = Ry
        goal.goal_goal6 = Rz
        result = call_server()
        print 'The result is:', result
        print "moved to position to place object"
    except rospy.ROSInterruptException as e:
        print 'Something went wrong:', e

    #tm retract arm
    broadcaster3 = tf2_ros.StaticTransformBroadcaster()
    obj2_to_obj3 = geometry_msgs.msg.TransformStamped()
    obj2_to_obj3.header.stamp = rospy.Time.now()
    obj2_to_obj3.header.frame_id = "object_location"
    obj2_to_obj3.child_frame_id = "object_location3"
    obj2_to_obj3.transform.translation.x = 0
    obj2_to_obj3.transform.translation.y = 300
    obj2_to_obj3.transform.translation.z = 140
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
    print "tm retract arm from object"
    try:
        goal = ActionGoal()
        goal.goal_goal1 = trans3.transform.translation.x
        goal.goal_goal2 = trans3.transform.translation.y
        goal.goal_goal3 = trans3.transform.translation.z
        goal.goal_goal4 = Rx
        goal.goal_goal5 = Ry
        goal.goal_goal6 = Rz
        result = call_server()
        print 'The result is:', result
        print "retracted arm"
    except rospy.ROSInterruptException as e:
        print 'Something went wrong:', e

    from tm_motion.msg import ActionAction, ActionGoal
    print "tm rotating to home position"
    try:
        goal = ActionGoal()
        goal.goal_goal1 = 385.87
        goal.goal_goal2 = 94.79
        goal.goal_goal3 = 601.79
        goal.goal_goal4 = -174.87
        goal.goal_goal5 = -1.66
        goal.goal_goal6 = -47.14
        result = call_server()
        print 'The result is:', result
        print "tm rotated"
    except rospy.ROSInterruptException as e:
        print 'Something went wrong:', e

    from tm_motion.msg import ActionAction, ActionGoal
    print "tm moving to home position"
    try:
        goal = ActionGoal()
        goal.goal_goal1 = 233.64
        goal.goal_goal2 = 262.68
        goal.goal_goal3 = 429.64
        goal.goal_goal4 = 179.07
        goal.goal_goal5 = -2.51
        goal.goal_goal6 = 44.91
        result = call_server()
        print 'The result is:', result
        print "tm homed"
    except rospy.ROSInterruptException as e:
        print 'Something went wrong:', e
