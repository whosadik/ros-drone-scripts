#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

current_state = None
target_pose = None

def state_cb(msg):
    global current_state
    current_state = msg

def target_cb(msg):
    global target_pose
    target_pose = msg

def main():
    rospy.init_node("follow_to_target_node")
    state_sub = rospy.Subscriber("/mavros/state", State, state_cb)
    rospy.Subscriber("/target_position", PoseStamped, target_cb)
    
    local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rate = rospy.Rate(20)
    pose = PoseStamped()

    # Ожидаем подключения
    while not rospy.is_shutdown() and current_state is None:
        rospy.loginfo("Ожидаем подключения к FCU...")
        rate.sleep()

    rospy.loginfo("Подключено!")

    while not rospy.is_shutdown() and target_pose is None:
        rospy.loginfo("Ожидаем получения цели от /target_position...")
        rate.sleep()
    


    # Отправляем 100 setpoint перед OFFBOARD
    for _ in range(100):
        pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(pose)
        rate.sleep()

    # Подключение к сервисам
    rospy.wait_for_service('/mavros/cmd/arming')
    rospy.wait_for_service('/mavros/set_mode')
    arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

    set_mode_client(custom_mode="OFFBOARD")
    arming_client(True)

    rospy.loginfo("Начинаем следовать за целью...")

    while not rospy.is_shutdown():
        if target_pose:
            pose.pose.position = target_pose.pose.position
            pose.header.stamp = rospy.Time.now()
            local_pos_pub.publish(pose)
        rate.sleep()
        

    rospy.loginfo("Начинаем посадку")



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
