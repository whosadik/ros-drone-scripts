#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

current_state = None

def state_cb(msg):
    global current_state
    current_state = msg

def main():
    rospy.init_node("fly_to_target_node")
    state_sub = rospy.Subscriber("/mavros/state", State, state_cb)
    local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rate = rospy.Rate(20)
    pose = PoseStamped()

    # Ожидаем подключения
    while not rospy.is_shutdown() and current_state is None:
        rospy.loginfo("Ожидаем подключения к FCU...")
        rate.sleep()

    rospy.loginfo("Подключено!")

    # Запрашиваем координаты цели у пользователя
    waypoints = []
    num_points = int(input("Сколько точек хотите задать?"))

    for i in range(num_points):
        x = float(input(f"Введите X для точки {i+1}: "))
        y = float(input(f"Введите Y для точки {i+1}: "))
        z = float(input(f"Введите Z для точки {i+1}: "))
        waypoints.append((x, y, z))

    


    # Установка целевой позиции
    pose.pose.position.x = waypoints[0][0]
    pose.pose.position.y = waypoints[0][1]
    pose.pose.position.z = waypoints[0][2]

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


    for i, point in enumerate(waypoints):
        rospy.loginfo("Летим в точку %d: x=%.2f y=%.2f z=%.2f", i+1, point[0], point[1], point[2])
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.position.z = point[2]
    # Держим цель 5 секунд
        for _ in range(500):
            pose.header.stamp = rospy.Time.now()
            local_pos_pub.publish(pose)
            rate.sleep()

    rospy.loginfo("Все точки пройдены. Миссия завершена.")

    rospy.loginfo("Начинаем посадку")

    landing_steps = int(waypoints[-1][2] * 10)

    start_z = pose.pose.position.z

    for step in range(landing_steps):
        pose.pose.position.z = start_z * (1.0 - step/landing_steps)
        pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(pose)
        rate.sleep()
    rospy.loginfo("Посадка завершена")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
