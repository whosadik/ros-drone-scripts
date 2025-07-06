#!/usr/bin/env python  

import rospy #импортироуем рос для работы 
from geometry_msgs.msg import PoseStamped #импорт для позиций и времени и др данных
from mavros_msgs.srv import CommandBool, SetMode #импорт дял управления мотором и настройки дрона
from mavros_msgs.msg import State #состояние дрона 

current_state = None #до запуска состояние 

def state_cb(msg): # стейт колбак для куррент стейта 
    global current_state
    current_state = msg

def main():
    rospy.init_node("fly_square_node") #инициализируем ros

    rospy.Subscriber("/mavros/state", State, state_cb) # обявляем сабскрайбера который будет получать состояние дрона 

    local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10) # публишер для того чтобы задать позицию и время а также очередь

    waypoints = [
        (0, 0, 2),
        (2, 0, 2),
        (2, 2, 2),
        (0, 2, 2),
        (0, 0, 2)
    ]

    pose = PoseStamped() #задаем переменное



    rate = rospy.Rate(20)#задаем 20 запросов 

    while not rospy.is_shutdown() and current_state is None: # пока наш код не приостановился и состояние None то логируем 
        rospy.loginfo("Ожидаем подключение...")
        rate.sleep()

    rospy.loginfo("Подключено!") # после выхода с цикла тоже лог

    pose.pose.position.x = waypoints[0][0]
    pose.pose.position.y = waypoints[0][1]
    pose.pose.position.z = waypoints[0][2]

    for _ in range(100): #запускаем цикл 100 раза отправляем запрос 
        pose.header.stamp = rospy.Time.now() #задаем данное время
        local_pos_pub.publish(pose) #и позицию
        rate.sleep()
    
    rospy.wait_for_service('/mavros/cmd/arming')# ждем подключения сервисов 
    rospy.wait_for_service('/mavros/set_mode')

    arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool) #создаем для управлением мотора и мода
    set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

    resp1 = set_mode_client(custom_mode = "OFFBOARD")
    rospy.loginfo("Ответ на установку режима: %s", resp1)


    resp2 = arming_client(True)
    rospy.loginfo("Ответ на арминг: %s", resp2)

    rospy.loginfo("Начнем летать по квадрату уууу")

    for i, point in enumerate(waypoints):
        rospy.loginfo("Летим в точку %d: x=%.1f y=%.1f z=%.1f", i+1, point[0], point[1], point[2])

        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.position.z = point[2]

        for _ in range(100):
            pose.header.stamp = rospy.Time.now()
            local_pos_pub.publish(pose)
            rate.sleep()

    rospy.loginfo("Миссия завершена")

    rospy.loginfo("Начинаем посадку")

    landing_steps = 40

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

    

    
            
    

