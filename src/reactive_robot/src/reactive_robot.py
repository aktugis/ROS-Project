#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard
import time

# Hız komutunu saklamak için global değişken
cmd = Twist()

# Başlangıç zamanı
start_time = time.time()

# Robotun hareket etme süresi (60 saniye)
TIME_LIMIT = 60

# Dönme hızı için bir değişken
angular_speed = 0.5

# Zaman limiti kontrol bayrağı
time_limit_reached = False

# Tuşa basıldığında yapılan işlemler
def on_press(key):
    global cmd, start_time, angular_speed, time_limit_reached
    if time_limit_reached:  # Zaman limiti dolduktan sonra işlem yapma
        return
    
    try:
        if key.char == 'w':  # İleri hareket
            cmd.linear.x = 0.2
            cmd.angular.z = 0.0
            rospy.loginfo("İleri hareket başladı")
        elif key.char == 's':  # Geri hareket
            cmd.linear.x = -0.2
            cmd.angular.z = 0.0
            rospy.loginfo("Geri hareket başladı")
        elif key.char == 'a':  # Sola dönüş
            cmd.linear.x = 0.0
            cmd.angular.z = angular_speed
            rospy.loginfo("Sola dönme başladı")
        elif key.char == 'd':  # Sağa dönüş
            cmd.linear.x = 0.0
            cmd.angular.z = -angular_speed
            rospy.loginfo("Sağa dönme başladı")
    except AttributeError:
        pass

# Tuş serbest bırakıldığında yapılan işlemler
def on_release(key):
    global cmd, time_limit_reached
    if time_limit_reached:  # Zaman limiti dolduktan sonra işlem yapma
        return

    # Robot hareketini durdur
    cmd.linear.x = 0.0
    cmd.angular.z = 0.0
    rospy.loginfo("Hareket durduruldu")
    if key == keyboard.Key.esc:
        return False  # 'esc' tuşuna basıldığında dinleyiciyi durdur

def main():
    global start_time, time_limit_reached

    # ROS düğümünü başlat
    rospy.init_node('reactive_robot_node', anonymous=True)

    # Publisher oluştur
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Yayın hızı
    rate = rospy.Rate(10)

    # Klavye dinleyicisini başlat
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        rospy.loginfo("Robot hareketi başlatıldı. 'w', 'a', 's', 'd' tuşlarıyla kontrol edebilirsiniz.")

        while not rospy.is_shutdown():
            # Geçen süreyi kontrol et
            elapsed_time = time.time() - start_time
            if elapsed_time >= TIME_LIMIT:
                rospy.loginfo("Zaman limiti doldu. Hareket durduruluyor...")
                time_limit_reached = True  # Zaman limiti bayrağını ayarla
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                pub.publish(cmd)  # Hareket komutunu sıfırla
                break

            # Mevcut hız komutunu yayınla
            pub.publish(cmd)
            rate.sleep()

        listener.join()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

