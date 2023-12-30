#!/usr/bin/env python2


from pymycobot import MyCobot

import rospy
from std_msgs.msg import Bool, Int16
import threading

import RPi.GPIO as GPIO

import time
import ast


# 
awake = 1
#
# 0 -> spi
# 1 -> dziala
# 2 -> usypianie
# 3 -> ponowna inicjalizacja
# 4 -> gotowy po przebudzeniu




#data.data == 1 => start move and wait for the end of process
def callback(data, (mc, trigger)):
    global awake
    
    if(data.data):
        #print('Receiver data = ', data.data)
        
        if data.data == 1:
            mc.set_color(0,0,200)
            if awake == 4:
                awake = 1
            trigger.set()
        elif data.data == 2:
            #print('Asleep')
            if awake != 0:
                mc.set_color(200,0,200)
                awake = 2 # uspienie
                trigger.set()
        elif data.data == 3:
            #print('Awake')
            if awake == 0:
                mc.set_color(200,200,200)
                awake = 3 # ponowna inicjalizacja
                trigger.set()



def set_pos(angle, pwm):
    duty = angle / 18 + 2
    pwm.ChangeDutyCycle(duty)
    
    





def main():
    global awake
    
    # Robot initialization
    mc = MyCobot('/dev/ttyAMA0', 1000000)

    if not mc.is_power_on():
        mc.power_on()
        print('Robot is turned on')
    print('')
    mc.set_color(200,200,200)
    
    
    
    # Settings initialization
    with open("/home/pi/Desktop/MyCobot_launch/coords.txt", "r") as file:
        lines = file.read().splitlines()    
    
    data_from_file = []
 
    for line in lines:
        name, list_str = line.split("=")
        list_value = ast.literal_eval(list_str)
        if isinstance(list_value, list):
            data_from_file.append(list_value)
    
    

    speed = 30
    mode = 1 # 1-linear, 0-angular
    
    
    # gripper
    servo_pin = 3
    g_open = 0
    g_close = 60
    g_fullclose = 120
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(servo_pin, GPIO.OUT)

    pwm = GPIO.PWM(servo_pin, 50)
    pwm.start(0)



#    upper_ball_pos_c = [4.0, 246.3, 169.8, -97.65, 54.39, 83.86]
#    get_ball_pos_c = [4.1, 211.8, 36.7, -105.98, 67.18, 76.55]
#    unbend_pos_c = [17.7, 60.2, 375.7, -99.2, 43.56, 79.39]
#    leave_ball_pos_c = [112.4, 60.9, 360.1, -96.1, 25.58, 27.71]
    
    ang = mc.get_angles()
    start_pos_a = [90.61, 19.59, -88.59, -20.12, 93.69, ang[5]]
    sleep_pos_a = [90.26, -67.5, -104.85, -2.81, 9.84, 106.78]
    
    upper_ball_pos_c = data_from_file[0]
    get_ball_pos_c = data_from_file[1]
    unbend_pos_c = data_from_file[2]
    leave_ball_pos_c = data_from_file[3]
    
    
    #Position initialization 
    time.sleep(1)
    mc.sync_send_angles(start_pos_a, int(speed*0.75))
    set_pos(g_open, pwm)
    time.sleep(1)
    set_pos(g_fullclose, pwm)
    time.sleep(1)
    pwm.ChangeDutyCycle(0)
    
    
    

    # Ros initialization
    rospy.init_node('robot_node')
    trigger = threading.Event()
    rospy.Subscriber('move_enable', Int16, callback, (mc, trigger))
    mc.set_color(0,200,0)
    
    
    
    while not rospy.is_shutdown():
        trigger.wait()
        print('trigger.wait() end, awake -> ', awake)

        
        if awake == 1:
            
            #######################################################
            mc.sync_send_coords(upper_ball_pos_c, speed, mode, timeout=1.5)
            set_pos(g_open, pwm)
            time.sleep(1)
            mc.sync_send_coords(get_ball_pos_c, int(speed*0.5), mode, timeout=1.5)
            set_pos(g_close, pwm)
            time.sleep(1)
            mc.sync_send_coords(unbend_pos_c, speed, mode, timeout=1.5)
            time.sleep(1)
            mc.sync_send_coords(leave_ball_pos_c, speed, mode, timeout=1.5)
            set_pos(g_open, pwm)
            time.sleep(2)
            mc.sync_send_coords(unbend_pos_c, speed, mode, timeout=1.5)
            time.sleep(1)
            set_pos(g_fullclose, pwm)
            time.sleep(1)
            pwm.ChangeDutyCycle(0)
            #######################################################
            
            print('WORK TIME')
            time.sleep(1)
            
            mc.set_color(0,200,0)
            trigger.clear()
    
    
        elif awake == 2:
            set_pos(g_fullclose, pwm)
            time.sleep(1)
            pwm.ChangeDutyCycle(0)
            mc.sync_send_angles(sleep_pos_a, int(speed*0.5))
            time.sleep(3)
            mc.set_color(200,0,0)
            mc.release_all_servos()
            mc.power_off()
            print('Robot is turned off')
            time.sleep(0.5)
            awake = 0 # spi
            trigger.clear()
            
        elif awake == 0:
            mc.set_color(200,0,0)
            trigger.clear()
            
            
        elif awake == 3:
            print('Robot is turned on')
            mc.power_on()
            mc.set_color(200,200,200)
            time.sleep(1)
            mc.sync_send_angles(start_pos_a, int(speed*0.5))
            time.sleep(1)
            set_pos(g_fullclose, pwm)
            time.sleep(1)
            pwm.ChangeDutyCycle(0)
            mc.set_color(0,200,0)
            time.sleep(0.5)
            awake = 4 # gotowy do pracy
            trigger.clear()




if __name__ == '__main__':
    main()




