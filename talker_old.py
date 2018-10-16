#!/usr/bin/env python
import rospy, time, math, random
from std_msgs.msg import Int32, Float32, Float64, String
from time import sleep


def talker():
    pubSpeed = rospy.Publisher('commands/motor/setspeed', Float64, queue_size=10)
    pubPos   = rospy.Publisher('commands/motor/position', Float64, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():                
        now = rospy.get_time()        
        freq = 1/5
        range = 1800
        set_pos = math.sin(2*math.pi* freq * now ) * range/2 + range/2                                     
        speed = 5000
        rospy.loginfo("pos %d speed %d" % (set_pos, speed) )
        pubSpeed.publish(speed)
        pubPos.publish(set_pos / 180.0 * math.pi)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
