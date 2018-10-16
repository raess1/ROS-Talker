#!/usr/bin/env python
import rospy, time, math, random
from std_msgs.msg import Int32, Float32, Float64, String
from time import sleep


def talker():
    pubSpeed = rospy.Publisher('commands/motor/setspeed', Float64, queue_size=10)
    pubPos   = rospy.Publisher('commands/motor/position', Float64, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz    
    # going to home pos...
    rospy.loginfo("home pos...")
    last_pos = 0  # home pos    
    pubSpeed.publish(1000)
    pubPos.publish(last_pos)
    rospy.sleep(5)     
    degreePerRevolution = 3600  # one gearbox rotation in motor degree (0..3600)
    range = 180  # desired gearbox limit (0..360)    
    T = 5 # time for one sinus cycle (seconds)
    freq = 1/T                
    rangeMotor = degreePerRevolution/360*range
    last_time = rospy.get_time()
    
    while not rospy.is_shutdown():                
        now = rospy.get_time()                        
        set_pos = math.sin(2*math.pi* freq * now ) * rangeMotor/2 + rangeMotor/2                                     
        deltaPos = abs(set_pos - last_pos)        
        deltaTime = abs(now - last_time)
        #rpm = 5000
        rpm = (deltaPos/degreePerRevolution) / deltaTime * 60
        rospy.loginfo("pos %d speed %d" % (set_pos, rpm) )
        pubSpeed.publish(speed)
        pubPos.publish(set_pos/ 180.0 * math.pi)
        last_pos = set_pos
        last_time = now
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
 
        pass
