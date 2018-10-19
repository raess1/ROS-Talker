#!/usr/bin/env python
#code by Alexander Grau http://grauonline.de/wordpress/
ros = True

import time, math, random
from time import sleep
import matplotlib.pyplot as plt
import numpy as np
if ros:
  import rospy
  from std_msgs.msg import Int32, Float32, Float64, String


T = 0.1                    # time for one sinus cycle (seconds)
maxRPM = 8000                # max motor RPM
homePos = 0                 # motor home pos
homeSpeed = 1000            # motor homing speed
degreePerRevolution = 2160   # one gearbox rotation in motor degree (0..3600)
range = 180                  # desired gearbox limit (0..360)    
currPos = 0


def initPlot():
  global y1, y2, y3, fig, line1, line2, line3
  x = np.linspace(0, 100, 100)
  y1 = np.linspace(0, degreePerRevolution, 100)
  y2 = np.linspace(0, degreePerRevolution, 100)
  y3 = np.linspace(0, maxRPM, 100)  

  plt.ion()

  fig = plt.figure()
  ax1 = fig.add_subplot(111)
  ax1.set_xlabel('time (s)')
  ax1.set_ylabel('setpos', color='r')
  ax2 = ax1.twinx()
  ax2.set_ylabel('currpos', color='g')
  ax3 = ax2.twinx()
  ax3.set_ylabel('rpm', color='b')

  line1, = ax1.plot(x, y1, 'r-') # Returns a tuple of line objects, thus the comma
  line2, = ax2.plot(x, y2, 'g-') # Returns a tuple of line objects, thus the comma
  line3, = ax3.plot(x, y3, 'b-') # Returns a tuple of line objects, thus the comma
  ax1.grid()
  fig.canvas.draw()
  fig.canvas.flush_events()


def plot(v1, v2, v3):  
  global y1, y2, y3, fig, line1, line2, line3
  y1 = y1[1:]
  y1 = np.append(y1, v1) 
  line1.set_ydata(y1)                  
  
  y2 = y2[1:]
  y2 = np.append(y2, v2) 
  line2.set_ydata(y2)                  
  
  y3 = y3[1:]
  y3 = np.append(y3, v3) 
  line3.set_ydata(y3)
                                                     
  fig.canvas.draw()
  fig.canvas.flush_events()
    

def posCallback(data):
   global currPos
   currPos = data.data/math.pi*180.0


def talker():        
    if ros:
      rospy.Subscriber("/sensors/rotor_position", Float32, posCallback)
      pubSpeed = rospy.Publisher('/commands/motor/setspeed', Float64, queue_size=10)
      pubPos   = rospy.Publisher('/commands/motor/position', Float64, queue_size=10)
      rospy.init_node('talker', anonymous=True)
      rate = rospy.Rate(10) # 10hz    
      # going to home pos...
      rospy.loginfo("home pos...")   
      pubSpeed.publish(homeSpeed)
      pubPos.publish(homePos/ 180.0 * math.pi)
      rospy.sleep(5)         
      lastTime = rospy.get_time()
    else:
      lastTime = time.time()
    freq = 1/T               
    rangeMotor = degreePerRevolution/360*range
    lastPos = homePos
    nextPlotTime = 0
    
    while True:      
      if ros:
        if rospy.is_shutdown(): break                    
        now = rospy.get_time()                                
      else:        
        now = time.time()                
        
      setPos = math.sin(10*math.pi* freq * now ) * rangeMotor/2 + rangeMotor/2                                                   
      #if not ros: currPos = lastPos
      #global currPos
      #print(str(currPos))
      #print(str(setPos))
      
      deltaPos = abs(setPos - currPos)
      #print(str(deltaPos))                
      deltaTime = max(0.0001, abs(now - lastTime))                
      rpm = 1000        
      #rpm = min(maxRPM, (deltaPos/360.0) / deltaTime * 60)
      #print(str(rpm))                
      print(str(deltaPos) + ", " + str(rpm)) 
	     
      
      if ros:
        #rospy.loginfo("pos %d speed %d" % (setPos, rpm) )
        pubSpeed.publish(rpm)
        pubPos.publish(setPos/ 180.0 * math.pi)      
      
      if time.time() > nextPlotTime:
        nextPlotTime = time.time() + 0.02
        plot(setPos, currPos, rpm)        
      
      lastPos = setPos
      lastTime = now
      if ros:      
        rate.sleep()
      else:      
        time.sleep(0.1)
      

if __name__ == '__main__':
    initPlot()
    if ros:    
      try:    
        talker()
      except rospy.ROSInterruptException:    
        pass
    else:      
        talker()
