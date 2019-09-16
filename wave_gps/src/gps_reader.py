#! /usr/bin/python
# Written by Dan Mandle http://dan.mandle.me September 2012
# License: GPL 2.0 
import os
from gps import *
from time import *
import time
import threading
import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus


gpsd = None #seting the global variable
navsat_fix_pub = rospy.Publisher('fix', NavSatFix, queue_size=10)
rospy.init_node('gps_node', anonymous=True)
os.system('clear') #clear the terminal (optional)

class GpsPoller(threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self)
    global gpsd #bring it in scope
    global navsat_fix_pub
    gpsd = gps(mode=WATCH_ENABLE) #starting the stream of info 
   
    self.current_value = None
    self.running = True #setting the thread running to true

  def run(self):
    global gpsd
    global navsat_fix_pub
    while gpsp.running:
      gpsd.next() #this will continue to loop and grab EACH set of gpsd info to clear the buffer

if __name__ == '__main__':
  gpsp = GpsPoller() # create the thread
  try:
    gpsp.start() # start it up
    r = rospy.Rate(2)
    while not rospy.is_shutdown():
      p = gpsd
      os.system('clear')
      fix = NavSatFix()
      fix.header.stamp = rospy.Time.now();
      fix.status.status = p.status
      fix.status.service = p.fix.mode
      fix.latitude = p.fix.latitude;
      fix.longitude = p.fix.longitude;
      fix.altitude = p.fix.altitude;
      fix.position_covariance[0] = p.fix.epx;
      fix.position_covariance[4] = p.fix.epy;
      fix.position_covariance[8] = p.fix.epv;
      navsat_fix_pub.publish(fix);
      print dir(gpsd)
      print dir(gpsd.fix)  
      print
      print 'vdop        ' , gpsd.vdop
      print 'hdop        ' , gpsd.hdop
      print 'xdop        ' , gpsd.xdop
      print 'ydop        ' , gpsd.ydop
      print 'status      ' , gpsd.status
      print 'epe         ' , gpsd.epe
      print 'latitude    ' , gpsd.fix.latitude
      print 'longitude   ' , gpsd.fix.longitude
      print 'time utc    ' , gpsd.utc,' + ', gpsd.fix.time
      print 'altitude (m)' , gpsd.fix.altitude
      print 'eps         ' , gpsd.fix.eps
      print 'epx         ' , gpsd.fix.epx
      print 'epv         ' , gpsd.fix.epv
      print 'ept         ' , gpsd.fix.ept
      print 'speed (m/s) ' , gpsd.fix.speed
      print 'climb       ' , gpsd.fix.climb
      print 'track       ' , gpsd.fix.track
      print 'mode        ' , gpsd.fix.mode
      print
      r.sleep()

  except (KeyboardInterrupt, SystemExit): #when you press ctrl+c
    print "\nKilling Thread..."
    gpsp.running = False
    gpsp.join() # wait for the thread to finish what it's doing
  print "Done.\nExiting."
