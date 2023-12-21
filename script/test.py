#!/usr/bin/python
# license removed for brevity
import sys
import rospy
from std_msgs.msg import String
import argparse

def talker(fullscreen_mode):
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    if fullscreen_mode == True:
    	print "I get it"
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == "__main__":
	#if len(sys.argv)) > 
	#for arg in sys.argv:
	#	print arg
	#print sys.argv[1] --> return True
	
	arg_fmt = argparse.RawDescriptionHelpFormatter
	parser = argparse.ArgumentParser(formatter_class=arg_fmt)
	parser.add_argument('--fullscreen', dest='fullscreen_mode', action='store_true')
	parser.add_argument('--no-fullscreen', dest='fullscreen_mode', action='store_false')   
	parser.set_defaults(fullscreen_mode=True)
	
	args = parser.parse_args(rospy.myargv()[1:])
	mode = args.fullscreen_mode 
	print mode
	
	try:
		talker(mode)
	except rospy.ROSInterruptException:
		pass

                                                                                   
