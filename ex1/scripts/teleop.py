#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty
"""
def on_press(key):
    print('Hey!')
    talker(key)

def on_release(key):
    print('{0} released'.format(key))
    if key == keyboard.Key.esc:
        # Stop listener
        return False


def talker(choice):
    pub = rospy.Publisher('chatter', Twist)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        twist = Twist()
        if choice == keyboard.Key.down:
        	twist.linear.y = -1;
    	elif choice == keyboard.Key.left:
        	twist.linear.x = -1;
        twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
	# Collect events until released
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
		listener.join()
"""
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def talker(choice):
        twist = Twist()
        if choice == 'a':
        	twist.linear.y = -1;
    	
        twist.linear.x = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        return twist
        
if __name__ == '__main__':
	settings = termios.tcgetattr(sys.stdin)
	pub = rospy.Publisher('/cmd_vel', Twist)
	rospy.init_node('talker', anonymous=True)
	while(1):
		if getKey() == 'q':
			print(getKey()) 
			break
		elif getKey() == 'a':
			pub.publish(talker('a'))
