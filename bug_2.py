#!/usr/bin/env python3

# ROS stuff
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
# other useful math tools
from tf.transformations import euler_from_quaternion
from math import atan2, sqrt, pi
import math

 
# angle and distant difference constraints
# you can adjust the values for better performance
angle_eps = 0.03
dis_eps = 0.05

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

pub = None

# Class that will be used to read and parse /odom topic
class odomReader:

    def __init__(self):

        # subscribing to "/odom" topic
        # function newOdom() will take care of the new incoming message  
        sub = rospy.Subscriber("/odom", Odometry, self.newOdom)
        self.x = None
        self.y = None
        self.theta = None

    # Function that will take care of input message from odom topic
    # This function will be called whenever new message is available to read
    # Subsequently odom topic is parsed to get (x,y,theta) coordinates 
    def newOdom(self, msg):
        # get x and y coordinates
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # convert quaternion to Euler angles
        rot_q = msg.pose.pose.orientation
        (self.roll, self.pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        if self.theta < 0:
            self.theta += 2*pi

# Class that is responsible to read and parse raw LaserScan data 
class scanReader:

    def __init__(self):
        # subscribing to "/odom" topic
        # function newOdom() will take care of the new incoming message  
        sub = rospy.Subscriber("/scan", LaserScan, self.newScan)
        # divide laser scan data into 5 regions
        self.region = {
            'right': 0,
            'fright': 0,
            'front': 0,
            'fleft': 0,
            'left': 0,
        }
    # Function that will take care of input message from scan topic
    # This function will be called whenever new message is available to read
    # Subsequently scan topic is parsed to get region data: minimum distance to object from every sight 
    def newScan(self, msg):
        self.ranges = msg.ranges
        self.msg = msg
        self.region['left'] = min(self.ranges[60:100])
        self.region['fleft'] = min(self.ranges[20:60])
        self.region['front'] = min(self.ranges[0:20]+self.ranges[-20:])
        self.region['fright'] = min(self.ranges[300:340])
        self.region['right'] = min(self.ranges[260:300])
        
        #print "range[90]: ", msg.ranges[90]


# divide robot motion in 2 scenario
state_dict = {
    0: 'go to goal',
    1: 'circumnavigate obstacle',
}
# define initial scenario
state = 0


def main():
    global pub
    global state

    # initialize ROS node
    rospy.init_node("bug_2")
    # run stop function when this node is killed
    rospy.on_shutdown(stop)
    rospy.sleep(0.5)

    # define the control velocity publisher of topic type Twist
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

    # initialize odom and scan objects
    # Use these objects to access robot position in space and scan information
    odom = odomReader()
    scan = scanReader()
    rospy.sleep(0.5)

    # initialize speed as Twist topic 
    speed = Twist()

    # set the loop frequency
    rate = rospy.Rate(500)

    # Set the goal point 
    goal = Point()
    goal.x = 2.0 #-3.0
    goal.y = 4.0 #-1.0

    # Set the initial point
    init = Point()
    init.x = 0.0
    init.y = 0.0

    # slope of the m-line(line btw the initial and goal point)
    m_line = atan2((goal.y-init.y),(goal.x-init.x))
    m_line1 = atan2((goal.y-init.y),(goal.x-init.x))
    if (m_line1<0):
        m_line1+=2*pi
    

    # Variable that stores the coordinate of hit point when you 
    # encounter obstacle for the first time
    hit_point = Point()

    count = 0
    hit_count = 0
    c=0
    p=0
    s=0
    d=0.4
    state=0
    q=0
    r=0
    pa=0
    pb=0
    pc=0
    pd=0

    while not rospy.is_shutdown():
        # TODO:

        '''c=0
        p=0
        s=0
        d=0.4'''
        

        # Decide what to do for the robot in each of these states:

        # the x,y distance to the goal from current position
        inc_x = goal.x - odom.x
        inc_y = goal.y - odom.y

        # the angle of the goal point wrt global frame
        angle_to_goal = atan2(inc_y, inc_x)
        if (angle_to_goal<0):
            angle_to_goal+=2*pi

        # the distance to the goal from current location
        dist_diff = sqrt(inc_x**2 + inc_y**2)

        # find the heading angle difference
        angle_diff = angle_to_goal - odom.theta


        beta = atan2((goal.y-odom.y),(goal.x-odom.x))
        if (beta<0):
            beta+=2*pi
        zeta = (beta-m_line1)

        if state == 0:
            # go to goal state. 
            '''
            Hint: 
                Here robot should go towards a the goal unless it encounters an obstacle.
                When it encounters the wall it should change the state to 
                "circumnavigate obstacle".

                It's an updated version of the "go_to_point.py"
            '''
            # TODO:

            #if(c==0):
            pa=0
            pb=0
            pc=0
            pd=0

            if (angle_diff>0.1):

                speed.linear.x=0.0 # stopping linear movement
                speed.angular.z=0.3 # rotating anti-clockwise

            elif (angle_diff<-0.1):

                speed.linear.x=0.0 # stopping linear movement
                speed.angular.z=-0.3 # rotating clockwise
                    
            # Move towards wall if distance from wall is more than 0.3
            elif((dist_diff>=0.2) and scan.region['front'] > 0.3):

                speed.linear.x=0.11 # setting linear speed
                speed.angular.z=0.0 # stopping rotation

            # Stop moving towards goal if distance between wall and 
            # turtlebot is less than or equal to 0.3
            elif ((dist_diff>=0.2) and scan.region['front'] <= 0.3):

                speed.linear.x = 0.0 # stopping linear movement
                speed.angular.z = 0.0 # stopping rotation

                # checking if turtlebot is a bit away from hit point
                if (sqrt((odom.x-hit_point.x)**2+((odom.y-hit_point.y)**2))>0.2):
                    hit_point.x=odom.x
                    hit_point.y=odom.y
                    r=0
                    pa=1
                    state=1

                else:
                    pb=1
                    state=1

            # checking if there's a wall right in front of the goal
            elif ((dist_diff<0.2) and (dist_diff-scan.region['front'])>0.0):
                pc=1
                state=1

            # checking for final goal
            elif((dist_diff<0.2) and (dist_diff-scan.region['front'])<0.0):
                speed.linear.x=0.0 # stopping linear movement
                speed.angular.z=0.0 # stopping rotation
                p=1
                break
            else:
                pd=1
                state=1

            if(p==1):
                break


            print ("current state: ", state_dict[state])

        elif state == 1:
            # circumnavigate obstacle. 
            '''
            Hint: 
                Here robot should turn right/left based on your choice. And, circumnavigate the obstacle using wall following
                algorithm from previous project. While in this state, record the beta-alpha mentions in the Bug 2 algorithm
                in the Project 3 manual. This state terminates as failure when you reach hit point again.

                Finally, do not forget to change the state!

                It's an updated version of the "follow_wall.py"
            '''
            # TODO:
            #c=1
            if s==0:
                # Turn left by rotating the turtlebot anti-clockwise if near wall
                if ((scan.region['front'] <= d and scan.region['fleft'] > d and scan.region['fright'] > d)or
            (scan.region['front'] <= d and scan.region['fleft'] > d and scan.region['fright'] <= d )or
            (scan.region['front'] <= d and scan.region['fleft'] <= d and scan.region['fright'] > d ) or
            (scan.region['front'] <= d and scan.region['fleft'] <= d and scan.region['fright'] <= d )):

                    speed.linear.x = 0.01 # setting linear speed
                    speed.angular.z=0.3 # rotating anti-clockwise

                    #checking if turtlebot is away from the hit point
                    if(sqrt(((odom.x-hit_point.x)**2)+((odom.y-hit_point.y)**2))>1):
                        r+=1
                    
                    # checking to see if slope aligns near a hit point
                    if((abs(zeta)<=0.1) and r>0 and (pa==1 or pb==1 or pc==1 or pd==1) and (dist_diff< (sqrt(((goal.y-hit_point.y)**2)+((goal.x-hit_point.x)**2))))):
                        q=1
                        speed.linear.x = 0.0 # stopping linear speed
                        speed.angular.z=0.0 # rotating anti-clockwise
                        state=0

                        
                    # checking to see if slope aligns away from a hit point
                    elif ((abs(zeta)<=0.1) and (sqrt(((odom.x-hit_point.x)**2)+((odom.y-hit_point.y)**2))>0.2) and (dist_diff< (sqrt(((goal.y-hit_point.y)**2)+((goal.x-hit_point.x)**2))))):
                        speed.linear.x = 0.0# stopping linear speed
                        speed.angular.z=0.0 # rotating anti-clockwise
                        state=0
                    s=1

                else:
                    s=1

            elif s==1:

                # If wall is to the right move along the wall
                if (scan.region['front'] > d and scan.region['fleft'] > d and scan.region['fright'] <= d ):
                
                    speed.linear.x=0.11 # setting linear speed
                    speed.angular.z=0.0 # stopping rotation
                    if(sqrt(((odom.x-hit_point.x)**2)+((odom.y-hit_point.y)**2))>1):
                        r+=1

                    # checking to see if slope aligns near a hit point
                    if(abs(zeta)<=0.1 and (r>0) and (pa==1 or pb==1 or pc==1 or pd==1) and (dist_diff< sqrt(((goal.y-hit_point.y)**2)+((goal.x-hit_point.x)**2)))):
                        speed.linear.x = 0.0 # stopping linear speed
                        speed.angular.z=0.0 # rotating anti-clockwise
                        state=0
                        

                    # checking to see if slope aligns away from a hit point
                    elif ((abs(zeta)<=0.1) and (sqrt(((odom.x-hit_point.x)**2)+((odom.y-hit_point.y)**2))>0.2) and (dist_diff< sqrt(((goal.y-hit_point.y)**2)+((goal.x-hit_point.x)**2)))):
                        speed.linear.x = 0.0 # stopping linear speed
                        speed.angular.z=0.0 # rotating anti-clockwise
                        state=0

                # If the wall is too far to the right then move ahead while turning right by rotating clockwise
                elif ((scan.region['front'] > d and scan.region['fleft'] > d and scan.region['fright'] > d )or
                (scan.region['front'] > d and scan.region['fleft'] <= d and scan.region['fright'] > d )):
                    
                    speed.linear.x=0.06 # setting linear speed
                    speed.angular.z=-0.4 # rotating clockwise
                    if(sqrt(((odom.x-hit_point.x)**2)+((odom.y-hit_point.y)**2))>1):
                        r+=1

                    # checking to see if slope aligns near a hit point
                    if(abs(zeta)<=0.1 and (r>0) and (pa==1 or pb==1 or pc==1 or pd==1) and (dist_diff< sqrt(((goal.y-hit_point.y)**2)+((goal.x-hit_point.x)**2)))):
                        speed.linear.x = 0.0 # stopping linear speed
                        speed.angular.z=0.0 # rotating anti-clockwise
                        state=0
                        

                    # checking to see if slope aligns away from a hit point
                    elif ((abs(zeta)<=0.1) and (sqrt(((odom.x-hit_point.x)**2)+((odom.y-hit_point.y)**2))>0.2) and (dist_diff< sqrt(((goal.y-hit_point.y)**2)+((goal.x-hit_point.x)**2)))):
                        speed.linear.x = 0.0 # stopping linear speed
                        speed.angular.z=0.0 # rotating anti-clockwise
                        state=0

                    s=0

                else:
                    s=0


            print ("current state: ", state_dict[state])

        print (scan.region)
        pub.publish(speed)
        rate.sleep()


# call this function when you press CTRL+C to stop the robot
def stop():
    global pub
    speed = Twist()
    speed.linear.x = 0.0
    speed.angular.z = 0.0

    pub.publish(speed)

if __name__ == '__main__':
    main()
