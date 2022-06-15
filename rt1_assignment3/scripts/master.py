#! /usr/bin/env python3

import rospy
import os
import time
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID, GoalStatusArray
import actionlib
from sensor_msgs.msg import LaserScan
from getkey import getkey, keys
import roslaunch

# publisher of velocity
pub_vel = rospy.Publisher("cmd_vel", Twist, queue_size = 50)

vel = Twist()

##############################################################################

#mode one is to reach to the goal position
def mode_one():
    os.system('clear')
    
    stop_vel = Twist()
    
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    
    # Wait 60 seconds for the action server to become available
    client.wait_for_server(rospy.Duration(60))
    rospy.loginfo("Connected to move base server")
    
    #self.my_goal = MoveBaseActionGoal()
    my_goal = MoveBaseGoal()
    my_goal.target_pose.header.frame_id = "map"
    my_goal.target_pose.pose.orientation.w = 1
    
    
    return_system = False

    while return_system == False:

        ## renew the system
        os.system('clear')
        rospy.loginfo("Autonomous driving mode active!")

        ## enter x y coordinate
        goal_x = float(input("Insert x coordinate: "))
        goal_y = float(input("Insert y coordinate: "))
        
        rospy.loginfo("heading towards :"+ str(goal_x) + ", " + str(goal_y))
        
        ##publish x, y values
        my_goal.target_pose.pose.position.x = goal_x
        my_goal.target_pose.pose.position.y = goal_y
        
        #Send the goal pose to the MoveBaseAction server
        client.send_goal(my_goal)

        print("press 'c' to cancel goal position")
        
        # Allow 1 minute to get there
        duration = rospy.Duration(60)
                
        #while True:                        
            # If robot didn't get there in time, abort the goal
        if not duration: 
            rospy.loginfo("Timed out achieving goal position!")
            client.cancel_goal()
            rospy.loginfo("goal has not been achieved in allocated time")
            stop_vel.linear.x = 0
            stop_vel.angular.z = 0
            pub_vel.publish(stop_vel)
            
        elif getkey() == 'c' or getkey() == 'C':
                client.cancel_goal()
                rospy.loginfo("goal has been canceled")
            
                stop_vel.linear.x = 0
                stop_vel.angular.z = 0
                pub_vel.publish(stop_vel)
                    
        else:
            # We made it!
            status = client.get_state()
            if status == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")
            #return
        # if you want to provide new goal
        again = input("Do you want to provide new goal? \nIf yes, press [y] or [Y]: \n")
        if again == 'y' or again == 'Y':
            return_system = False
        else:
            return_system = True
    return 1

#########################################################################################
# to drive robot manually
def mode_two():
    os.system('clear')
    rospy.loginfo("teleop node starting..")
    pakage = 'rt1_assignment3'
    executable = 'teleop.py'

    node = roslaunch.core.Node(pakage, executable)
    
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    script = launch.launch(node)
    
    rospy.loginfo(script.is_alive())
    
    print("Drive it...")
    #return_v = True
    #while return_v:
        ## start controlling the robot and publish
    #    teleop_keys() 
    #    time.sleep(0.2)
    #    if return_v is False: 
    #        break 
    return 1


############################################################
#SECOND WAY TWO DRIVE ROBOT MANUALLY
# Control keys function to drive robot manually

def teleop_keys():
 
    global vel   
    key = getkey()
        
    ## keys for movement forward, backward, right, left & stop
        
    if key == 'i' or key == 'I':
        vel.linear.x = +1.0
    elif key == 'k' or key == 'K':
        vel.linear.x = 0.0
    elif key == ',' or key == '<':
        vel.linear.x = -1.0
        
    elif key == 'j' or key == 'J':
        vel.angular.z = +10.0
        pub_vel.publish(vel)
        time.sleep(0.1)
        vel.angular.z = 0.0
                
    elif key == 'l' or key == 'L':
        vel.angular.z = -10.0
        pub_vel.publish(vel)
        time.sleep(0.1)
        vel.angular.z = 0.0
                
    elif key == 'q' or key == 'Q':
        return False        
    else:
        print("invalid key pressed")
    
    pub_vel.publish(vel) 
    return 1

##############################################################################

def clbk_laser(msg):
    regions = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:719]), 10),
    }

    take_action(regions)


def take_action(regions):
    
    linear_x = 0
    global status 
    state_description = ''

    if regions['front'] > 0.5 and regions['fleft'] > 0.5 and regions['fright'] > 0.5:
        state_description = 'case 1 - nothing'
        status = 1
        
    elif regions['front'] < 0.5 and regions['fleft'] > 0.5 and regions['fright'] > 0.5:
        state_description = 'case 2 - front'
        status = 2
        
    elif regions['front'] > 0.5 and regions['fleft'] > 0.5 and regions['fright'] < 0.5:
        state_description = 'case 3 - fright'
        status = 3
        
    elif regions['front'] > 0.5 and regions['fleft'] < 0.5 and regions['fright'] > 0.5:
        state_description = 'case 4 - fleft'
        status = 4
        
    elif regions['front'] < 0.5 and regions['fleft'] > 0.5 and regions['fright'] < 0.5:
        state_description = 'case 5 - front and fright'
        status = 5
        
    elif regions['front'] < 0.5 and regions['fleft'] < 0.5 and regions['fright'] > 0.5:
        state_description = 'case 6 - front and fleft'
        status = 6
        
    elif regions['front'] < 0.5 and regions['fleft'] < 0.5 and regions['fright'] < 0.5:
        state_description = 'case 7 - front and fleft and fright'
        status = 7
        
    elif regions['front'] > 0.5 and regions['fleft'] < 0.5 and regions['fright'] < 0.5:
        state_description = 'case 8 - fleft and fright'
        status = 8
        
    else:
        state_description = 'unknown case'
        #rospy.loginfo(regions)
    
    if regions['front'] < 0.5 and vel.linear.x > linear_x: #emergency stop
        vel.linear.x = linear_x    

    #rospy.loginfo(state_description)
       
    pub_vel.publish(vel)
    
    
def mode_three():
    os.system('clear')
    global status
    global vel
    
    #subscriber of laserscaner
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    
    msg = """ Assistive collision avoidance driving is activated

Press navigation keys for provding assistence to Robot    

---------------------------
Moving around:
        i    
   j    k    l
        ,     
        
For Holonomic mode (strafing), hold down the shift key:
---------------------------
        I     
   J    K    L
        <     
   
Press Q or q to quit
   
   """
    print(msg)
    while True:
        key = getkey()
        if True:
            if status == 1:
                vel.linear.x = 1.0
            if key == 'i' or key == 'I':
              if status==2 or status==5 or status==6 or status==7: 
                vel.linear.x = 0
                
              else:
                vel.linear.x = 1.0
            elif key == 'k' or key == 'K':
                vel.linear.x = 0
            elif key == ',' or key == '<':
                vel.linear.x = -1
            elif key == 'j' or key == 'J':
              if (status==4 or status==6 or status==7 or status==8) and vel.linear.x != 0 :
                  vel.angular.z = 0.5
              else:
                vel.angular.z = 10
                pub_vel.publish(vel)
                time.sleep(0.1)
                vel.angular.z = 0
            elif key == 'l' or key == 'L':
              if (status==3 or status==5 or status==7 or status==8) and vel.linear.x != 0 :
                 vel.angular.z = 0.5
              else:
                vel.angular.z = -10
                pub_vel.publish(vel)
                time.sleep(0.1)
                vel.angular.z = 0
            elif key == 'q' or key == 'Q':
                print('quit from assistive mode!')
                break 
        pub_vel.publish(vel) 
    
########################################################################################    

#Main function
def main():
    os.system('clear')

    msg = '''
    Press key 1 for autonomous drive of the robot
    Press key 2 for manual driving of the robot
    Press key 3 for manual driving of the robot with collision avoidance
    Press key 4 for quitting the system"
    '''

    print(msg)
    
    rospy.init_node('all_modes')
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)

    while not rospy.is_shutdown():
        while True:
            try:
                key = int(input("Choose the driving mode of the robot: "))
                break
            except:
                print("Your input is not possible to obtain")
        if key == 1:
            mode_one()
               
        elif key == 2:
            mode_two()
                
        elif key == 3:
            mode_three()
                
        elif key == 4:
            exit()
            
        else:
            rospy.loginfo("Invalid input")
    
    rospy.spin()
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass 
