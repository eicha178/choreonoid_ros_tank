#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import sys, select, termios, tty

MAX_LIN_VEL = 1.00
MAX_ANG_VEL = 1.00

LIN_VEL_STEP_SIZE = 0.25
ANG_VEL_STEP_SIZE = 0.2

msg = """
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d

space key : stop
CTRL-C to quit

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
"""

e = """
Communications Failed
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

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def torques(target_linear_vel, target_angular_vel):
    return "currently:\tjoint0 %s\t joint1 %s " % (target_linear_vel,target_angular_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    vel = constrain(vel, -MAX_LIN_VEL, MAX_LIN_VEL)

    return vel

def checkAngularLimitVelocity(vel):
    vel = constrain(vel, -MAX_ANG_VEL, MAX_ANG_VEL)

    return vel

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)


    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    pub_arm = rospy.Publisher('cmd_arm', JointState, queue_size=10)

    rospy.init_node('key_to_msgs')

    status = 0
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0

    joint_state = JointState()
#    joint_state.header = Header()
#    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = ['joint0', 'joint1']
    joint_state.position = []
    joint_state.velocity = []
    joint_state.effort = [0.0, 0.0]

    try:
        print msg
        while(1):
            key = getKey()

            if key == 'w' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                print vels(target_linear_vel,target_angular_vel)
            elif key == 's' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print vels(target_linear_vel,target_angular_vel)
            elif key == 'a' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                status = status + 1
                print vels(target_linear_vel,target_angular_vel)
            elif key == 'd' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                status = status + 1
                print vels(target_linear_vel,target_angular_vel)
            elif key == 'r':
                joint_state.effort[0] = joint_state.effort[0] + 0.1
                print torques(joint_state.effort[0], joint_state.effort[1])
            elif key == 'f':
                joint_state.effort[0] = joint_state.effort[0] - 0.1
                print torques(joint_state.effort[0], joint_state.effort[1])
            elif key == 't':
                joint_state.effort[1] = joint_state.effort[1] + 0.1
                print torques(joint_state.effort[0], joint_state.effort[1])
            elif key == 'g':
                joint_state.effort[1] = joint_state.effort[1] - 0.1
                print torques(joint_state.effort[0], joint_state.effort[1])
            #elif key == ' ' or key == 's' :
            elif key == ' ' :
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                print vels(target_linear_vel, target_angular_vel)

                joint_state.effort[0] = 0.0
                joint_state.effort[1] = 0.0
            else:
                if (key == '\x03'):
                    break

            if status == 20 :
                print msg
                status = 0

            twist = Twist()

            control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

            pub.publish(twist)
            pub_arm.publish(joint_state)

    except:
        print e

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
