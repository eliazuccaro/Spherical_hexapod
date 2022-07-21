#!/usr/bin/env python3

# --------- Keyboard input listening node ---------

import time
import curses
import rospy
from math import pi, tan
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import Vector3, Twist
import shellbot_kinematic_laws as kinematic_laws
from shellbot_pkg.msg import Pose

# message to be shown in hexapod mode
msg_hexapod = """
        SHELLBOT GUI - HEXAPOD MODE

  -------------- WALKING --------------
      7  8  9       rot_L  forw  rot_R
      4  5  6   ->   left  stop  right
         2                 back

  --------------- POSE ----------------
    q  w  e   ->   -yaw  +pitch  +yaw
    a  s  d   ->  -roll  -pitch  +roll

  0 --> trigger shell mode

  HEXAPOD MODE VELOCITIES (%)
    forward:  {:1.2f}
    lateral:  {:1.2f}
    rotation: {:1.2f}

       CTRL-C or ESC to quit gui
"""

# message to be shown in shell mode
msg_shell = """
        SHELLBOT GUI - SHELL MODE

  --------------- MOVING --------------
      7  8  9       rot_L  forw  rot_R
      4  5  6   ->   left  stop  right
         2                 back
  -------------------------------------

  1 -> trigger hexapod mode

  SHELL MODE VELOCITIES (%)
    forward:  none
    lateral:  none    


       CTRL-C or ESC to quit gui
"""

# main body pose increment at each key pressed
inc_angle = pi / 128.
inc_pos = .0025

# walking increment at each key pressed
inc_vel_fw = 0.05
inc_vel_lat = 0.05
inc_vel_rot = 0.05
inc_pos_up = 0.02
inc_ang_up = 0.05

rot_threshold = 0.35   # maximum absolute rotation velocity allowed while walking (otherwhise stop and rotate only)
lin_threshold = 0.7    # maximum linear velocity allowed for combined linear movements

theta_open = pi/4      # maximum upper shells opening angle

# key links, only for main body's pose movements
poseMoves = {

    97: (0., 0., 0., 0., 0., -inc_angle),  # a
    100: (0., 0., 0., 0., 0., inc_angle),  # d
    115: (0., 0., 0., 0., -inc_angle, 0.),  # s
    119: (0., 0., 0., 0., inc_angle, 0.),  # w
    113: (0., 0., 0., -inc_angle, 0., 0.),  # q
    101: (0., 0., 0., inc_angle, 0., 0.),  # e
    259: (2 * inc_pos, 0., 0., 0., 0., 0.),  # up
    258: (-2 * inc_pos, 0., 0., 0., 0., 0.),  # down
    260: (0., -inc_pos, 0., 0., 0., 0.),  # left
    261: (0., inc_pos, 0., 0., 0., 0.),  # right
    49: (0., 0., 6 * inc_pos, 0., 0., 0.),  # keypad_1
    48: (0., 0., -6 * inc_pos, 0., 0., 0.),  # keypad_0
    
}

# maximum walking limits
max_vel_fw = 1
max_vel_lat = 1
max_vel_rot = 1
max_pos_upper = 1
min_pos_upper = 0
max_pos_upangle = 1
min_pos_upangle = 0

# maximum limits for main body pose
max_x = .2
max_y = .05
max_z = kinematic_laws.STARTING_Z * 2
max_z = 1
min_z = 0.05
max_phi = pi / 30.
max_theta = pi / 30.
max_psi = pi / 30.


# enshure that no values has module bigger than max allowed
def enforceLimits(val, max_val, min_val=None):

    if min_val is None:
        min_val = -max_val

    if(val > max_val):
        return max_val

    if(val < min_val):
        return min_val

    return val


def main(screen):

    rospy.init_node('teleop_key', anonymous=False)
    pose_pub = rospy.Publisher('des_pose', Pose, queue_size=1)
    dir_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    upper_pub = rospy.Publisher('up_coord', Float64MultiArray, queue_size=1)
    mode_pub = rospy.Publisher('mode', String, queue_size=1)

    direction = Twist()
    direction.linear = Vector3()
    direction.angular = Vector3()
    upper_coordinates_msg = Float64MultiArray()
    pose = Pose()
    mode = String

    # messages initialization
    direction.linear.x = 0      # forward
    direction.linear.y = 0      # lateral
    direction.angular.z = 0     # rotation
    upper_position = 1          # upper shell center quote
    upper_angle = 0             # upper shell angle
    pose.x = 0
    pose.y = 0
    pose.z = kinematic_laws.STARTING_Z
    pose.z = 0
    pose.phi = 0
    pose.theta = 0
    pose.psi = 0
    mode = "hexapod"
    upper_coordinates_msg.data = upper_position, upper_angle        # upper coordinates message composition

    
    try:

        # graphic stuff
        curses.noecho()
        curses.curs_set(0)
        screen.keypad(1)
        screen.scrollok(1)
        msg = msg_hexapod                       # screen message initialization
        screen.addstr(msg.format(0, 0, 0))
        screen.nodelay(1)

        rate = rospy.Rate(50)  # Hz

        while not rospy.is_shutdown():

            # get last key pressed (after cleaning-up the buffer)
            curses.flushinp()
            time.sleep(0.01)

            key = screen.getch()

            if (key == 27):  # Esc
                break

            # ----- messages filling for topics /cmd_vel, /des_pose and /mode -----

            # walking commands
            elif (key == 53):  # numpad center (5)
                direction.linear.x = 0
                direction.linear.y = 0
                direction.angular.z = 0
                pose.phi = 0
                pose.theta = 0
                pose.psi = 0


            elif (key == 56):  # numpad up (8)
                direction.linear.x = enforceLimits(direction.linear.x + inc_vel_fw, max_vel_fw)
                direction.angular.z = 0

            elif (key == 50):  # numpad down (2)
                direction.linear.x = enforceLimits(direction.linear.x - inc_vel_fw, max_vel_fw)
                direction.angular.z = 0

            elif (key == 52):  # numpad left (4)
                direction.linear.y = enforceLimits(direction.linear.y + inc_vel_lat, max_vel_lat)
                direction.angular.z = 0
                
            elif (key == 54):  # numpad right (6)
                direction.linear.y = enforceLimits(direction.linear.y - inc_vel_lat, max_vel_lat)
                direction.angular.z = 0

            elif (key == 55):  # numpad rotate left (7)
                direction.angular.z = enforceLimits(direction.angular.z + inc_vel_rot, max_vel_rot)

            elif (key == 57):  # numpad rotate right (9)
                direction.angular.z = enforceLimits(direction.angular.z - inc_vel_rot, max_vel_rot)

            elif (key == 43):  # numpad lift upper legs (+)
                upper_position = enforceLimits(upper_position + inc_pos_up, max_pos_upper, min_pos_upper)
                upper_coordinates_msg.data = upper_position, upper_angle        # upper coordinates message composition
                # print("Upper legs lifting")

            elif (key == 45):  # numpad push upper legs down (-)
                upper_position = enforceLimits(upper_position - inc_pos_up, max_pos_upper, min_pos_upper)
                upper_coordinates_msg.data = upper_position, upper_angle        # upper coordinates message composition
                # print("Upper legs going down")

            elif (key == 51):  # numpad open upper legs (3)
                upper_angle = enforceLimits(upper_angle - inc_ang_up, max_pos_upangle, min_pos_upangle)
                upper_coordinates_msg.data = upper_position, upper_angle        # upper coordinates message composition

            elif (key == 46):  # numpad close upper legs (.)
                upper_angle = enforceLimits(upper_angle + inc_ang_up, max_pos_upangle, min_pos_upangle)
                upper_coordinates_msg.data = upper_position, upper_angle        # upper coordinates message composition

            # development & debug line - do not uncomment
            # elif (key != -1): print(key)

            elif (key == 48):  # numpad shell mode (0)
                direction.linear.x = 0
                direction.linear.y = 0
                direction.angular.z = 0
                pose.phi = 0
                pose.theta = 0
                pose.psi = 0
                mode = "shell"
                msg = msg_shell

            elif (key == 49):  # numpad hexapod mode (1)
                direction.linear.x = 0
                direction.linear.y = 0
                direction.angular.z = 0
                pose.phi = 0
                pose.theta = 0
                pose.psi = 0
                upper_position = 1          # upper shell center quote
                upper_angle = 0             # upper shell angle
                mode = "hexapod"
                msg = msg_hexapod

            # if rotating too much it can't walk at the same time
            if (abs(direction.angular.z) > rot_threshold):
                direction.linear.x = 0
                direction.linear.y = 0

            # if translating too much in both directions, stop TODO: magari mettere un'altra condizione
            if (abs(direction.linear.x) > lin_threshold) and (abs(direction.linear.y) > lin_threshold):
                direction.linear.x = 0
                direction.linear.y = 0

            # pose commands
            if key in poseMoves.keys():

                pose.x = enforceLimits(pose.x + poseMoves[key][0], max_x)
                pose.y = enforceLimits(pose.y + poseMoves[key][1], max_y)
                pose.z = enforceLimits(pose.z + poseMoves[key][2], max_z, min_z)
                pose.phi = enforceLimits(pose.phi + poseMoves[key][3], max_phi)
                pose.theta = enforceLimits(pose.theta + poseMoves[key][4], max_theta)
                pose.psi = enforceLimits(pose.psi + poseMoves[key][5], max_psi)

            dir_pub.publish(direction)
            pose_pub.publish(pose)
            mode_pub.publish(mode)
            upper_pub.publish(upper_coordinates_msg)

            screen.addstr(0, 0, msg.format(direction.linear.x, direction.linear.y, direction.angular.z))
            screen.refresh()

            rate.sleep()

    except (rospy.ServiceException, rospy.ROSInterruptException) as e:
        
        pass


if __name__ == "__main__":
    curses.wrapper(main)
