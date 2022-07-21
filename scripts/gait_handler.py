#!/usr/bin/env python3
import rospy
from math import pi, sin, cos
import numpy as np
# import time
from iDynTree import vectorize, forwardKinematicsDH
import shellbot_kinematic_laws as kinematic_laws
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, String

# Discrete time sampling interval for gait desired positions
DT = .05  # [s]

# ------------- gait values ------------
# Here is defined maximum distances for one single step:
# the topic '/cmd_vel' will indicate a percentage of this values, not a real velocity in m/s.
# The time of a single gait is fixed, the motion velocity is imposed by gait distances.

# TODO --> sono collegati ai parametri contenuti in teleop_key, sistemarli di pari passo
GAIT_STEP_TIME = 3  # [s] tempo dedicato al compimento di un passo
FW_MAX_DIST = 50  # [mm]   maximum distance of one single gait (forward)
LAT_MAX_DIST = 50  # [mm]  maximum distance of one single gait (lateral)

MAX_ROT = pi / 6  # [rad]  maximum allowed rotation --> dovrebbe essere circa 30°
Z_STEP_RAISE = kinematic_laws.STARTING_Z / 3.  # [mm] Z lift for non-resting foot

# TODO --> qui vanno messi i limiti di velocità degli altri movimenti, tutti riferiti al GAIT_STEP_TIME
FW_MAX_VEL = FW_MAX_DIST / GAIT_STEP_TIME
LAT_MAX_VEL = LAT_MAX_DIST / GAIT_STEP_TIME
ROT_MAX_VEL = MAX_ROT / GAIT_STEP_TIME

# TODO --> questi flag definiscono la logica dei movimenti, evenutalmente aggiungere quelli necessari
is_moving = False
gait_state = 0.  # rest: 0 | step right: 1 | step left: -1

# TODO --> questi valori sono le "manopole" di velocità dei movimenti, va inserito un valore per ogni movimento di cui si voglia definire la velocità
# current velocities as percentual of the maximum --> sono i valori che compaiono nella schermata di teleop_key.py
forward_speed_perc = 0.  # [-1, .. , 1]
lateral_speed_perc = 0.  # [-1, .. , 1]
rotation_speed_perc = 0.  # [-1, .. , 1]

# topic a cui si sottoscrive l'inverse kinematic server --> è il risultato di questo nodo
# TODO: vedere che succede se qui si mette effettivamente una 6 x 4
des_feet_position = np.array([[None for j in range(3)] for i in range(6)])

# messaggio che viene inviato nel topic /shellbot/des_feet_position
des_feet_msg = Float64MultiArray()
des_feet_msg.layout.dim = [MultiArrayDimension()]
des_feet_msg.layout.dim[0].stride = 1
des_feet_msg.layout.dim[0].size = 18

# robot body parameters
starting_z = kinematic_laws.STARTING_Z
l_o0_z = kinematic_laws.l_o0_z
l_o0_x = kinematic_laws.l_o0_x
# l_01_z = 0 --> h1 = 0
l_01_x = kinematic_laws.l_01_x
l_12_x = kinematic_laws.l_12_x
l_12_z = kinematic_laws.l_12_z
l_23_x = kinematic_laws.l_23_x
l_23_z = kinematic_laws.l_23_z
l_34_x = kinematic_laws.l_34_x
# l_34_x = 0 --> h4 = 0
theta_1_open = kinematic_laws.theta_1_open
theta_1_close = kinematic_laws.theta_1_close
theta_2_still = kinematic_laws.theta_2_still
theta_3_still = kinematic_laws.theta_3_still
theta_4_still = kinematic_laws.theta_4_still

# parametri DH per far stare ferme le gambe inferiori (theta dipende dalla gamba)
d_vector = vectorize([l_o0_z, 0, l_12_z, l_23_z, 0])
alpha_vector = vectorize([0, 0, -pi/2, 0, 0])
a_vector = vectorize([l_o0_x, l_01_x, l_12_x, l_23_x, l_34_x])


# -------------------- Feet rest and actual positions -------------------

# homogeneous vectors representing resting tiptoe positions --> i.e. where the feet are when the hexapod stands still.
def feet_rest_position(leg_j):

    # robot_center_pose = vectorize([0, 0, 0, 0, 0, starting_z]).T      # --> non so se a noi possa servire
    # matrice T dall'origine spatial al centro del robot
    robot_center_T = np.array([[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, starting_z],[0, 0, 0, 1]])
    # leg_start_j = kinematic_laws.space_to_bodyCenter(robot_center).dot(kinematic_laws.bodyCenter_to_legStart(leg_j))      # ARAGOG #

    theta_vector = vectorize([(2*pi*leg_j)/6, theta_1_open, theta_2_still, theta_3_still, theta_4_still])

    if leg_j > 6:
        raise Exception('feet_rest_position: leg_j must be in [1-6]')

    # TODO: capire perché [:, 3] -> vedere la funzione forwardKinematicsDH -> questa funzione prende in input una tablella DH e restituisce la trasformazione omogenea risultante
    leg_end_j = robot_center_T.dot(forwardKinematicsDH(theta_vector, d_vector, alpha_vector, a_vector))[:, 3]

    # return (robot_center_T.dot(forwardKinematicsDH(theta_vector, d_vector, alpha_vector, a_vector)))[:, 3]
    return (leg_end_j)


# TODO: implement actual_feet_position ------ questo potrebbe non essere necessario
def actual_feet_position(leg_j):
    # leg_start_j = kinematic_laws.space_to_bodyCenter([0, 0, 0, 0, 0, kinematic_laws.STARTING_Z]).dot(kinematic_laws.bodyCenter_to_legStart(leg_j))
    # return (leg_start_j.dot(kinematic_laws.Tbf(get_actual_qs(leg_j))))[:, 3]

    return feet_rest_position(leg_j)


# ---------------------------- Gait handling ----------------------------

def gait_next_step(leg_j, start_position, is_left_step, tau):

    # TODO --> verificare che il mappaggio sia lo stesso (dovrebbe esserlo)
    # selezione delle gambe da muovere in base alla variabile gait_state
    if is_left_step:
        resting_legs = [1, 3, 5]
        moving_legs = [2, 4, 6]
    else:
        resting_legs = [2, 4, 6]
        moving_legs = [1, 3, 5]

    ######################### 1) first the rotation, if any #########################
    # ruota in verso "positivo" le moving_legs e in senso negativo le resting_legs
    # incrementa/decrementa la velocità in yaw
    rot_angle = (rotation_speed_perc * MAX_ROT/2) if leg_j in moving_legs else (-rotation_speed_perc * MAX_ROT/2)

    # qui andrebbero inseriti posx, posy, posz
    # posx_ground = dist * cos[MAX_ROT*t*rotation_speed_perc+2*leg_j*pi/-pi/3]
    # posy_ground = dist * sin[MAX_ROT*t*rotation_speed_perc+2*leg_j*pi/-pi/3]
    # posz_ground = -170
    # posx_lift = dist * cos[-MAX_ROT*t*rotation_speed_perc+2*leg_j*pi/-pi/3]
    # posy_lift = dist * sin[-MAX_ROT*t*rotation_speed_perc+2*leg_j*pi/-pi/3]
    # posz_lift = -170 + Z_STEP_RAISE + sin((t+n)*pi/(n+1))

    # posx_punto_ground = dist * MAX_ROT * rotation_speed_perc * cos[pi/6 + leg_j*pi/3 + MAX_ROT*rotation_speed_perc*t]
    # posy_punto_ground = dist * MAX_ROT * rotation_speed_perc * sin[pi/6 + leg_j*pi/3 + MAX_ROT*rotation_speed_perc*t]
    # posx_punto_lift = - dist * MAX_ROT * rotation_speed_perc * cos[pi/6 + leg_j*pi/3 - MAX_ROT*rotation_speed_perc*t]
    # posy_punto_lift = - dist * MAX_ROT * rotation_speed_perc * sin[pi/6 + leg_j*pi/3 - MAX_ROT*rotation_speed_perc*t]
    # posz_punto_lift = - height*pi*cos(pi*(n+t)/(1+n))/(1+n)

    # rotazione di matrice omogenea lungo l'asse z
    rotated_position_j = np.array([[cos(rot_angle), -sin(rot_angle), 0, 0],
                                   [sin(rot_angle), cos(rot_angle), 0, 0],
                                   [0, 0, 1, 0],
                                   [0, 0, 0, 1]]).dot(feet_rest_position(leg_j))
    #################################################################################

    ######################### 2) then forward and lateral motion #########################
    # è qui che vengono generate le posizioni finali desiderate alla fine del "passo"
    # vengono sommati i movimenti lineari a quelli rotazionali già applicati

    if leg_j in resting_legs:        # these feet will be kept to the ground

        # forward_speed_perc e lateral_speed_perc vengono riempite dalle callback "in background"
        # queste gambe spingono all'indietro senza lazarsi da terra
        final_position = rotated_position_j - [forward_speed_perc * FW_MAX_DIST/2, lateral_speed_perc * LAT_MAX_DIST/2, 0, 0]
        z_raise = 0

    elif leg_j in moving_legs:       # these feet will advance

        final_position = rotated_position_j + [forward_speed_perc * FW_MAX_DIST/2, lateral_speed_perc * LAT_MAX_DIST/2, 0, 0]
        z_raise = -170 + Z_STEP_RAISE*sin((tau)*pi) # TODO: come inserire n?

    else:

        raise Exception('left_step: leg_j must be in [1-6]')
    #######################################################################################

    # convex combination of initial and final state, plus a lift on Z.
    current_position = (1 - tau) + final_position * tau + [0, 0, z_raise, 0]

    next_position = start_position * current_position

    return next_position


# Callback for /cmd_vel topic listener
def on_move_update(msg):

    global forward_speed_perc, lateral_speed_perc, rotation_speed_perc, is_moving

    # qui vengono riempite le quantità di tutte le variabili legate a movimenti modulabili
    forward_speed_perc = msg.linear.x
    lateral_speed_perc = msg.linear.y
    rotation_speed_perc = msg.angular.z

    # flag per capire se il robot si muova o meno
    is_moving = (forward_speed_perc != 0) or (lateral_speed_perc != 0) or (rotation_speed_perc != 0)
    # rospy.loginfo('Received [%f %f %f]', msg.x, msg.y, msg.z)

# Callback for /mode topic listener
def mode_update(mode_msg):

    global mode
    mode = mode_msg.data
    # print("mode: " + mode)


if __name__ == "__main__":      # listen to cmd_vel and publish desired feet positions at rate 1/DT

    rospy.init_node('gait_handler', anonymous = False) # inizializzazione del nodo
    rospy.Subscriber('cmd_vel', Twist, on_move_update) # sottoscrizione al topic /cmd_vel
    rospy.Subscriber('mode', String, mode_update) # sottoscrizione al topic /mode
    des_feet_pub = rospy.Publisher('des_feet_position', Float64MultiArray, queue_size = 1) # definizione del publisher del topic /des_feet_position

    # sono le posizioni di partenza di ogni passo, vengono aggiornate a ogni passo --> per ogni movimento ci potrebbe essere bisogno di una diversa start_position
    start_position = np.zeros(6 * 3).reshape(6, 3) # matrice di coordinate omogenee: posizione di 18 giunti -> 31 x [x, y, z, 1] TODO: provare a metterle a 6 x 4

    tau = 0. # normalized time interval [0, .. , 1]

    # initial desired feet positions to allow inverse_kinematic_server to run --> cinematica diretta da space alle punte dei piedi
    for leg_j in range(1, 7):
        
        des_feet_position[1:, :] = np.array([feet_rest_position(leg_j)])

    actual_t = rospy.Time.now()     # tempo corrente
    rate = rospy.Rate(1 / DT)       # frequenza di pubblicazione in Hz

    print("gait_handler is online at rate {}Hz.".format(1 / DT))

    try:

        while not rospy.is_shutdown():

            actual_t = rospy.Time.now()   # tempo corrente
            prev_t = actual_t             # riempimento variabile prev_t col tempo corrente
            real_dt = (actual_t - prev_t).secs + (actual_t - prev_t).nsecs * 1.0e-9  # [s] --> tempo trascorso nello step corrente
            
            # da qui le traiettorie vengono differenziate a seconda del task
            # if mode == "hexapod": --> non vede il topic

            if not is_moving and tau == 0.:     # rest condition --> riempie il vettore "des_feet_position" coi valori calcolati da "feet_rest_position"

                # TODO: valutare se inserire qui una funzione per far passare al robot gli angoli per stare fermo, o se magari farla scattare dopo un tot che il robot non riceve più comandi
                for leg_j in range(1, 7):
                    des_feet_position[1:, :] = np.array([feet_rest_position(leg_j)])    # riempie le des_feet_position con posizioni a riposo
                gait_state = 0

            else:   # not in rest condition

                ################################# if che si attiva solo al primo passo #################################
                if gait_state == 0:
                    for leg_j in range(1, 7):
                        # riempie le start_position con le posizioni attuali dei piedi --> posizione di partenza passo destro
                        start_position[leg_j] = actual_feet_position(leg_j)     # in Aragog c'era la actual_rest_position, uguale alla feet_rest_position
                    gait_state = 1      # then start with a right step
                    tau = 0.            # reinizializzazione intervallo di tempo normalizzato --> inizio del passo
                ########################################################################################################

                tau += real_dt / GAIT_STEP_TIME     # valore corrente del tempo nell'intervallo di tempo normalizzato

                ################################# cambio passo destro/sinistro #################################
                if tau >= 1.:   # time exceedes normalized interval --> switch from left to right step and vice-versa
                    for leg_j in range(1, 7):
                        # store last positions as new starts --> al cambio di passo, le vecchie des_feet_position diventano le nuove start_position
                        start_position[leg_j] = des_feet_position[leg_j]
                    gait_state *= -1        # invert resting foot
                    tau = 0.                # reinizializzazione intervallo di tempo normalizzato
                ################################################################################################
                
                # compute next time-step position of feet
                des_feet_position[1:, :] = np.array([gait_next_step(leg_j, start_position[leg_j], gait_state == 1, tau) for leg_j in range(1, 7)])      # compute feet positions of next time-step


                # "appiattimento" del messaggio e pubblicazione
                des_feet_msg.data = kinematic_laws.flatten(des_feet_position[1:, :])  # take ony not None values
                des_feet_pub.publish(des_feet_msg)

            # elif mode == "shell":

                # pass

            # print("[Evaluation time: %s ms]\n" % (time.time() - start_time)*1000)
            # it seems to run at ~2ns per loop
            rate.sleep()

    except rospy.ROSInterruptException:
        pass