#!/usr/bin/env python3

import random as rnd
from std_msgs.msg import String
from open_manipulator_tools.msg import kinematicsActionAction, kinematicsActionGoal, kinematicsActionResult
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import rospy
import actionlib
import sys

# A board egy lista a jelenlegi állással, amit színfelismerés alapján kapunk meg.
# Ezek alapján dönt a robot arról, milyen lépést választ.
# 0 -üres cella, 1 - 1-es játékos (kék), 2 - 2-es játékos (piros)kinematicsAction.action
board = [0, 0, 0, 0, 0, 0, 0, 0, 0]

# Z síkok a felvétel és lerakás során
pick_up_high = 0.1
pick_up_low = 0.03

# az a pozíció, amiből a robot a táblát nézi
basePosition = [0.14, 0, 0.12]

# a bábúk felvételi pozíciói és a tábla pozíciói
redPieces = [[0.059, -0.094], [0.059, -0.0363], [0.059, 0.0363], [0.059, 0.094]]
redPiecesNames = ["hengeresBabuPiros_1", "hengeresBabuPiros_2", "hengeresBabuPiros_3", "hengeresBabuPiros_4"]
redPiecesPlaced = 0
bluePieces = [[0.107, -0.094], [0.107, -0.0363], [0.107, 0.0363], [0.107, 0.094]]
bluePiecesNames = ["hengeresBabuKek_4", "hengeresBabuKek_3", "hengeresBabuKek_2", "hengeresBabuKek_1"]
bluePiecesPlaced = 0
'''
boardPosition = [[-0.048, 0.2155], [0, 0.2155], [0.048, 0.2155],
                 [-0.048, 0.1675], [0, 0.1675], [0.048, 0.1675],
                 [-0.048, 0.1195], [0, 0.1195], [0.048, 0.1195]]
'''
boardPosition = [[0.2455, 0.048], [0.2455, 0], [0.2455, -0.048],
                 [0.1975, 0.048], [0.1975, 0], [0.1975, -0.048],
                 [0.1495, 0.048], [0.1495, 0], [0.1495, -0.048]]

# global variables for the action servser communication
action_in_progress = False
action_result = False

# global variables for the runtime states of the tic-tac-toe
position = 0
count_of_current_step = 0
sent_to_base_state = False
state_of_move_action = "Empty"
state_of_pick_place = "Empty"
return_of_pick_place = "Empty"
return_of_move = "Empty"


def move_selector(board, player):
    if player == 1:
        opponent = 2
    else:
        opponent = 1

    if board == [0, 0, 0, 0, 0, 0, 0, 0, 0]:  # Ha üres, random választunk elsőre
        position = rnd.randint(0, 8)
        return position
    elif is_winner(board, player) or is_winner(board, opponent):  # Megnézzük, nem-e nyert már valaki, ha igen, akkor -1-et küldünk vissza pozícióként
        position = -1
        return position
    else:  # Megnézzük, melyik cella szabad, és abból választunk logika alapján
        possibleMoves = empty_cells(board)

        for i in [player, opponent]:
            for move in possibleMoves:
                boardCopy = board[:]
                boardCopy[move] = i
                if is_winner(boardCopy, i):
                    rospy.loginfo(str(move) + " winner")
                    return move

        if 4 in possibleMoves:
            position = 4
            return position

        cornersOpen = []
        for i in possibleMoves:
            if i in [0, 2, 6, 8]:
                cornersOpen.append(i)

        if len(cornersOpen) > 0:
            position = rnd.choice(cornersOpen)
            return position

        edgesOpen = []
        for i in possibleMoves:
            if i in [1, 3, 5, 7]:
                edgesOpen.append(i)

        if len(edgesOpen) > 0:
            position = rnd.choice(edgesOpen)
            return position


def is_winner(board, color):
    return ((board[6] == color and board[7] == color and board[8] == color) or
            (board[3] == color and board[4] == color and board[5] == color) or
            (board[0] == color and board[1] == color and board[2] == color) or
            (board[0] == color and board[3] == color and board[6] == color) or
            (board[1] == color and board[4] == color and board[7] == color) or
            (board[2] == color and board[5] == color and board[8] == color) or
            (board[0] == color and board[4] == color and board[8] == color) or
            (board[2] == color and board[4] == color and board[6] == color))


def empty_cells(board):
    possibleMoves = []
    j = 0
    for i in board:
        if i == 0:
            possibleMoves.append(j)
        j += 1
    return possibleMoves


def board_color_recognition(msg):
    global board
    points = str(msg.data).split(";")
    for point in points:
        if point !=  "":
            data_split = point.split(",")
            #print(data_split)
            data = []
            data.append(data_split[0])
            data.append(int(data_split[1]))
            data.append(int(data_split[2]))
            if data[1] > 80 and data[1] < 227:
                column = 0
            elif data[1] > 250 and data[1] < 387:
                column = 1
            elif data[1] > 410 and data[1] < 558:
                column = 2
            else:
                column = -1

            if data[2] > 20 and data[2] < 156:
                row = 0
            elif data[2] > 217 and data[2] < 277:
                row = 1
            elif data[2] > 338 and data[2] < 480:
                row = 2
            else:
                row = -1

            if row != -1 and column != -1:
                if data[0] == 'B':
                    board[column+row*3] = 1
                elif data[0] == 'R':
                    board[column+row*3] = 2


def iterate(player):
    global bluePiecesPlaced, redPiecesPlaced
    if player == 1:
        bluePiecesPlaced += 1
    elif player == 2:
        redPiecesPlaced += 1

def find_figure(player):
    global bluePiecesPlaced
    global redPiecesPlaced
    if player == 1:
        figurePosition = bluePieces[bluePiecesPlaced]
    elif player == 2:
        figurePosition = redPieces[redPiecesPlaced]
    else:
        figurePosition = [0, 0]

    return figurePosition


def kinematics_client_done(state, result):
    global action_result, action_in_progress
    rospy.loginfo("Action server is done. State: %s, result: %s" % (str(state), str(result)))
    action_in_progress = False
    action_result = result


def kinematics_client_operation(coord, angle=0.0):
    global action_in_progress
    client = actionlib.SimpleActionClient('send_joint_angles_ik', kinematicsActionAction)
    rospy.loginfo('Waiting for action server...')
    client.wait_for_server()

    goal = kinematicsActionGoal(coord[0], coord[1], coord[2], angle)
    action_in_progress = True
    client.send_goal(goal, done_cb=kinematics_client_done)
    rospy.loginfo('Goal (%f, %f, %f) sent...', coord[0], coord[1], coord[2])
    #client.wait_for_result()
    #return client.get_result()
    '''try:
        kinematics_handler = rospy.ServiceProxy('kinematics_handler', kinematicsMessage)
        response = kinematics_handler()
        return response.sum
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)'''


def attach(name):
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
    attach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

    # Link them
    rospy.loginfo("Attaching gripper and %s.", name)
    req = AttachRequest()
    req.model_name_1 = "robot"
    req.link_name_1 = "gripper_link"
    req.model_name_2 = name
    req.link_name_2 = "link"
    attach_srv.call(req)


def detach(name):
    attach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
    attach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")

    # Link them
    rospy.loginfo("Detaching gripper and %s.", name)
    req = AttachRequest()
    req.model_name_1 = "robot"
    req.link_name_1 = "gripper_link"
    req.model_name_2 = name
    req.link_name_2 = "link"
    attach_srv.call(req)


def gripper_operation(player, gripperAction):
    '''False = open, True = Close'''
    controller_name = "gripper_controller"
    joint_names = rospy.get_param("/%s/joints" % controller_name)
    rospy.loginfo("Joint names: %s" % joint_names)
    trajectory_command = JointTrajectory()
    trajectory_command.header.stamp = rospy.Time.now()
    trajectory_command.joint_names = joint_names
    point = JointTrajectoryPoint()
    # Joint names: ['gripper']

    pieceName = ""
    if player == 1:
        pieceName = bluePiecesNames[bluePiecesPlaced]
    elif player == 2:
        pieceName = redPiecesNames[redPiecesPlaced]

    if simSwitch == "sim":
        if gripperAction: #true->close
            point.positions = [-0.01]
        else: #false->open
            point.positions = [0.003]
    else:
        if gripperAction: #true->close
            point.positions = [0.006]
        else: #false->open
            point.positions = [0.0]


    point.velocities = [0.0]
    point.time_from_start = rospy.rostime.Duration(1, 0)
    trajectory_command.points = [point]
    pub.publish(trajectory_command)

    rospy.sleep(0.5)
    if simSwitch == "sim" and not gripperAction and pieceName != "":  # detach model
         detach(pieceName)
    rospy.sleep(1) # várjon hogy becsukódjon a megfogó
    if simSwitch == "sim" and gripperAction and pieceName != "":  # attach model
         attach(pieceName)


def pick_or_place(position, player, gripperAction):
    global state_of_pick_place, return_of_pick_place
    if not action_in_progress:
        '''gripperAction: False = open, True = Close'''
        if state_of_pick_place == "Empty":
            return_of_pick_place = "Empty"
            if gripperAction:  # pick operation
                kinematics_client_operation([position[0], position[1], pick_up_high], math.pi/2)
            else:  # place operation
                kinematics_client_operation([position[0], position[1], pick_up_high], math.pi/4)
            state_of_pick_place = "FirstStarted"

        elif state_of_pick_place == "FirstStarted":
            if not action_result:
                state_of_pick_place = "Done"
                return_of_pick_place = "cannot reach position: " + position + " at pick_up_high"
            else:
                state_of_pick_place = "FirstFinished"

        elif state_of_pick_place == "FirstFinished":
            kinematics_client_operation([position[0], position[1], pick_up_low], math.pi/2)
            state_of_pick_place = "SecondStarted"

        elif state_of_pick_place == "SecondStarted":
            if not action_result:
                state_of_pick_place = "Done"
                return_of_pick_place = "cannot reach position: " + position + " at pick_up_low"
            else:
                state_of_pick_place = "SecondFinished"
                gripper_operation(player, gripperAction)

        elif state_of_pick_place == "SecondFinished":
            if gripperAction:  # pick operation
                kinematics_client_operation([position[0], position[1], pick_up_high], math.pi/2)
            else:  # place operation
                kinematics_client_operation([position[0], position[1], pick_up_high], math.pi/4)
            state_of_pick_place = "ThirdStarted"

        elif state_of_pick_place == "ThirdStarted":
            if not action_result:
                state_of_pick_place = "Done"
                return_of_pick_place = "cannot reach position: " + position + " at pick_up_high"
            else:
                state_of_pick_place = "Done"
        return_of_pick_place = "Done"


def move(position, player):
    global state_of_move_action, return_of_pick_place, state_of_pick_place, return_of_move
    '''position: 0-8 position to place on the board; player: 1: blue, 2: red'''
    if state_of_move_action == "Empty":
        pick_or_place(find_figure(player), player, True)
        if state_of_pick_place == "Done":
            if return_of_pick_place != "Done":
                return_of_move = "Pick operation failed"
                rospy.loginfo(return_of_pick_place)
                state_of_move_action = "Done"
            else:
                state_of_pick_place = "Empty"
                state_of_move_action = "FirstDone"

    elif state_of_move_action == "FirstDone":
        pick_or_place(boardPosition[position], player, False)
        if state_of_pick_place == "Done":
            if return_of_pick_place != "Done":
                return_of_move = "Pick operation failed"
                rospy.loginfo(return_of_pick_place)
                state_of_move_action = "Done"
            else:
                state_of_pick_place = "Empty"
                state_of_move_action = "SecondDone"
                iterate(player)

    elif state_of_move_action == "SecondDone" and not action_in_progress:
        kinematics_client_operation([basePosition[0], basePosition[1], basePosition[2]], math.pi/2)
        state_of_move_action = "ThirdStarted"

    elif state_of_move_action == "ThirdStarted" and not action_in_progress:
        if not action_result:
            return_of_move = "Pick operation failed"
            rospy.loginfo(return_of_pick_place)
            state_of_move_action = "Done"
        else:
            return_of_move = "Done"
            state_of_move_action = "Done"



def tic_tac_toe():
    global sent_to_base_state, count_of_current_step, state_of_move_action, position
    # initial player selection:
    player1 = rnd.randint(1, 2)
    if player1 == 1:  # kék
        player2 = 2  # piros
    else:
        player2 = 1
    player = player1
    rate = rospy.Rate(5)
    rospy.loginfo("tic tac toe node started")

    while not rospy.is_shutdown():  # run the node until Ctrl-C is pressed
        if count_of_current_step == 0:
            if not sent_to_base_state:
                kinematics_client_operation([basePosition[0], basePosition[1], basePosition[2]], math.pi / 2)
                if simSwitch != "sim":
                    gripper_operation(-1, True)
                gripper_operation(-1, False)
                sent_to_base_state = True
            if not action_in_progress:
                if not action_result:
                    rospy.loginfo("error while moving to the first target")
                else:
                    count_of_current_step += 1

        if 0 < count_of_current_step < 10:
            if state_of_move_action == "Empty" and state_of_pick_place == "Empty":
                rospy.sleep(2)
                board_color_recognition(rospy.wait_for_message('/color_recognition', String, 5))
                rospy.loginfo("current board: (%i,%i,%i),(%i,%i,%i),(%i,%i,%i)",
                              board[0], board[1], board[2],
                              board[3], board[4], board[5],
                              board[6], board[7], board[8])
                position = move_selector(board, player)
                if position == -1:
                    rospy.loginfo("The game ended.")
                    break
            move(position, player)
            if state_of_move_action == "Done":
                if return_of_move == "Done":
                    if player == player1:
                        player = player2
                    else:
                        player = player1
                    state_of_move_action = "Empty"
                    count_of_current_step += 1
                else:
                    rospy.loginfo(return_of_move)
            rate.sleep()


if __name__ == "__main__":
    try:
        simSwitch = rospy.myargv(argv=sys.argv)[1]
        basePosition = [0.15, 0, 0.09]
        pick_up_low = 0.02
    except:
        simSwitch = ""
        basePosition = [0.14, 0, 0.12]
        pick_up_low = 0.03
    rospy.init_node('tic_tac_toe')
    pub = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=1)
    tic_tac_toe()

        