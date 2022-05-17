import random as rnd
from std_msgs.msg import String
import rospy
from kinematicsMessage.srv import *
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import rospy

# A board egy lista a jelenlegi állással, amit színfelismerés alapján kapunk meg.
# Ezek alapján dönt a robot arról, milyen lépést választ.
# 0 -üres cella, 1 - 1-es játékos (kék), 2 - 2-es játékos (piros)kinematicsMessage.srv
board = [0, 0, 0, 0, 0, 0, 0, 0, 0]

# Z síkok a felvétel és lerakás során
pick_up_high = 0.100
pick_up_low = 0.015

# az a pozíció, amiből a robot a táblát nézi
basePosition = [0, 0.12]

# a bábúk felvételi pozíciói és a tábla pozíciói
redPieces = [[-0.064, 0.029], [-0.0213, 0.029], [0.0213, 0.029], [0.064, 0.029]]
redPiecesPlaced = 0
bluePieces = [[-0.064, 0.077], [-0.0213, 0.077], [0.0213, 0.077], [0.064, 0.077]]
bluePiecesPlaced = 0
boardPosition = [[-0.048, 0.2155], [0, 0.2155], [0.048, 0.2155],
                 [-0.048, 0.1675], [0, 0.1675], [0.048, 0.1675],
                 [-0.048, 0.1195], [0, 0.1195], [0.048, 0.1195]]


def move_selector(board, player):
    if player == 1:
        opponent = 2
    else:
        opponent = 1

    if board == [0, 0, 0, 0, 0, 0, 0, 0, 0]:  # Ha üres, random választunk elsőre
        position = rnd.randint(0, 8)
        return position
    elif isWinner(board, player) or isWinner(board, opponent):  # Megnézzük, nem-e nyert már valaki, ha igen, akkor 0-t küldünk vissza pozícióként
        position = -1
        return position
    else:  # Megnézzük melyik cella szabad, és abból választunk logika alapján
        possibleMoves = emptyCells(board)

        for i in [player, opponent]:
            for j in possibleMoves:
                boardCopy = board[:]
                boardCopy[j] = i
                if isWinner(boardCopy, i):
                    position = j
                    return position

        if 4 in possibleMoves:
            position = 5
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


def isWinner(board, color):
    return ((board[6] == color and board[7] == color and board[8] == color) or
            (board[3] == color and board[4] == color and board[5] == color) or
            (board[0] == color and board[1] == color and board[2] == color) or
            (board[0] == color and board[3] == color and board[6] == color) or
            (board[1] == color and board[4] == color and board[7] == color) or
            (board[2] == color and board[5] == color and board[8] == color) or
            (board[0] == color and board[4] == color and board[8] == color) or
            (board[2] == color and board[4] == color and board[6] == color))


def emptyCells(board):
    possibleMoves = []
    j = 0
    for i in board:
        if i == 0:
            possibleMoves.append(j)
        j += 1
    return possibleMoves


def boardColorRecognition(msg):
    points = msg.split(";")
    for point in points:
        data = point.split(",")
        if data[1] > 168 and data[1] < 227:
            column = 0
        elif data[1] > 289 and data[1] < 349:
            column = 1
        elif data[1] > 410 and data[1] < 470:
            column = 2
        else:
            column = -1

        if data[2] > 96 and data[2] < 156:
            row = 0
        elif data[2] > 217 and data[2] < 277:
            row = 1
        elif data[2] > 338 and data[2] < 398:
            row = 2
        else:
            row = -1

        if row is not -1 and column is not -1:
            if data[0] == 'B':
                board[column*3+row] = 1
            elif data[0] == 'R':
                board[column*3+row] = 2
def find_figure(player):
    global bluePiecesPlaced
    global redPiecesPlaced
    if player == 1:
        figurePosition = bluePieces[bluePiecesPlaced]
        bluePiecesPlaced += 1
    elif player == 2:
        figurePosition = redPieces[redPiecesPlaced]
        redPiecesPlaced += 1
    else:
        figurePosition = [0, 0]

    return figurePosition


def kinematics_client_operation(coord, angle=0.0):
    rospy.wait_for_service('kinematics_handler')
    try:
        kinematics_handler = rospy.ServiceProxy('kinematics_handler', kinematicsMessage)
        response = kinematics_handler(coord[0], coord[1], coord[2], angle)
        return response.sum
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def gripper_operation(gripperAction):
    '''False = open, True = Close'''
    controller_name = "gripper_controller"
    joint_names = rospy.get_param("/%s/joints" % controller_name)
    rospy.loginfo("Joint names: %s" % joint_names)
    rate = rospy.Rate(10)

    trajectory_command = JointTrajectory()
    trajectory_command.header.stamp = rospy.Time.now()
    trajectory_command.joint_names = joint_names
    point = JointTrajectoryPoint()
    # Joint names: ['gripper']
    if (gripperAction): #true->close
        point.positions = [-0.1]
    else: #true->close
        point.positions = [0.2]
    point.velocities = [0.0]
    point.time_from_start = rospy.rostime.Duration(1, 0)
    trajectory_command.points = [point]

    pub.publish(trajectory_command)


def pick_or_place(position, gripperAction):
    '''gripperAction: False = open, True = Close'''
    retVal = kinematics_client_operation([position[0], position[1], pick_up_high], -math.pi/2)
    if not retVal:
        return "cannot reach position: " + position + " at pick_up_high"
    retVal = kinematics_client_operation([position[0], position[1], pick_up_low], -math.pi/2)
    if not retVal:
        return "cannot reach position: " + position + " at pick_up_low"
    gripper_operation(gripperAction)
    retVal = kinematics_client_operation([position[0], position[1], pick_up_high], -math.pi/2)
    if not retVal:
        return "cannot reach position: " + position + " at pick_up_high"
    return "Done"


def move(position, player):
    '''position: 0-8 position to place on the board; player: 1: blue, 2: red'''
    retVal = pick_or_place(find_figure(player), True)
    if retVal != "Done":
        return "Pick operation failed"
    retVal = pick_or_place(boardPosition[position], False)
    if retVal != "Done":
        return "Place operation failed"
    retVal = kinematics_client_operation([basePosition[0], basePosition[1], pick_up_high], -math.pi/2)
    if not retVal:
        return "Return to basePosition operation failed"
    return "Done"


if __name__ == "__main__":
    rospy.init_node('tic_tac_toe')
    pub = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=1)
    rospy.spin()

    player1 = randint(1,2)
        
    if player1 == 1: # kék
        player2 = 2 # piros
    else:
        player2 = 1

    player = player1

    for i in range(1, 8):
        
        board = boardColorRecognition()

        position = move_selector(board, player)

        if position == -1:
            break

        retVal = move(position, player)

        if retVal != "Done":
            break

        if player == player1:
            player = player2
        else:
            player = player1
        