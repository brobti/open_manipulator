import random as rnd
from std_msgs.msg import String
import rospy

board = [0, 0, 0, 0, 0, 0, 0, 0, 0]


# A board egy lista a jelenlegi állással, amit színfelismerés alapján kapunk meg.
# Ezek alapján dönt a robot arról, milyen lépést választ.
# 0 -üres cella, 1 - 1-es játékos (kék), 2 - 2-es játékos (piros)


def moveSelector(board, player):
    if player == 1:
        opponent = 2
    else:
        opponent = 1

    if board == [0, 0, 0, 0, 0, 0, 0, 0, 0]:  # Ha üres, random választunk elsőre
        position = rnd.randint(0, 8)
        return position
    elif isWinner(board, player) or isWinner(board, opponent):  # Megnézzük, nem-e nyert már valaki, ha igen, akkor 0-t küldünk vissza pozícióként
        position = 0
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


"""def findFigure(color):
    return figurePosition"""


def pick(figurePosition):
    pass


def place(boardPosition):

    pass


def move(boardPosition, colorNumber):
    figurePosition = findFigure(colorNumber)
    pick(figurePosition)
    place(boardPosition)
    pass

def viewBoardMove():



if __name__ == "__main__":

    rospy.init_node('ticTacToe')
    player1 = randint(1,2)
        
    if player1 == 1: # kék
        player2 = 2 # piros
    else:
        player2 = 1

    player = player1

    for i in range (1,8):

        rospy.Subscriber('/color_recognition', String, boardColorRecognition, queue_size=1)

        position = moveSelector(board, player)

        if position == 0:
            break

        move(position, player)

        if player == player1:
            player = player2
        else:
            player = player1
        