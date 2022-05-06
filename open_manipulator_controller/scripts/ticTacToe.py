from random import randint


board = [0,0,0,0,0,0,0,0,0]
# A board egy lista a jelenlegi állással, amit színfelismerés alapján kapunk meg. 
# Ezek alapján dönt a robot arról, milyen lépést választ.
# 0 -üres cella, 1 - 1-es játékos (kék), 2 - 2-es játékos (piros)


def moveSelector(board, player):
    if board == [0,0,0,0,0,0,0,0,0]:
        position = randint(1,9)
        return position
    elif isWinner(board, player):
        position = 0
        return position
    else:
        possibleMoves = emptyCells(board)

        if player == 1:
            opponent = 2
        else:
            opponent = 1
    
        for i in [player, opponent]:
            for j in possibleMoves:
                boardCopy = board[:]
                boardCopy[j] = i
                if isWinner(boardCopy, i):
                    position = j
                    return position

        if 5 in possibleMoves:
            position = 5
            return position

        cornersOpen = []
        for i in possibleMoves:
            if i in [1,3,7,9]:
                cornersOpen.append(i)
                
        if len(cornersOpen) > 0:
            position = selectRandom(cornersOpen)
            return position

        edgesOpen = []
        for i in possibleMoves:
            if i in [2,4,6,8]:
                edgesOpen.append(i)
                
        if len(edgesOpen) > 0:
            position = selectRandom(edgesOpen)
            return position


def isWinner(board, color):
    return ((board[7] == color and board[8] == color and board[9] == color) or 
    (board[4] == color and board[5] == color and board[6] == color) or 
    (board[1] == color and board[2] == color and board[3] == color) or
    (board[1] == color and board[4] == color and board[7] == color) or
    (board[2] == color and board[5] == color and board[8] == color) or
    (board[3] == color and board[6] == color and board[9] == color) or
    (board[1] == color and board[5] == color and board[9] == color) or
    (board[3] == color and board[5] == color and board[7] == color))


def emptyCells(board):
    possibleMoves = []
    j = 0
    for i in board:
        if i==0:
            possibleMoves.append(j)
        j += 1
    return possibleMoves
    

def boardColorRecognition():
    return 0
    # Kell két alap pozíció, ahonnan látja a pályát és ahonnan keres a pálya mellől bábukat


def findFigure(color):
    return figurePosition


def pick(figurePosition):
    pass


def place(boardPosition):

    pass


def move(boardPosition, colorNumber):
    figurePosition = findFigure(colorNumber)
    pick(figurePosition)
    place(boardPosition)
    pass


if __name__ == "__main__":

    player1 = randint(1,2)
        
    if player1 == 1: # kék
        player2 = 2 # piros
    else:
        player2 = 1

    player = player1

    for i in range (1,8):
        
        board = boardColorRecognition()

        position = moveSelector(board, player)

        if position == 0:
            break

        move(position, player)

        if player == player1:
            player = player2
        else:
            player = player1
        