class Color:
    EMPTY, BLACK, WHITE = range(3)
    str = ["Empty", "Black", "White"]
    chr = [u'\u0f1d', u'\u25cf', u'\u101d']
    #chr = ["-", "b", "w"]

class Direction:
    NW, N, NE, W, E, SW, S, SE = range(8)

SIZE = 8
MAX = SIZE - 1

def getOpposite(color):
	if color == Color.EMPTY:
		return Color.EMPTY
	elif color == Color.BLACK:
		return Color.WHITE
	else:
		return Color.BLACK

def getAlpha(pos):
    return chr(pos + ord('A'))

def getScore(x, y):
	if (x == 0 and y == 0) or \
		(x == 0 and y == MAX) or \
		(x == MAX and y == 0) or \
		(x == MAX and y == MAX):
		return SIZE
	elif (x == 0 and y == 1) or \
		(x == 1 and y == 0) or \
		(x == MAX - 1 and y == 0) or \
		(x == MAX and y == 1) or \
		(x == 0 and y == MAX - 1) or \
		(x == 1 and y == MAX) or \
		(x == MAX - 1 and y == MAX) or \
		(x == MAX and y == MAX - 1):
		return -SIZE / 4
	elif x == 0 or y == 0 or x == MAX or y == MAX:
		return SIZE / 2 - 1
	elif (x == 1 and y == 1) or \
		(x == MAX - 1 and y == 1) or \
		(x == 1 and y == MAX - 1) or \
		(x == MAX - 1 and y == MAX - 1):
		return 1
	elif x == 1 or y == 1 or x == MAX - 1 or y == MAX - 1:
		return SIZE / 4
	else:
		return 1

def getWeightedMax():
    total = 0
    for i in range(SIZE):
        for j in range(SIZE):
            total += getScore(i, j)
    return total

WEIGHT = getWeightedMax()