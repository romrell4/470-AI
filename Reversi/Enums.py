class Color:
    EMPTY, BLACK, WHITE = range(3)

class Direction:
    NW, N, NE, W, E, SW, S, SE = range(8)

def getOpposite(color):
	if color == Color.EMPTY:
		return Color.EMPTY
	elif color == Color.BLACK:
		return Color.WHITE
	else:
		return Color.BLACK

def getAlpha(pos):
    return chr(pos + ord('A'))