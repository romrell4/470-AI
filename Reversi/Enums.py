class Color:
    EMPTY, BLACK, WHITE = range(3)
    str = ["Empty", "Black", "White"]
    chr = [u'\u0f1d', u'\u25cf', u'\u101d'] # For white display
    #chr = [u'\u0f1d', u'\u101d', u'\u25cf'] # For black display
    #chr = ["-", "b", "w"] # Universal display

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

DEPTH = 5