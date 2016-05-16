class Color:
    EMPTY, BLACK, WHITE = range(3)
    opp = [EMPTY, WHITE, BLACK]
    str = ["Empty", "Black", "White"]
    chr = [[u'\u0f1d', u'\u25cf', u'\u101d'], # For white display
           [u'\u0f1d', u'\u101d', u'\u25cf'], # For black display
           ["-", "b", "w"]] # Universal display

class Direction:
    NW, N, NE, W, E, SW, S, SE = range(8)

def getAlpha(pos):
    return chr(pos + ord('A'))