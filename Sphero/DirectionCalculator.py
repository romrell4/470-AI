import math
from Point import Point

# Enter the true values here!
xMin = 50
yMin = 40
xMax = 750
yMax = 530

# I programmed this assuming that "location" would be a Location object with attributes x and y
# and that "options" would be an array of doubles representing angles
def getOptimalDirection(location, options):
    bestHeuristic = {"index": -1, "value": -1}
    for i in range(len(options)):
        heuristic = getHeuristicForDirection(location, options[i])
        if heuristic > bestHeuristic["value"]:
            bestHeuristic["index"] = i
            bestHeuristic["value"] = heuristic
    return (options[bestHeuristic["index"]], bestHeuristic["value"])

def getHeuristicForDirection(location, direction):
    (x, y) = getWallsForDirection(direction)
    xDist = yDist = float("inf")
    if x is not None:
        xDist = getDistanceToWallX(location, direction, x)
    if y is not None:
        yDist = getDistanceToWallY(location, direction, y)
    return min(xDist, yDist)

def getWallsForDirection(direction):
    direction %= 2 * math.pi
    if direction == 0:
        return xMax, None
    elif direction < math.pi / 2:
        return xMax, yMin
    elif direction == math.pi / 2:
        return None, yMin
    elif direction < math.pi:
        return xMin, yMin
    elif direction == math.pi:
        return xMin, None
    elif direction < 3 * math.pi / 2:
        return xMin, yMax
    elif direction == 3 * math.pi / 2:
        return None, yMax
    elif direction < 2 * math.pi:
        return xMax, yMax
    else:
        print "THIS IS A HUGE ERROR! DOUBLE CHECK YOUR LOGIC!"
        exit()

def getDistanceToWallX(location, direction, wallX):
    return math.fabs(math.fabs(location.x - wallX) / math.cos(direction))

def getDistanceToWallY(location, direction, wallY):
    return math.fabs(math.fabs(location.y - wallY) / math.sin(direction))
