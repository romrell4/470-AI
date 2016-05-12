from Board import Board
from Square import Square
import Enums
from Enums import Color, Direction

board = Board()
print board
possibilities = board.getPlayableSquares(Color.BLACK)
choice = possibilities[0]
board.play(choice.x, choice.y, Color.BLACK)
print board

