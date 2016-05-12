from Board import Board
from Square import Square
import Enums
from Enums import Color, Direction

board = Board()
print board
squares = board.getPlayableSquares(Color.BLACK)
for square in squares:
	print square

