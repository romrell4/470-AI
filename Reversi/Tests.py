from Board import Board
from Square import Square
import Enums
from Enums import Color, Direction

def endTurn(board, turn):
	print board
	raw_input("Continue?")
	return Enums.getOpposite(turn)

board = Board()
print board
turn = Color.BLACK
while(True):
	possibilities = board.getPlayableSquares(turn)
	if len(possibilities) == 0:
		turn = endTurn(board, turn)
		continue
	choice = possibilities[0]
	board.play(choice.x, choice.y, turn)
	turn = endTurn(board, turn)

