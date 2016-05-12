from Board import Board
from Square import Square
import Enums
from Enums import Color

class Reversi:
	def __init__(self):
		self.board = Board()
		self.turn = Color.WHITE

	def start(self):
		# Print the board to start
		print self.board

		while(True):
			possibilities = self.board.getPlayableSquares(self.turn)
			if len(possibilities) == 0:
				self.endTurn()
				continue
			choice = possibilities[0]
			self.board.play(choice.x, choice.y, self.turn)
			self.endTurn()

	def endTurn(self):
		print self.board
		# raw_input("Continue?")
		self.turn = Enums.getOpposite(self.turn)

game = Reversi()
game.start()