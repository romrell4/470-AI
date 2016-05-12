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

		done = False

		while(True):
			#Check for all posssible locations
			possibilities = self.board.getPlayableSquares(self.turn)

			#Check if you had no possible locations
			if len(possibilities) == 0:
				#Check if the other player also couldn't play. If so, the game is over
				if done:
					return

				#Set the variable to notify the program that you couldn't go
				done = True
				self.endTurn()
				continue

			#Set the variable to notify the program that you could go	
			done = False

			#Get the best choice and play it
			choice = self.getChoice(possibilities)
			self.board.play(choice.x, choice.y, self.turn)
			self.endTurn()

	def getChoice(self, possibilities):
		#TODO: Add logic to choose the best possibility
		return possibilities[0]

	def endTurn(self):
		print self.board
		# raw_input("Continue?")
		self.turn = Enums.getOpposite(self.turn)

game = Reversi()
game.start()