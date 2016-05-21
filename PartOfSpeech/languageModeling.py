import random

def getWord(token):
	return token[:token.index("_")]

infilename = "trainingData.txt"
data = open(infilename).read()


model = {}
data = data.split()
for i in range(len(data)):
	word = getWord(data[i])
	if i != len(data) - 1:
		nextWord = getWord(data[i+1])

	if word not in model:
		model[word] = []
	model[word].append(nextWord)
	# print word
# print model

prevWord = getWord(data[0])
print model[prevWord]
for i in range(100):
	print prevWord + " ",
	prevWord = random.choice(model[prevWord])


# contextconst = [""]

# context = contextconst
# model = {}

# for word in trainingdata.split():
# 	model[str(context)] = model.setdefault(str(context),[]) + [word]
# 	context = (context+[word])[1:]

# context = contextconst
# for i in range(100):
# 	word = random.choice(model[str(context)])
# 	print (word, end= " ")
# 	context = (context+[word])[1:]

# print()