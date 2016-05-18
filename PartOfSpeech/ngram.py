import random

infilename = "textprep/2009-Obama.txt"
trainingdata = open(infilename).read()

contextconst = [""]

context = contextconst
model = {}

for word in trainingdata.split():
	model[str(context)] = model.setdefault(str(context),[]) + [word]
	context = (context+[word])[1:]

context = contextconst
for i in range(100):
	word = random.choice(model[str(context)])
	print (word, end= " ")
	context = (context+[word])[1:]

print()