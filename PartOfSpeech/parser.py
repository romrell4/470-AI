fileName = "trainingData.txt"
# fileName = "test.txt"

data = open(fileName).read()

partsOfSpeech = {}

for mapping in data.split():
	parts = mapping.split("_")

	key = parts[1]
	value = parts[0]

	if key not in partsOfSpeech:
		partsOfSpeech[key] = {}

	if value not in partsOfSpeech[key]:
		partsOfSpeech[key][value] = 1
	else:
		partsOfSpeech[key][value] += 1
print partsOfSpeech
