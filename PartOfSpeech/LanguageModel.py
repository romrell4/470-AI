import random

class LanguageModel:
    
    def __init__(self, fileName):
        self.data = open(fileName).read().split()
        self.model = self.getModel()
    
    def getWord(self, i):
        return self.data[i][:self.data[i].index("_")]

    def getModel(self):
        model = {}
        for i in range(len(self.data)):
            word = self.getWord(i)
            if i != len(self.data) - 1:
                nextWord = self.getWord(i + 1)
            else:
                nextWord = self.getWord(0)
            if word not in model:
                model[word] = []
            model[word].append(nextWord)
        return model

    def getText(self, length, seed):
        text = ""
        if seed in self.model: word = seed
        else: word = random.choice(self.model.keys())
        text += word
        for i in range(length):
            word = random.choice(self.model[word])
            text += " " + word
        return text

lm = LanguageModel("trainingData.txt")
print lm.getText(20, "The")

