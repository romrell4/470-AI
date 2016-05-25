import random

class LanguageModel:
    
    def __init__(self, fileName):
        self.data = open(fileName).read().split()
        self.model = self.get_model()
        self.transition_probabilities = self.get_transition_probabilities()
    
    def get_word(self, i):
        return self.data[i][:self.data[i].index("_")]

    def get_model(self):
        model = {}
        for i in range(len(self.data)):
            word = self.get_word(i)
            if i != len(self.data) - 1:
                nextWord = self.get_word(i + 1)
            else:
                nextWord = self.get_word(0)
            if word not in model:
                model[word] = []
            model[word].append(nextWord)
        return model

    def get_transition_probabilities(self):
        word_count = {}
        model = {}
        for i in range(len(self.data)):
            word = self.get_word(i)
            if word not in word_count:
                word_count[word] = 0
            word_count[word] += 1

            if i != len(self.data) - 1:
                next_word = self.get_word(i + 1)
            else:
                next_word = self.get_word(0)

            if word not in model:
                model[word] = {}
            if next_word not in model[word]:
                model[word][next_word] = 0
            model[word][next_word] += 1

        for word in model:
            for next_word in model[word]:
                model[word][next_word] /= float(word_count[word])
        return model



    def generate_sentence(self, length, seed):
        text = ""
        if seed in self.model: word = seed
        else: word = random.choice(self.model.keys())
        text += word
        for i in range(length):
            word = random.choice(self.model[word])
            text += " " + word
        return text

lm = LanguageModel("training_sample.txt")
# print lm.transition_probabilities
print lm.generate_sentence(20, "The")

