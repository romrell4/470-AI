import math

WORD, POS = range(2)
UNKNOWN = '__'

class HMM:
    def __init__(self):
        self.sts = { UNKNOWN : 1 }
        self.obs = { UNKNOWN : 1 }
        self.starts = { UNKNOWN : 1 }
        self.transitions = { UNKNOWN : { UNKNOWN : 1 } }
        self.emissions = { UNKNOWN : { UNKNOWN : 1 } }
    
        self.startCount = 1
        self.wordCount = 0
        
    def train(self, fileName):
        data = open(fileName).read().split()

        for i in range(len(data)):
            self.wordCount += 1
            tag = data[i].split("_")
            pos = tag[POS]
            word = tag[WORD].lower()
            
            # States logic
            if pos not in self.sts:
                self.sts[pos] = 0
            self.sts[pos] += 1
            
            # Observations logic
            if word not in self.obs:
                self.obs[word] = 0
            self.obs[word] += 1
            
            # Starts Logic
            if i == 0 or data[i - 1].split("_")[POS] == '.':
                if pos not in self.starts:
                    self.starts[pos] = 0
                self.starts[pos] += 1
                self.startCount += 1

            # Transitions logic
            trans = data[i + 1].split("_")[POS] if i < len(data) - 1 else UNKNOWN
            if pos not in self.transitions:
                self.transitions[pos] = { UNKNOWN : 1 }
            if trans not in self.transitions[pos]:
                self.transitions[pos][trans] = 0
            self.transitions[pos][trans] += 1
            if trans not in self.transitions[UNKNOWN]:
                self.transitions[UNKNOWN][trans] = 0
            self.transitions[UNKNOWN][trans] += 1

            # Emissions logic
            if pos not in self.emissions:
                self.emissions[pos] = { UNKNOWN : 1 }
            if word not in self.emissions[pos]:
                self.emissions[pos][word] = 0
            self.emissions[pos][word] += 1
            if word not in self.emissions[UNKNOWN]:
                self.emissions[UNKNOWN][word] = 0
            self.emissions[UNKNOWN][word] += 1
    
    def states(self):
        return self.sts.keys()
        
    def observations(self):
        return self.obs.keys()
        
    def start_probability(self, pos):
        st = pos if pos in self.starts else UNKNOWN
        return math.log10( self.starts[st] / float(self.startCount) )

    def transition_probability(self, pos, next):
        st = pos if pos in self.transitions else UNKNOWN
        tr = next if next in self.transitions[st] else UNKNOWN
        div = self.wordCount if st == UNKNOWN else self.sts[st]
        return math.log10( self.transitions[st][tr] / float(self.sts[st]) )
        
    def emission_probability(self, pos, word):
        st = pos if pos in self.emissions else UNKNOWN
        obs = word.lower() if word.lower() in self.emissions[st] else UNKNOWN
        div = self.wordCount if st == UNKNOWN else self.sts[st]
        #print "emission_probability(" + str(pos) + ", " + str(word) + ")"
        #print str(self.emissions[st][obs]) + " / " + str(float(self.sts[st]))
        return math.log10( self.emissions[st][obs] / float(div) )

