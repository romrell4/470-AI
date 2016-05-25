from HMM import HMM, UNKNOWN
import random

class ProseWriter:
    def __init__(self):
        self.hmm = HMM()

    def train(self, fileName):
        self.hmm.train(fileName)
        
    def clear_unknowns(self):
        del self.hmm.sts[UNKNOWN]
        del self.hmm.obs[UNKNOWN]
        del self.hmm.starts[UNKNOWN]
        del self.hmm.transitions[UNKNOWN]
        for t in self.hmm.transitions:
            del self.hmm.transitions[t][UNKNOWN]
        del self.hmm.emissions[UNKNOWN]
        for e in self.hmm.emissions:
            del self.hmm.emissions[e][UNKNOWN]
        self.hmm.startCount -= 1

    def start_state(self):
        starts = self.hmm.starts
        count = random.randint(0, self.hmm.startCount - 1)
        for s in starts:
            if count == 0: return s
            count -= 1

    def next_state(self, curState):
        transitions = self.hmm.transitions[curState]
        count = random.randint(0, self.hmm.sts[curState] - 1)
        for t in transitions:
            for c in range(transitions[t]):
                if count == 0: return t
                count -= 1

    def next_word(self, nextState):
        emissions = self.hmm.emissions[nextState]
        count = random.randint(0, self.hmm.sts[nextState] - 1)
        for e in emissions:
            for c in range(emissions[e]):
                if count == 0: return e
                count -= 1

    def write(self, count):
        state = None
        for i in range(count):
            while state == None: state = self.start_state()
            #if i != 0 and state != '.': print " ",
            print self.next_word(state),
            state = self.next_state(state)
        print ""


pw = ProseWriter()
pw.train("testingData.txt")
pw.clear_unknowns()
pw.write(100)