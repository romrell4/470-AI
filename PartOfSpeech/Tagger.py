from HMM import HMM, WORD, POS, UNKNOWN

class Tagger:
    def __init__(self):
        self.hmm = HMM()
        self.tests = {}

    def train(self, fileName):
        self.hmm.train(fileName)

    def test(self, fileName):
        data = open(fileName).read().split()
        observations = []
        partsofspeech = []
        for i in range(len(data)):
            tag = data[i].split("_")
            observations.append(tag[WORD])
            partsofspeech.append(tag[POS])
        self.tests[fileName] = {}
        self.tests[fileName]["text"] = observations
        self.tests[fileName]["pos_real"] = partsofspeech
        self.tests[fileName]["table"] = self.viterbi_table(observations)
        (self.tests[fileName]["prob"], self.tests[fileName]["pos_guess"]) = \
        self.optimal_tags(self.tests[fileName]["table"])
        self.tests[fileName]["accuracy"] = self.accuracy(fileName)

    def viterbi_table(self, observations):
        vt = []
        states = self.hmm.states()
        #print str(observations)
        for t in range(0, len(observations)):
            vt.append({})
            #print str(observations[t])
            for st in states:
                emis_prob = self.hmm.emission_probability(st, observations[t])
                #print "emis_prob " + str(st) + " " + str(observations[t]) + " = " + str(emis_prob)
                if t == 0: (max_prob, best_state) = (self.hmm.start_probability(st), None)
                else: (max_prob, best_state) = self.get_best_prob_and_state(t, st, vt)
                #print str(t) + " " + str(st) + " " + str(max_prob) + " " + str(emis_prob) + " " + str(best_state)
                vt[t][st] = {"prob": max_prob + emis_prob, "prev": best_state}
        return vt

    def get_best_prob_and_state(self, t, st, vt):
        states = self.hmm.states()
        best_state = None
        max_prob = -float("inf")
        for prev_st in states:
            prevProb = vt[t - 1][prev_st]["prob"]
            transProb = self.hmm.transition_probability(prev_st, st)
            product = prevProb + transProb
            if max_prob < product:
                max_prob = product
                best_state = prev_st
        #print str((max_prob, best_state))
        return (max_prob, best_state)

    def print_table(self, fileName):
        vt = self.tests[fileName]["table"]
        obs = self.tests[fileName]["text"]
        # Print a table of steps from dictionary
        print "   ",
        for i in range(len(vt)):
            print str(obs[i].ljust(7)),
        print ""
        for state in vt[0]:
            print "%s:" % state,
            for v in vt:
                print "%.7s" % ("%f" % v[state]["prob"]),
            print ""

    def optimal_tags(self, vt):
        opt = []
        # The highest probability
        max_prob = max(value["prob"] for value in vt[-1].values())
        previous = None
        # Get most probable state and its backtrack
        for st, data in vt[-1].items():
            if data["prob"] == max_prob:
                opt.append(st)
                previous = st
                break
        # Follow the backtrack till the first observation
        for t in range(len(vt) - 2, -1, -1):
            opt.insert(0, vt[t + 1][previous]["prev"])
            previous = vt[t][previous]["prev"]
        return (max_prob, opt)

    def accuracy(self, fileName):
        obs = self.tests[fileName]["text"]
        real = self.tests[fileName]["pos_real"]
        guess = self.tests[fileName]["pos_guess"]
        matches = 0
        for i in range(len(obs)):
            if real[i] == guess[i]: matches += 1
        return matches / float(len(obs))

    def output(self, fileName):
        opt = self.tests[fileName]["pos_guess"]
        max_prob = self.tests[fileName]["prob"]
        accuracy = self.tests[fileName]["accuracy"]
        print 'The steps of states are ' + ' '.join(opt) + \
              ' with highest probability of %s' % max_prob
        print fileName + ' tagged with %s accuracy' % accuracy

trainFile = "training_sample.txt"
testFile = "training_sample.txt"
#trainFile = "train.txt"
#testFile = "test.txt"
tagger = Tagger()
tagger.train(trainFile)
tagger.test(testFile)
#tagger.print_table(testFile)
tagger.output(testFile)
#print str(tagger.hmm.starts)
#print str(tagger.hmm.transitions[UNKNOWN])
#print str(tagger.hmm.emissions[UNKNOWN])

