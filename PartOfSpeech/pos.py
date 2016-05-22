import math


class Viterbi:
    def train(self, fileName):
        data = open(fileName).read()

        stateCount = {}
        observations = []
        transitionProbabilities = {}
        emissionProbabilities = {}

        fullList = data.split()
        for i in range(len(fullList)):
            currentParts = fullList[i].split("_")

            key = currentParts[1]
            value = currentParts[0].lower()

            # Transition logic
            if i < len(fullList):
                futureParts = fullList[i + 1 if i != len(fullList) - 1 else 0].split("_")
                nextKey = futureParts[1]
                if key not in transitionProbabilities:
                    transitionProbabilities[key] = {}

                if nextKey not in transitionProbabilities[key]:
                    transitionProbabilities[key][nextKey] = 0
                transitionProbabilities[key][nextKey] += 1

            # Emission logic
            if key not in emissionProbabilities:
                emissionProbabilities[key] = {}
            if value not in emissionProbabilities[key]:
                emissionProbabilities[key][value] = 0
            emissionProbabilities[key][value] += 1

            # Observation logic
            # if value not in observations:
            observations.append(value)

            if key not in stateCount:
                stateCount[key] = 0
            stateCount[key] += 1

        self.states = stateCount.keys()
        self.startProbabilities = self.calculateStartProbabilities(stateCount)
        self.transitionProbabilities = self.calculateProbabilities(stateCount, transitionProbabilities)
        self.emissionProbabilities = self.calculateProbabilities(stateCount, emissionProbabilities)

        # print states
        # print observations
        # print startProbabilities
        # print transitionProbabilities
        # print emissionProbabilities

    def test(self, fileName):
        # Gather observations from new file
        observations = []
        self.viterbi(observations)

    def calculateStartProbabilities(self, stateCount):
        probs = {}
        for partOfSpeech in stateCount:
            probs[partOfSpeech] = 1 / float(len(stateCount))
            probs[partOfSpeech] = math.log10(probs[partOfSpeech])
        return probs

    def calculateProbabilities(self, stateCount, probs):
        for currentState in probs:
            # Add the unknown state
            probs[currentState][""] = 1

            # Divide to make the probability
            for nextState in probs[currentState]:
                probs[currentState][nextState] /= float(stateCount[currentState] + 1)
                probs[currentState][nextState] = math.log10(probs[currentState][nextState])
        return probs

    def viterbi(self, observations):
        V = [{}]

        # Set up our base case of V[0]
        for st in self.states:
            observation = observations[0] if observations[0] in self.emissionProbabilities[st] else ""
            V[0][st] = {"prob": self.startProbabilities[st] + self.emissionProbabilities[st][observation], "prev": None}

        # Run Viterbi when t > 0
        for t in range(1, len(observations)):
            print observations[t]
            V.append({})
            for st in self.states:
                (max_prob, best_state) = self.get_best_prob_and_state(t, st, self.states, self.transitionProbabilities,
                                                                      V)

                observation = observations[t] if observations[t] in self.emissionProbabilities[st] else ""
                V[t][st] = {"prob": max_prob + self.emissionProbabilities[st][observation], "prev": best_state}

        self.printTable(V)

        opt = []
        # The highest probability
        max_prob = max(value["prob"] for value in V[-1].values())
        previous = None
        # Get most probable state and its backtrack
        for st, data in V[-1].items():
            if data["prob"] == max_prob:
                opt.append(st)
                previous = st
                break
        # Follow the backtrack till the first observation
        for t in range(len(V) - 2, -1, -1):
            opt.insert(0, V[t + 1][previous]["prev"])
            previous = V[t][previous]["prev"]

        print 'The steps of states are ' + ' '.join(opt) + ' with highest probability of %s' % max_prob

    def get_best_prob_and_state(self, t, st, states, V):
        best_state = None
        max_prob = -float("inf")
        for prev_st in states:
            prevProb = V[t - 1][prev_st]["prob"]
            transProb = self.transitionProbabilities[prev_st][st if st in self.transitionProbabilities[prev_st] else ""]
            product = prevProb + transProb

            if max_prob < product:
                max_prob = product
                best_state = prev_st
        return (max_prob, best_state)

    def printTable(self, V):
        # Print a table of steps from dictionary
        # print " ".join(("%7d" % i) for i in range(len(V)))

        for i in range(len(V)):
            print "      " + str(i),
        print ""

        for state in V[0]:
            print "%s:" % state,
            for v in V:
                print "%.7s" % ("%f" % v[state]["prob"]),
            print ""


v = Viterbi()
v.train("train.txt")
v.test("test.txt")
