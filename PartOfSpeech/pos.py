import math


def train(fileName):
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
        if value not in observations:
            observations.append(value)

        if key not in stateCount:
            stateCount[key] = 0
        stateCount[key] += 1

    states = stateCount.keys()
    startProbabilities = calculateStartProbabilities(stateCount)
    transitionProbabilities = calculateProbabilities(stateCount, transitionProbabilities)
    emissionProbabilities = calculateProbabilities(stateCount, emissionProbabilities)

    # print states
    # print observations
    # print startProbabilities
    # print transitionProbabilities
    # print emissionProbabilities

    viterbi(states, observations, startProbabilities, transitionProbabilities, emissionProbabilities)


def calculateStartProbabilities(stateCount):
    probs = {}
    for partOfSpeech in stateCount:
        probs[partOfSpeech] = 1 / float(len(stateCount))
    return probs


def calculateProbabilities(stateCount, probs):
    for currentState in probs:
        # Add the unknown state
        probs[currentState][""] = 1

        # Divide to make the probability
        for nextState in probs[currentState]:
            probs[currentState][nextState] /= float(stateCount[currentState] + 1)
        # probs[currentState][nextState] = math.log10(probs[currentState][nextState])
    return probs


def ourViterbi(states, observations, startProbabilities, transitionProbabilities, emissionProbabilities):
    print "Our Viterbi"


def viterbi(states, observations, startProbabilities, transitionProbabilities, emissionProbabilities):
    V = [{}]
    for st in states:
        # print "We are checking state: " + st
        # print "We are using the word: " + observations[0]
        # print "The emissionProbability for this state is: " + str(emissionProbabilities[st])
        observation = observations[0] if observations[0] in emissionProbabilities[st] else ""
        # print "Observation: " + observation
        V[0][st] = {"prob": startProbabilities[st] * emissionProbabilities[st][observation], "prev": None}
    # print "V[0][" + st + "] = " + str(V[0][st])
    # print
    # Run Viterbi when t > 0

    for t in range(1, len(observations)):
        # print "Starting on word " + str(t) + " which is: " + observations[t]
        V.append({})
        for st in states:
            # print "State: " + st

            max_tr_prob = 0
            for prev_st in states:
                prevProb = V[t-1][prev_st]["prob"]
                transProb = transitionProbabilities[prev_st][st if st in transitionProbabilities[prev_st] else ""]
                if max_tr_prob < prevProb * transProb:
                    max_tr_prob = prevProb * transProb


            for prev_st in states:
                state = st if st in transitionProbabilities[prev_st] else ""
                if V[t - 1][prev_st]["prob"] * transitionProbabilities[prev_st][state] == max_tr_prob:
                    observation = observations[t] if observations[t] in emissionProbabilities[st] else ""
                    max_prob = max_tr_prob * emissionProbabilities[st][observation]
                    V[t][st] = {"prob": max_prob, "prev": prev_st}
                    break
                # print
    for line in dptable(V):
        print line
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


def dptable(V):
    # Print a table of steps from dictionary
    yield " ".join(("%7d" % i) for i in range(len(V)))
    for state in V[0]:
        yield "%.7s: " % state + " ".join("%.7s" % ("%f" % v[state]["prob"]) for v in V)


train("test.txt")
# test("test.txt")
