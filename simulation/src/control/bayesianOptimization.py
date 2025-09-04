from skopt import Optimizer

opt = Optimizer([(0.0, 30.0), (0.0, 5.0), (0.0, 5.0)], acq_func="EI", random_state=0)
optDuel = Optimizer([(0.0, 30.0), (0.0, 5.0), (0.0, 5.0),(0.0, 30.0), (0.0, 5.0), (0.0, 5.0)], acq_func="LCB", random_state=0)
iterationDuel = 0
iteration = 0
def getNext():
    global iteration
    iteration += 1
    return opt.ask()

def updateResult(pid, cost):
    opt.tell(pid, cost)

def reset():
    global opt, iteration
    opt = Optimizer([(0.0, 30.0), (0.0, 5.0), (0.0, 5.0)], acq_func="EI", random_state=0)
    iteration = 0


def getNextDuel():
    global iterationDuel
    iterationDuel += 1
    return optDuel.ask()

def updateResultDuel(pid, cost):
    
    optDuel.tell(pid, cost)

def resetDuel():
    global optDuel, iterationDuel
    optDuel = Optimizer([(0.0, 30.0), (0.0, 5.0), (0.0, 5.0),(0.0, 30.0), (0.0, 5.0), (0.0, 5.0)], acq_func="LCB", random_state=0)
    iterationDuel = 0



