from skopt import Optimizer

opt = Optimizer([(0.0, 30.0), (0.0, 5.0), (0.0, 5.0)], acq_func="EI", random_state=0)
iteration = 0

def getNext():
    global iteration
    iteration += 1
    return opt.ask()

def updateResult(pid, cost):
    opt.tell(pid, cost)