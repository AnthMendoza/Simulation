from skopt import Optimizer
#######################################################
#just an exmaple not used. would need Python.h or pybind 
#######################################################
#PID search domain
space = [(0.1, 5.0),   # Kp
         (0.0, 2.0),   # Ki
         (0.0, 2.0)]   # Kd


opt = Optimizer(dimensions=space, acq_func="EI", random_state=0)


for i in range(10):
    next_pid = opt.ask()
    cost = #get cost 
    opt.tell(next_pid, cost)
