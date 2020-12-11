from casadi import *
import numpy as np
from PyCon.dynamics import *
from PyCon.ocp import *

import matplotlib
import matplotlib.pyplot as plt

# Declare model variables
x = MX.sym('x',2,1)
u = MX.sym('u',2,1)
t = MX.sym('t',1,1)
# Model equations
xdot = vertcat(    t*x[1]    , 
                   x[0]+u[0]           )

## Build dynamics obj
idyn = dynamics(t,x,u,xdot)
idyn.SetIntegrator()


## Build OCP obj
PathCost   = 1e-3*dot(u,u)
FinalCost   = dot(x,x) 

iocp = ocp(idyn,PathCost,FinalCost)

# Choose integrator 
iocp.functional.SetIntegrator()

# Set initial Condition
x0_num = [1,0]
# Compile the NLP, where you fix the initial condition of your problem

iocp.BuildNLP(x0_num)

