from casadi import *
import numpy as np

from PyCon.dynamics import *
from PyCon.ocp import *

import matplotlib
import matplotlib.pyplot as plt

# Model equations

A = DM([[-1 , 0.1  , 0. ],
        [0.1 ,-0.5 , -0.5  ],
        [0. , -0.1, -5.0 ]] )

B = DM([ [1],
       [1],
       [1]])

x = MX.sym('x',3,1)
u = MX.sym('u',1,1)
t = MX.sym('t',1,1)

xdot = mtimes(A,x) + mtimes(B,u)
## Build dynamics obj
idyn = dynamics(t,x,u,xdot)
# If you want you can set a time span, if not the default time spam is np.linspace(0,1,100)
idyn.set_tspan(np.linspace(0,3,100))
idyn.SetIntegrator()


## Build OCP obj
PathCost   =  0 
FinalCost   = dot(x,x)
##
iocp = ocp(idyn,PathCost,FinalCost)

g = u
iocp.SetControlConstraints(g)

# Choose integrator 
iocp.functional.SetIntegrator()
# Set initial Condition
x0_num = [1,1,1]
# NPL = Nonlinear Programming
# Compile the NLP, where you fix the initial condition of your problem
iocp.BuildNLP(x0_num)

# Take a initial guess of control for optimisation
ut_guess = iocp.dynamics.ZerosControl()

gt_low = -1.5
gt_up  = +0.5

# Solve NLP via IPOPT Optimizer
[u_opt,xt_opt] = iocp.SolveNLP(ut_guess,gt_low,gt_up)

## PLOTS
fig, ax = plt.subplots(3,1)

Nt    = idyn.get_Nt()
tspan = idyn.tspan

ax[0].plot(tspan,xt_opt.T)
ax[0].set_title('Opt State')

xt_free = iocp.dynamics.Ftraj(x0_num,u_opt*0 )

ax[2].plot(tspan,xt_free.T)
ax[2].set_title('Free State')

ax[1].plot(tspan[0:-1], u_opt.T)
ax[1].set_title('Control')
plt.show()