from casadi import *
import numpy as np

from PyCon.dynamics import *
from PyCon.ocp import *

import matplotlib
import matplotlib.pyplot as plt

# Model equations

A = DM([[-1 , 0.1  , 0. ],
        [0.1 ,-0.5 , -1  ],
        [0. , -1.  , -1 ]])

B = DM([ [1,1],
       [0,1],
       [0,0]])

x = MX.sym('x',3,1)
u = MX.sym('u',2,1)
t = MX.sym('t',1,1)

xdot = mtimes(A,x) + mtimes(B,u)
## Build dynamics obj
idyn = dynamics(t,x,u,xdot)
# If you want you can set a time span, if not the default time spam is np.linspace(0,1,100)
idyn.set_tspan(np.linspace(0,3,50))
idyn.SetIntegrator()

## Build OCP obj
PathCost   = 1e-5*dot(u,u)
FinalCost   = dot(x,x)
##
iocp = ocp(idyn,PathCost,FinalCost)


# Choose integrator 
iocp.functional.SetIntegrator()
# Set initial Condition
x0_num = [1,1,1]
# NPL = Nonlinear Programming
# Compile the NLP, where you fix the initial condition of your problem
iocp.BuildNLP(x0_num)


# Take a initial guess of control for optimisation
ut_guess = iocp.dynamics.ZerosControl()
# Solve NLP via IPOPT Optimizer
[u_opt,xt_opt] = iocp.SolveNLP(ut_guess)

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