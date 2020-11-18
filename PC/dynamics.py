from casadi import *
import numpy as np


class dynamics:
    t = []
    x = []
    u = []
    F = []
    xdot = []
    tspan = np.linspace(0,1,100)
    Ftraj = []
    x0 = []
    ut = []
    parent = []
    
    def __init__(self,t,x,u,xdot):

        self.x = x
        self.u = u
        self.t = t

        self.xdot = xdot
        F = Function('f', [t,x,u], [xdot])
        self.F = F

    def set_tspan(self,tspan):
        self.tspan = tspan
        self.x0 = []
        self.ut = []
        self.Ftraj = []
        print("When you set time span of dynamics, then you must use SetIntegrator() method.")
        if self.parent:
            self.parent.NLP = []
            self.parent.x0_num = []
            print("You must use BuildNLP() method in ocp obj")
            self.parent.functional.Cost = []
            print("You must use SetIntegrator() method in ocp.functional obj.")

    def SetIntegrator(self):
        
        Nt = len(self.tspan)
        ut = SX.sym('ut',self.ControlDim(),Nt-1)
        x0 = SX.sym('x0',self.StateDim(),1)
        self.x0 = x0
        self.ut = ut
        
        xt = [x0]

        for it in range(1,Nt):
            dt = self.tspan[it] - self.tspan[it-1]
            xnext = xt[it-1] + dt*(self.F(self.tspan[it-1],xt[it-1],ut[it-1]))
            xt.append(xnext)
        xt = hcat(xt)
        Ftraj = Function('Ftraj',[x0,ut],[xt])
        self.Ftraj = Ftraj

    def get_Nt(self):
        return len(self.tspan)

    def ControlDim(self):
        return self.u.size()[0]

    def StateDim(self):
        return self.x.size()[0]

    def ZerosControl(self):
        return np.zeros([self.ControlDim(), self.get_Nt()-1]) 
