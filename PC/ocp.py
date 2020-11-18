from casadi import *
import numpy as np

class ocp:

    dynamics = []
    functional = []
    NLP = []
    x0_num = []

    def __init__(self,dynamics,PathCostStep,FinalCost):
        self.dynamics = dynamics
        self.dynamics.parent = self

        self.functional = functional(dynamics,PathCostStep,FinalCost)

    def BuildNLP(self,x0_num):

        Nt   = self.dynamics.get_Nt()
        ut   = self.dynamics.ut
        Udim = self.dynamics.ControlDim()
        Cost = self.functional.Cost

        xt = self.dynamics.Ftraj(x0_num,ut)
        

        ut_flat = reshape(ut,Udim*(Nt-1),1)

        nlp = {'x':ut_flat, 'f':Cost(xt,ut), 'g':[]}
        self.NLP = nlpsol('S', 'ipopt', nlp)
        self.x0_num = x0_num


    def SolveNLP(self,u_guess):

        if not self.NLP:
            raise Exception("Must be execute BuildNLP() method of ocp class.")
        
        Udim    = self.dynamics.ControlDim()
        Nt      = self.dynamics.get_Nt()
        u_guess = reshape(u_guess,Udim*(Nt-1),1)

        ###############################
        result  = self.NLP(x0=u_guess)
        ###############################

        u_opt = result['x']
        u_opt = reshape(u_opt,Nt-1,Udim)
        u_opt = u_opt.T

        xt_opt = self.dynamics.Ftraj(self.x0_num,u_opt)
        return [u_opt,xt_opt]

    def ZerosControl(self):
        return self.dynamics.ZerosControl()





################################################################################################################
################################################################################################################
################################################################################################################
################################################################################################################
################################################################################################################
################################################################################################################

class functional():

    PathCostStep = []
    FinalCost = []
    Cost = []
    dynamics = []
    
    def __init__(self,dynamics,PathCostStep,FinalCost):

        t = dynamics.t 
        x = dynamics.x 
        u = dynamics.u 

        self.PathCostStep = Function('L',[t,x,u],[PathCostStep])
        self.FinalCost    = Function('Psi',[x],[FinalCost])
        self.dynamics     = dynamics

    def SetIntegrator(self):

        if  isinstance(self.dynamics.ut,list):
            raise Exception("Must be execute SetIntegrator() method of dynamics class.")
        
        Nt       = self.dynamics.get_Nt()
        tspan    = self.dynamics.tspan
        ut       = self.dynamics.ut
        # create a new symbolical var for trajectory
        xt       = SX.sym('xt',self.dynamics.StateDim(),Nt)

        # Riemann Sum
        Lsum = 0

        ut = ut.T
        xt = xt.T 

        for it in range(0,Nt-1):
            dt = tspan[it+1] - tspan[it]
            Lsum = Lsum + dt*self.PathCostStep(tspan[it],xt[it,:],ut[it,:])
        Cost = Lsum + self.FinalCost(xt[Nt-1,:])

        self.Cost = Function('Cost',[xt.T,ut.T],[Cost])