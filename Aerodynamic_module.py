### aerodynamic module ###
# this model computes the aerodynamic forces of the vehicle

import math as m
from SetUp import Airbreathing, MachBeta, C_l0
global Airbreathing, MachBeta, C_l0

def lift_force(MachNumber, Alltitude, alpha=0):
    #compute effect of angle of attack
    CL_alpha = C_l0 * (1+5*alpha)
    #compute effect of mach number
    if MachNumber<1.0:
        fm = CL_alpha / m.sqrt(1-MachNumber**2)
    elif MachNumber>=1.0:
        fm = CL_alpha *MachBeta * MachNumber    #todo find the function of lift dependent on mach number
    #get conditions of aircraft
    alltitudecondition = 25                     #todo get atmospheric conditions from tudat module and speed conditions
    #compute lift
    Lift = fm*alltitudecondition

    return Lift