###MAIN SCRIPT ####
'''script should call upon all  modules and combine those to optimize the mass of the system by finding the msot optimal trajectory'''

### IMPORTED PACKAGES###
import pygmo as pg
from tudatpy.kernel.interface import spice_interface
from tudatpy.kernel.numerical_simulation import propagation_setup
from tudatpy.kernel.constants import  JULIAN_DAY
from numpy import pi, array, vstack, sqrt

###START CODE###

from RocketMotion_Module_v3 import SSTOAscent
from SetUp import M_0, M_prop, M_dry #TODO the initial mass and mass imput should come from the sizing module




###SETUP  ROCKET MOTION MOTION MODULE
    #GIVE VARIABLES TO SAVE
dependent_variables_to_save = [
    propagation_setup.dependent_variable.altitude("ROCKET", "Earth"),  # 0th column
    propagation_setup.dependent_variable.relative_velocity("ROCKET", "Earth"), #1-3 colums
    propagation_setup.dependent_variable.airspeed("ROCKET", "Earth"), #4th colum
    propagation_setup.dependent_variable.total_acceleration("ROCKET"), #5th-7th column
    propagation_setup.dependent_variable.body_mass("ROCKET"), #8th column
    propagation_setup.dependent_variable.total_acceleration("ROCKET"), # 9 -11 columns x y z accelerations
    propagation_setup.dependent_variable.periapsis_altitude("ROCKET", "Earth") #12 colum

]


Current_ascent = SSTOAscent("ROCKET", M_0, M_prop, dependent_variables_to_save)

###Testing new engine class

from Falcon9_engine import Falcon9Engine
from Falcon9_engine_v1 import EngineThrustClass, Isp_Function_Falcon9, Thrust_Direction


Thrust_model = Falcon9Engine(M_0,M_dry, Current_ascent.bodies, Current_ascent.Rocket)
Current_ascent.set_thrust_model(ThrustMagnitude=Thrust_model.ThrustSize,
                                Thrustguidance=Thrust_model.Thrust_Guidance,
                                ThrustISP=Thrust_model.IspThrust,
                                Thrust_class=Thrust_model)

###PROVIDE CURRENT ENGINE MODULE TO ROCKET MOTION MODULE###
    #TODO : STATUS : INCOMPLETE (STILL HAS TO BE DEVELOPED)
"""currently use the falcon9 as illustrational purpose"""
from Falcon9_engine_v1 import EngineThrustClass, Isp_Function_Falcon9, Thrust_Direction
from SetUp import T_force_const
ThrustEngine = EngineThrustClass(Current_ascent.vehicle,
                                 Current_ascent.bodies,
                                 T_force_const,
                                 2000)


###PROVIDE AERODYNAMIC MODEL TO ROCKET MOTION MODULE
    #TODO | STATUS : INCOMPLETE (must become a custom class which still ahs to be made)
Current_ascent.set_aerodynamic_model()


#TESTING
InitialState_position = [spice_interface.get_average_radius("Earth"), 0, 0]  # cartesian X Y Z
Surface_Speed = spice_interface.get_average_radius("Earth")*(2*pi/JULIAN_DAY)
InitialState_velocities = [0, Surface_Speed, 0]
InitialState = array(InitialState_position + InitialState_velocities)
Current_ascent.set_initial_state(InitialState)
Current_ascent.set_terminationConditions()


### PYGMO OPTIMILISATION
import pygmo
population_size = 50

Current_ascent.get_acceleration_settings()
Current_ascent.get_mass_rate_settings()
Current_ascent.get_propagation_model()
Current_ascent.Status_setup = True

prob = pygmo.problem(Current_ascent)
algo = pygmo.algorithm(pygmo.de())
pop = pygmo.population(prob, size= population_size)

print(pop.get_f())
print("next")
print(pop.champion_f, pop.champion_x)




printgraphs = False

if printgraphs == True:


    time_seconds = Current_ascent.time_range
    carthersian_positions = Current_ascent.current_states
    dependent_variables_list = Current_ascent.current_output_variables

        #dedicating variables
    altitude_list = dependent_variables_list[:, 0]
    mass_list = dependent_variables_list[:, 8]
    y_displacement = carthersian_positions[:, 1]
    x_displacement = carthersian_positions[:,0]
    x_acceleration = dependent_variables_list[:, 9]
    y_acceleration = dependent_variables_list[:, 10]

    ###DATA ANALYSIS###
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.size': 20})
    #figure 1
    plt.figure(figsize=(20, 10))
    plt.plot(time_seconds, altitude_list)
    plt.xlabel("time [s]")
    plt.ylabel("altitude [m]")

    #figure 2
    plt.figure(figsize=(20, 15))
    plt.title("trajcetory in inertial frame")
    plt.ylabel("altitude")
    plt.xlabel("Y-plane translation")
    plt.plot(carthersian_positions[:, 1], carthersian_positions[:,0])

    #figure 3
    plt.figure(figsize=(20,15))
    plt.title("Velocities")
    plt.xlabel("time")
    plt.ylabel("velocities")
    plt.plot(time_seconds, dependent_variables_list[:, 1], color='blue')    #x-velocity
    plt.plot(time_seconds, dependent_variables_list[:, 2], color='green')   #y-velocity
    plt.plot(time_seconds, dependent_variables_list[:, 3], color='red')     #z-velocity (should remain zero)
    plt.plot(time_seconds, dependent_variables_list[:, 4], color="orange")  #relative airpseed

    #figure 4
    plt.figure(figsize=(20, 15))
    plt.subplot(2, 2, 1)
    plt.title("Velocities")
    plt.plot(time_seconds, dependent_variables_list[:, 1], color='blue')    #x-velocity
    plt.plot(time_seconds, dependent_variables_list[:, 2], color='green')   #y-velocity
    plt.plot(time_seconds, dependent_variables_list[:, 3], color='red')     #z-velocity (should remain zero)
    plt.plot(time_seconds, dependent_variables_list[:, 4], color="orange")  #relative airpseed
    plt.subplot(2,2,2)
    plt.title("altitude over time")
    plt.plot(time_seconds, altitude_list/1E3)
    plt.subplot(2,2,3)
    plt.title("mass over time")
    plt.plot(time_seconds, mass_list/1E6)
    plt.subplot(2,2,4)
    plt.title("movement in Inertial frame")
    plt.plot(y_displacement/1E3, (x_displacement-x_displacement[0])/1E3)

    #figure 5
    plt.figure(figsize=(19, 15))
    plt.subplot(2,1,1)
    plt.title("x-acc vs time")
    plt.plot(time_seconds, x_acceleration)
    plt.subplot(2,1,2)
    plt.title("y-acc vs time")
    plt.plot(time_seconds, y_acceleration)

    #figure 6
    plt.figure(figsize=(19,15))

    plt.plot(time_seconds, dependent_variables_list[:, 12]/1000, color="aqua")
    plt.ylim(380, 420)
    plt.show()
