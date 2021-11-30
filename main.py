### IMPORTS ###
# packages imports
import pygmo as pg
from tudatpy.kernel.numerical_simulation.propagation_setup import dependent_variable
import matplotlib.pyplot as plt
from numpy import array
# module imports
from RocketMotion_Module import SSTOAscent
from Arbitrary_Engine import FalconEngineGuided
from Falcon9_engine import Falcon9Engine
# set up or initial inputs imports
from SetUp import M_0, M_prop, M_dry, trajectory_evolutions, trajectory_populations

### CREATE ENGINE CLASS ###
# TODO | STATUS : HASN'T BEEN DEVELOPED YET
"""
THIS SHOULD BE THE OUTER CLASS OF THE ENGINE THAT TAKES THE DESIGN ASPECTS SET FOR THE ENGINE TO BE DETERMINED. CLASS
SHOULD GET FURTHER SIZING PARAMETERS UNDER "SIZING ENGINE CLASS", WERE IT IS LINKED TO THE ROCKET MOTION MODULE. 

THESE ASPECT CURRENTLY ARE : 
-FUEL TYPE
-OXIDISER TYPE
-ENGINE TYPE
-ENGINE COMBUSTION CYCLE TYPE
-AIRBREATHER (True/False)
"""

### CREATE ROCKET SIZING CLASS ###
# TODO | STATUS : HASN'T BEEN DEVELOPED YET
"""
blablabla
"""


#_____________________________________________SHOULD BE IN A LOOP_______________________________________________________
""""""
### CREATE ROCKET MOTION CLASS ###
#TODO | STATUS : COMPLETED
"""
CLASS THAT SET UPS THE TUDAT FROM THE SET UP FILE AND THE GIVEN INPUTS. IT USES THIS SET UP TO CREATE A PROBLEM CLASS 
THAT PYGMO CAN OPTIMISE FOR A GIVEN SET OF GUIDANCE PARAMETERS. THE AIM IS SUCH THAT THE MOST OPTIMAL TRAJECTORY CAN BE 
FOUND FOR THE SPECIFIED ASCENT VEHICLE.
ONCE THE MODEL HAS BEEN SET UP ONLY THE VEHICLE SPECIFICATIONS, THE THRUST, AND AERODYNAMIC MODULE SHOULD HAVE TO BE 
UPDATED. 

ASPECT THE CLASS CURRENTLY NEEDS: 
-NAME OF CURRENT ASCENT VEHICLE
-INITIAL MASS OF ASCENT VEHICLE
-PROPELLANT MASS OF ASCENT VEHICLE
-(THRUST MODULE)
-(AERODYNAMIC MODULE)
"""
Rocket_motion_module = SSTOAscent("ROCKET", M_0, M_prop)
#add output variables
Rocket_motion_module.add_outputs_to_save([dependent_variable.flight_path_angle(Rocket_motion_module.vehicle, Rocket_motion_module.Origin),
                                          dependent_variable.heading_angle(Rocket_motion_module.vehicle, Rocket_motion_module.Origin),
                                          dependent_variable.angle_of_attack(Rocket_motion_module.vehicle, Rocket_motion_module.Origin)
                                          ],
                                         ["Flight Path Angle", "heading angle", "angle of attack"])
    ## SIZING THE ENGINE CLASS ##
    # TODO | STATUS : THIS CLASS IS CURRENTLY HELD BY FALCON9_ENGINE (VV)
"""
    Thrust class should be linked to the bodies and rocket of the rocket motion module 
    """
Thrust_module = FalconEngineGuided(Rocket_motion_module.bodies,
                                   Rocket_motion_module.Rocket,
                                   M_0,
                                   M_prop)
    ## CREATING THE AERODYNAMIC CLASS
    # todo no such class exists at the moment
Aerodynamic_module = "to be developed"

    ## PROVIDE THE SIZED CLASSES TO THE ROCKET MOTION MODULE
Rocket_motion_module.set_thrust_model(ThrustMagnitude=Thrust_module.Thrust_Magnitude,
                                      Thrustguidance=Thrust_module.Thrust_Guidance,
                                      ThrustISP=Thrust_module.IspThrust,
                                      Thrust_class=Thrust_module
                                      )
Rocket_motion_module.set_aerodynamic_model()

    ## CREATE A PROBLEM CLASS FOR PYGMO
    #todo : this class should chang ethe mass for each iteration or the whole class should be created again
current_problem_class = Rocket_motion_module.get_problem_class(current_vehicle_M_0=M_0,
                                                               current_vehicle_M_prop=M_prop
                                                               )
print(current_problem_class)
### TRAJECTORY OPTIMISATION MODULE ###
"""
PYGMO MODULE USED TO OPTIMIZE THE TRAJECTORY FOR THE GIVEN CONDITIONS ON THE ENGINE AND ROCKET MOTION MODULE.

CURRENT ASPECT TO USE:
-GUIDANCE PARAMETERS
-(THRUST ON/OFF (ENGINE NUMBER))
-(PROP. MASS FLOW)
"""
current_problem = pg.problem(current_problem_class)
algorithm = pg.algorithm(pg.de())
current_population = pg.population(current_problem,
                           size= trajectory_populations
                           )

cham_list = []
cham_list_x = []
fit_x = [current_population.get_x()]
fit_y = [current_population.get_f()]
for i in range(trajectory_evolutions):
    # Evolve the population
    current_population = algorithm.evolve(current_population)
    print(current_population)
    # Store the fitness values for all individuals in a list
    cham_list.append(current_population.champion_f)
    cham_list_x.append(current_population.champion_x)
    fit_x.append(current_population.get_x())
    fit_y.append(current_population.get_f())
    print(f'Evolving population; at generation {i}')

print(current_population.champion_f, current_population.champion_x)
#_____________________________________________SHOULD BE IN A LOOP_______________________________________________________
Rocket_motion_module.run_simulation(current_population.champion_x)




colors = [ "orange", "aqua", "yellow"]
for i in range(3):
    plt.plot(Rocket_motion_module.time_range, Rocket_motion_module.current_output_variables[:, 10+i], color=colors[i])
print(Rocket_motion_module.Outputs_names[-3::])
print(colors)


def main():
    pass


if __name__ == '__main__':
    main()
printgraphs = True
if printgraphs == True:
    time_seconds = Rocket_motion_module.time_range
    carthersian_positions = Rocket_motion_module.current_states
    dependent_variables_list = Rocket_motion_module.current_output_variables

    # dedicating variables
    altitude_list = dependent_variables_list[:, 0]
    mass_list = dependent_variables_list[:, 8]
    y_displacement = carthersian_positions[:, 1]
    x_displacement = carthersian_positions[:, 0]
    x_acceleration = dependent_variables_list[:, 9]
    y_acceleration = dependent_variables_list[:, 10]
    import matplotlib.pyplot as plt

    plt.rcParams.update({'font.size': 20})

    plt.figure(figsize=(20, 15))
    plt.subplot(2, 2, 1)
    plt.title("The optimal trajectory")
    plt.ylabel("velocity [m/s]")
    plt.xlabel("time [s]")
    plt.plot(time_seconds, dependent_variables_list[:, 1], color='blue')    #x-velocity
    plt.plot(time_seconds, dependent_variables_list[:, 2], color='green')   #y-velocity
    plt.plot(time_seconds, dependent_variables_list[:, 3], color='red')     #z-velocity (should remain zero)
    plt.plot(time_seconds, dependent_variables_list[:, 4], color="orange")  #relative airpseed
    plt.subplot(2,2,2)
    plt.title("Altitude")
    plt.plot(time_seconds, altitude_list/1E3)
    plt.ylabel("altitude [km]")
    plt.xlabel("time [s]")
    plt.subplot(2,2,3)
    plt.title("mass")
    plt.ylabel("mass [tons]")
    plt.xlabel("time [s]")
    plt.plot(time_seconds, mass_list/1E6)
    plt.subplot(2,2,4)
    plt.title("Trajectory")
    plt.plot(y_displacement/1E3, (x_displacement-x_displacement[0])/1E3)
    plt.ylabel("x-position [km]")
    plt.xlabel("y-position [km]")

    plt.figure(figsize=(20, 15))
    plt.title("maximum altitude for each generation")
    plt.ylabel("altitude [km]")
    plt.xlabel("generation number [-]")
    plt.plot(range(trajectory_evolutions), -array(cham_list)/1E3)

print(current_population.champion_x, current_population.champion_f)
from SetUp import guidance_nodes, time_nodes
print(Thrust_module.guidance_angles)

last_parameters = [round(elem, 2) for elem in current_population.champion_x]
print( "angles", last_parameters[:guidance_nodes])
print("time steps", last_parameters[-time_nodes::])
plt.show()