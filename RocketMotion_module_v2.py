### Rocket Motion module ###

"""Here the set up of the general model is made including the optimisation tool to reach teh best trajectory
end point should be set at 200-400 km orbit """
###IMPORTS FROM MODULES###
import numpy as np
import matplotlib.pyplot as plt
import tudatpy.kernel.numerical_simulation as tp
from tudatpy.kernel.interface import spice_interface
from tudatpy.kernel.numerical_simulation import environment_setup
from tudatpy.kernel.numerical_simulation import propagation_setup
from tudatpy.kernel.astro import element_conversion
from tudatpy.kernel.constants import JULIAN_DAY
import tudatpy.kernel.numerical_simulation.propagation_setup.propagator
from tudatpy.kernel.numerical_simulation import estimation_setup

###IMPORTS OWN MODULES###
# IMPORT FROM SETUP
from SetUp import TargetAltitude, EngineSetUp, T_force_const, M_0, Isp_const, C_d0, A_ref, t_max_simulation, M_dry

# IMPORT FROM ENGINE MODULE
from Engine_module import Thrust_Direction, Thrust_Size_Falcon9, Isp_Function_Falcon9

### SETUP ENVIROMENT AND INTERFACE ###
# interface set up
spice_interface.load_standard_kernels()  # check what other states do and whether this is the best option

# start and end conditions
StartEpoch = 0.0
Altitude_termination = propagation_setup.propagator.dependent_variable_termination(
    dependent_variable_settings=propagation_setup.dependent_variable.altitude("ROCKET", 'Earth'),
    limit_value=TargetAltitude,
    use_as_lower_limit=False,
    terminate_exactly_on_final_condition=False
)

#stops as model is out of bound
Altitude_termination_stop = propagation_setup.propagator.dependent_variable_termination(
    dependent_variable_settings=propagation_setup.dependent_variable.altitude("ROCKET", 'Earth'),
    limit_value=-100,
    use_as_lower_limit=True,
    terminate_exactly_on_final_condition=False
)
TimeTermination = propagation_setup.propagator.time_termination(
    StartEpoch + t_max_simulation,
    terminate_exactly_on_final_condition=False,
)
Mass_Termination =  propagation_setup.propagator.dependent_variable_termination(
    dependent_variable_settings=propagation_setup.dependent_variable.body_mass("ROCKET"),
    limit_value=M_dry,
    use_as_lower_limit=True,
    terminate_exactly_on_final_condition=False

)
# todo add circular velocity termination term see get_termination_settings in LunarAscentUtilities

EndCondition = propagation_setup.propagator.hybrid_termination(
    [Altitude_termination, TimeTermination, Mass_Termination, Altitude_termination_stop],
    fulfill_single_condition=True
)

###DEFINE BODIES ###
BodiesToCreate = ["Earth"]
GlobalOrigin = BodiesToCreate[0]
GlobalOrientation = "J2000"
BodyToPropagate = ["ROCKET"]


body_settings = environment_setup.get_default_body_settings(BodiesToCreate, GlobalOrigin,
                                                            base_frame_orientation=GlobalOrientation)
body_settings.get("Earth").ephemeris_settings = environment_setup.ephemeris.constant(np.zeros(6), "SSB", "J2000")
bodies = environment_setup.create_system_of_bodies(body_settings)

### define the ROCKET ###
bodies.create_empty_body("ROCKET")
bodies.get_body("ROCKET").set_constant_mass(M_0)

### INITITAL STATE SETUP ###
InitialState_position = [spice_interface.get_average_radius("Earth"), 0, 0]  # cartesian X Y Z
Surface_Speed = spice_interface.get_average_radius("Earth")*(2*np.pi/JULIAN_DAY)
InitialState_velocities = [0, Surface_Speed, 0]
InitialState = np.array(InitialState_position + InitialState_velocities)  # contains the coordinates for the rocket and earth



#______________________END OF BODIES SETUP___________________

### SETUP ACCELERATION AND MASS RATE MODEL ###
"MODEL SHOULD HERE INTRODUCE ALL FORCES (THRUST, AERODYNAMIC AND GRAVITY FORCES) ACTING ON THE VEHICLES. " \
"FURTHERMORE THE MASS CHANGE OF THE VEHICLE IS ALSO INTRODUCED HERE"

    #SETUP THRUST MAGNITUDE
"currently set at levels of falcon 9 and no thrust variation"
from Engine_module import EngineThrustClass
"make thrust object form the fucntion which should cut of after a certain airspeed"
ThrustEngine = EngineThrustClass(BodyToPropagate[0],
                                 bodies,
                                 T_force_const,
                                 2000)


Thrust_Magnitude = propagation_setup.thrust.custom_thrust_magnitude(ThrustEngine.Thrust_size_class, Isp_Function_Falcon9)


    #SETUP THRUST DIRECTION
"""currently we only fly in x direction"""
ThrustDirection = propagation_setup.thrust.custom_thrust_direction(Thrust_Direction)

    #SETUP AERODYNAMIC FORCES
"CURRENTLY SET TO BE CONSTANT"
aero_coefficient_settings = environment_setup.aerodynamic_coefficients.constant(
    A_ref,
    [C_d0, 0, 0]
)
environment_setup.add_aerodynamic_coefficient_interface(
    bodies, BodyToPropagate[0], aero_coefficient_settings
)

    ##COMBINE AND SETUP TOTAL ACCELERATIONS MODEL##

acceleration_settings_on_ROCKET = dict(
    ROCKET=[propagation_setup.acceleration.thrust_from_direction_and_magnitude(
        thrust_magnitude_settings=Thrust_Magnitude,
        thrust_direction_settings=ThrustDirection
    )
    ],
    Earth=[propagation_setup.acceleration.point_mass_gravity(),
           propagation_setup.acceleration.aerodynamic()
           ]
)
acceleration_settings = dict(ROCKET=acceleration_settings_on_ROCKET)

    #SETUP MASS RATE
# setup mass propagation
MassRateSettings = dict(ROCKET=[propagation_setup.mass_rate.from_thrust()])



#_____________________END OF FORCE MODELS_______________________#

###SETUP PROPAGATIONS ###
# define variables to to store

dependent_variables_to_save = [
    propagation_setup.dependent_variable.altitude("ROCKET", "Earth"),  # 0th column
    propagation_setup.dependent_variable.relative_velocity("ROCKET", "Earth"), #1-3 colums
    propagation_setup.dependent_variable.airspeed("ROCKET", "Earth"), #4th colum
    propagation_setup.dependent_variable.total_acceleration("ROCKET"), #5th-7th column
    propagation_setup.dependent_variable.body_mass("ROCKET"), #8th column
    propagation_setup.dependent_variable.total_acceleration("ROCKET") # 9 -11 columns x y z accelerations

]

CentralBody = [GlobalOrigin]

    # SETUP ACCELERATION PROPAGATIONS
AccelerationModel = propagation_setup.create_acceleration_models(
    bodies,
    acceleration_settings,
    BodyToPropagate,
    CentralBody
)

AccelerationPropagationSettings = propagation_setup.propagator.translational(CentralBody,
                                                                 AccelerationModel,
                                                                 BodyToPropagate,
                                                                 InitialState,
                                                                 EndCondition,
                                                                 output_variables=dependent_variables_to_save
                                                                 )

    #SETUP MASS PORPAGATIONS
"currently we haven't figured it out if we can use this function"
MassRateModel = propagation_setup.create_mass_rate_models(bodies,
                                                          MassRateSettings,
                                                          AccelerationModel
                                                          )
MassPropagatorSettings = propagation_setup.propagator.mass(BodyToPropagate,
                                                           MassRateModel,
                                                           np.array([[M_0]]),
                                                           EndCondition
                                                           )

PropagationSettings = propagation_setup.propagator.multitype([AccelerationPropagationSettings, MassPropagatorSettings],
                                                             EndCondition,
                                                             dependent_variables_to_save)




###SETUP NUMERICAL SIMULATION ###

fixed_step_size = 1.0

IntergratorSettings = propagation_setup.integrator.runge_kutta_4(
    StartEpoch,
    fixed_step_size
)

### SIMULATION ###
DynamicsSimulator = tp.SingleArcSimulator(
    bodies, IntergratorSettings, PropagationSettings
)
# RETRIEVE OUTPUT VARIABLES

states = DynamicsSimulator.state_history #time and positions in the inertial frame
dependent_variables = DynamicsSimulator.dependent_variable_history #te defined variables



### DATA PROCESSING ###
time_seconds = list(states.keys())
carthersian_positions = np.vstack(list(states.values()))
dependent_variables_list = np.vstack(list(dependent_variables.values()))

    #dedicating variables
altitude_list = dependent_variables_list[:, 0]
mass_list = dependent_variables_list[:, 8]
y_displacement = carthersian_positions[:, 1]
x_displacement = carthersian_positions[:,0]
x_acceleration = dependent_variables_list[:, 9]
y_acceleration = dependent_variables_list[:, 10]

###DATA ANALYSIS###

plt.rcParams.update({'font.size': 20})
plt.figure(figsize=(20, 10))
plt.plot(time_seconds, altitude_list)
plt.xlabel("time [s]")
plt.ylabel("altitude [m]")

plt.figure(figsize=(20, 15))
plt.title("trajcetory in inertial frame")
plt.ylabel("altitude")
plt.xlabel("Y-plane translation")
plt.plot(carthersian_positions[:, 1], carthersian_positions[:,0])

plt.figure(figsize=(20,15))
plt.title("Velocities")
plt.xlabel("time")
plt.ylabel("velocities")
plt.plot(time_seconds, dependent_variables_list[:, 1], color='blue')    #x-velocity
plt.plot(time_seconds, dependent_variables_list[:, 2], color='green')   #y-velocity
plt.plot(time_seconds, dependent_variables_list[:, 3], color='red')     #z-velocity (should remain zero)
plt.plot(time_seconds, dependent_variables_list[:, 4], color="orange")  #relative airpseed

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

plt.figure(figsize=(19, 15))
plt.subplot(2,1,1)
plt.title("x-acc vs time")
plt.plot(time_seconds, x_acceleration)
plt.subplot(2,1,2)
plt.title("y-acc vs time")
plt.plot(time_seconds, y_acceleration)

plt.show()
